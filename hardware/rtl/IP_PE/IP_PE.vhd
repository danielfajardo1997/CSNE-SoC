-------------------------------------------------------------------------------
-- IP_PE.vhd
-- Avalon-MM Slave Wrapper for PE_TOP (Processing Element Top)
--
-- Description:
--   This wrapper implements a memory-mapped interface (Avalon-MM) for the
--   PE_TOP module. It allows a processor (e.g., Nios II, ARM) to:
--      - Write 8-bit operands A and B (using LSB of writedata)
--      - Start computation via a control register
--      - Read the 32-bit accumulator result
--      - Poll status flags (ready)
--      - Assert flush signal
--
-- Avalon-MM byte-enable note:
--   The Avalon-MM specification requires that the write-enable signal be
--   either a single bit (write) or a byte-enable bus whose width equals
--   data_width / 8.  For a 32-bit data bus this gives a 4-bit byteenable,
--   but Platform Designer also accepts an 8-bit variant when the interface
--   is declared with byteenable_width = 8.
--
--   In this file PE_write is declared as std_logic_vector(7 downto 0).
--   The active condition mirrors the single-bit version:
--     write is asserted  <=>  PE_write(0) = '1'
--   All eight bits are treated uniformly: only bit 0 is used as the
--   master write-enable qualifier, which is the standard minimal
--   implementation when the IP does not support partial-word writes.
--   Bits 7 downto 1 are intentionally ignored (reserved for future use).
--
-- Address Map (byte addresses, word-aligned 32-bit offsets):
--   Offset (decimal) | Access | Description
--   -----------------+--------+-------------------------------------------
--   0                | W      | Write A[7:0]  -> loads a_in  of PE_TOP
--   1                | W      | (reserved)
--   2                | W      | Write B[7:0]  -> loads b_in  of PE_TOP
--   3                | W      | (reserved)
--   4                | W      | Control register: bit0 = flush
--   5                | W      | (reserved)
--   6                | W      | Start computation (any value starts the PE)
--   7                | W      | (reserved)
--   8                | W      | (reserved)
--   9                | R      | Read flag_ready  (from PE_TOP.ip_ready)
--   10               | R      | Read acc_out     (32-bit signed result)
--   11               | R      | Read a_out       (8-bit forwarded A, zero-extended)
--   12               | R      | Read b_out       (8-bit forwarded B, zero-extended)
--   13-15            | R/W    | (reserved)
--
-- Author  : Doctoral Researcher Daniel Fajardo
-- Date    : 2026-05-06
-- Version : 1.1  – PE_write changed from std_logic to std_logic_vector(7:0)
--                   to comply with Avalon-MM byte-enable bus width rules.
--                   Internal write-enable logic unchanged; bit 0 is used
--                   as the active write qualifier.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity IP_PE is
    port (
        -- ----------------------------------------------------------------
        -- Avalon-MM slave interface
        -- ----------------------------------------------------------------
        clock          : in  std_logic;
        reset          : in  std_logic;

        -- Write byte-enable bus (8 bits to satisfy Avalon-MM bus-width rule).
        -- Only bit 0 is used as the write-active qualifier; bits 7:1 are
        -- reserved and ignored by this implementation.
        PE_write       : in  std_logic_vector(7 downto 0);

        PE_address     : in  std_logic_vector(31 downto 0); -- Byte address
        PE_writedata   : in  std_logic_vector(31 downto 0); -- Write data
        PE_readdata    : out std_logic_vector(31 downto 0)  -- Read data
    );
end entity IP_PE;

architecture avalonMMslave of IP_PE is

    ---------------------------------------------------------------------------
    -- Component declaration for PE_TOP
    ---------------------------------------------------------------------------
    component PE_TOP is
        port (
            clock      : in  std_logic;
            reset      : in  std_logic;
            start      : in  std_logic;
            a_in       : in  std_logic_vector(7 downto 0);
            b_in       : in  std_logic_vector(7 downto 0);
            flush      : in  std_logic;
            ip_ready   : out std_logic;
            acc_out    : out std_logic_vector(31 downto 0);
            a_out      : out std_logic_vector(7 downto 0);
            b_out      : out std_logic_vector(7 downto 0)
        );
    end component;

    ---------------------------------------------------------------------------
    -- Internal write-enable qualifier
    --   Extracted from the byte-enable bus so that the rest of the logic
    --   is identical to the single-bit version.  If the Avalon master
    --   asserts any byte lane (bit 0 = '1') we treat the transaction as
    --   a full 32-bit write, which is correct for this 32-bit slave.
    ---------------------------------------------------------------------------
    signal write_active : std_logic;

    ---------------------------------------------------------------------------
    -- Internal signals for PE_TOP connection
    ---------------------------------------------------------------------------
    signal sig_flush_reg : std_logic;
    signal sig_a_in      : std_logic_vector(7 downto 0);
    signal sig_b_in      : std_logic_vector(7 downto 0);
    signal sig_ready     : std_logic;
    signal sig_acc_out   : std_logic_vector(31 downto 0);
    signal sig_a_out     : std_logic_vector(7 downto 0);
    signal sig_b_out     : std_logic_vector(7 downto 0);

    ---------------------------------------------------------------------------
    -- Write-enable decode signals (one per register / address)
    ---------------------------------------------------------------------------
    signal wr_en_A     : std_logic;
    signal wr_en_B     : std_logic;
    signal wr_en_ctrl  : std_logic;
    signal wr_en_start : std_logic;

    ---------------------------------------------------------------------------
    -- Rising-edge pulse signals (one cycle wide; used to latch data)
    ---------------------------------------------------------------------------
    signal pulse_wr_en_A     : std_logic;
    signal pulse_wr_en_B     : std_logic;
    signal pulse_wr_en_start : std_logic;

    ---------------------------------------------------------------------------
    -- Previous-cycle shadow registers (used for edge detection)
    ---------------------------------------------------------------------------
    signal prev_wr_en_A     : std_logic;
    signal prev_wr_en_B     : std_logic;
    signal prev_wr_en_start : std_logic;

    ---------------------------------------------------------------------------
    -- Address decode helper (lower 4 bits of the byte address bus)
    ---------------------------------------------------------------------------
    signal addr_nibble : std_logic_vector(3 downto 0);

begin

    ---------------------------------------------------------------------------
    -- Write-active extraction
    --   Bit 0 of the byte-enable vector is the canonical write indicator.
    --   This single assignment is the only place the vector nature of
    --   PE_write is visible; the rest of the architecture uses write_active.
    ---------------------------------------------------------------------------
    write_active <= PE_write(0);

    ---------------------------------------------------------------------------
    -- Address nibble extraction
    --   Only the four LSBs are needed to decode offsets 0 .. 15.
    ---------------------------------------------------------------------------
    addr_nibble <= PE_address(3 downto 0);

    ---------------------------------------------------------------------------
    -- Write-enable decoders
    --   Each signal is high for exactly as long as the Avalon master holds
    --   the write strobe and the matching address.
    ---------------------------------------------------------------------------
    wr_en_A     <= '1' when (write_active = '1' and addr_nibble = "0000") else '0'; -- offset  0
    wr_en_B     <= '1' when (write_active = '1' and addr_nibble = "0010") else '0'; -- offset  2
    wr_en_ctrl  <= '1' when (write_active = '1' and addr_nibble = "0100") else '0'; -- offset  4
    wr_en_start <= '1' when (write_active = '1' and addr_nibble = "0110") else '0'; -- offset  6

    ---------------------------------------------------------------------------
    -- Rising-edge detector for operand A write
    --   Generates a guaranteed single-cycle pulse on the first cycle that
    --   wr_en_A is asserted.  The pulse is used to latch sig_a_in so that
    --   back-to-back writes to the same address only latch once.
    ---------------------------------------------------------------------------
    EDGE_DETECT_A : process(clock, reset)
    begin
        if reset = '1' then
            prev_wr_en_A  <= '0';
            pulse_wr_en_A <= '0';
        elsif rising_edge(clock) then
            prev_wr_en_A  <= wr_en_A;
            if wr_en_A = '1' and prev_wr_en_A = '0' then
                pulse_wr_en_A <= '1';
            else
                pulse_wr_en_A <= '0';
            end if;
        end if;
    end process EDGE_DETECT_A;

    ---------------------------------------------------------------------------
    -- Rising-edge detector for operand B write
    ---------------------------------------------------------------------------
    EDGE_DETECT_B : process(clock, reset)
    begin
        if reset = '1' then
            prev_wr_en_B  <= '0';
            pulse_wr_en_B <= '0';
        elsif rising_edge(clock) then
            prev_wr_en_B  <= wr_en_B;
            if wr_en_B = '1' and prev_wr_en_B = '0' then
                pulse_wr_en_B <= '1';
            else
                pulse_wr_en_B <= '0';
            end if;
        end if;
    end process EDGE_DETECT_B;

    ---------------------------------------------------------------------------
    -- Rising-edge detector for start write
    ---------------------------------------------------------------------------
    EDGE_DETECT_START : process(clock, reset)
    begin
        if reset = '1' then
            prev_wr_en_start  <= '0';
            pulse_wr_en_start <= '0';
        elsif rising_edge(clock) then
            prev_wr_en_start  <= wr_en_start;
            if wr_en_start = '1' and prev_wr_en_start = '0' then
                pulse_wr_en_start <= '1';
            else
                pulse_wr_en_start <= '0';
            end if;
        end if;
    end process EDGE_DETECT_START;

    ---------------------------------------------------------------------------
    -- Flush register
    --   Bit 0 of the 32-bit writedata word is stored as the flush flag.
    --   The register retains its value until overwritten; the PE_TOP flush
    --   input therefore stays asserted until the HPS explicitly clears it.
    ---------------------------------------------------------------------------
    FLUSH_REG : process(clock, reset)
    begin
        if reset = '1' then
            sig_flush_reg <= '0';
        elsif rising_edge(clock) then
            if wr_en_ctrl = '1' then
                sig_flush_reg <= PE_writedata(0);  -- bit 0 = flush
            end if;
        end if;
    end process FLUSH_REG;

    ---------------------------------------------------------------------------
    -- Operand A register
    --   Latched on the rising edge of pulse_wr_en_A to avoid glitches from
    --   the combinational writedata bus.
    ---------------------------------------------------------------------------
    REG_OPERAND_A : process(clock, reset)
    begin
        if reset = '1' then
            sig_a_in <= (others => '0');
        elsif rising_edge(clock) then
            if pulse_wr_en_A = '1' then
                sig_a_in <= PE_writedata(7 downto 0);  -- 8 LSBs only
            end if;
        end if;
    end process REG_OPERAND_A;

    ---------------------------------------------------------------------------
    -- Operand B register
    ---------------------------------------------------------------------------
    REG_OPERAND_B : process(clock, reset)
    begin
        if reset = '1' then
            sig_b_in <= (others => '0');
        elsif rising_edge(clock) then
            if pulse_wr_en_B = '1' then
                sig_b_in <= PE_writedata(7 downto 0);  -- 8 LSBs only
            end if;
        end if;
    end process REG_OPERAND_B;

    ---------------------------------------------------------------------------
    -- PE_TOP instantiation
    ---------------------------------------------------------------------------
    u_PE_TOP : PE_TOP
        port map (
            clock     => clock,
            reset     => reset,
            start     => pulse_wr_en_start,
            a_in      => sig_a_in,
            b_in      => sig_b_in,
            flush     => sig_flush_reg,
            ip_ready  => sig_ready,
            acc_out   => sig_acc_out,
            a_out     => sig_a_out,
            b_out     => sig_b_out
        );

    ---------------------------------------------------------------------------
    -- Read multiplexer
    --   Purely combinational; PE_readdata is valid one cycle after the
    --   master presents a stable address (zero wait-state read protocol).
    ---------------------------------------------------------------------------
    READ_MUX : process(addr_nibble, sig_ready, sig_acc_out, sig_a_out, sig_b_out)
    begin
        case addr_nibble is
            when "1001" =>   -- offset 9  : status – bit 0 = ip_ready
                PE_readdata <= (0 => sig_ready, others => '0');

            when "1010" =>   -- offset 10 : acc_out (32-bit signed result)
                PE_readdata <= sig_acc_out;

            when "1011" =>   -- offset 11 : a_out (8-bit, zero-extended to 32)
                PE_readdata <= std_logic_vector(resize(unsigned(sig_a_out), 32));

            when "1100" =>   -- offset 12 : b_out (8-bit, zero-extended to 32)
                PE_readdata <= std_logic_vector(resize(unsigned(sig_b_out), 32));

            when others =>
                PE_readdata <= (others => '0');
        end case;
    end process READ_MUX;

end architecture avalonMMslave;