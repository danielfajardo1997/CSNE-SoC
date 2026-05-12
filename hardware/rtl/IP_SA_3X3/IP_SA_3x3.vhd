-------------------------------------------------------------------------------
-- IP_SA_3x3.vhd
-- Avalon-MM Slave Wrapper for SA_TOP_3x3 (3x3 Systolic MAC Array)
--
-- Description:
--   Memory-mapped interface (Avalon-MM) for the SA_TOP_3x3 module.
--   Allows a processor (HPS ARM Cortex-A9 via LW-HPS2FPGA bridge) to:
--      - Write the 6 input operands (3 x a_row, 3 x b_col) as INT8
--      - Trigger one MAC operation via the start register
--      - Flush all 9 PE accumulators via the control register
--      - Poll the done/ready status flag
--      - Read the 9 x 32-bit accumulator results (matrix C[r][c])
--
--   For a full 3x3 matrix multiply C = A x B, the HPS performs 3 MAC
--   operations (one per k-slice of the shared dimension):
--     For k = 0, 1, 2:
--       Write a_row* = A[:,k]  (column k of matrix A)
--       Write b_col* = B[k,:]  (row    k of matrix B)
--       Write start register   (triggers one accumulation)
--       Poll status until done = 1
--     Read 9 accumulators = matrix C
--
-- Address Map (byte addresses, word-aligned 32-bit):
--   Offset  | Access | Description
--   --------|--------|--------------------------------------------------
--   0x00    | W      | a_row0[7:0]  -- A[0,k] (row 0, k-th column of A)
--   0x04    | W      | a_row1[7:0]  -- A[1,k]
--   0x08    | W      | a_row2[7:0]  -- A[2,k]
--   0x0C    | W      | b_col0[7:0]  -- B[k,0] (col 0, k-th row of B)
--   0x10    | W      | b_col1[7:0]  -- B[k,1]
--   0x14    | W      | b_col2[7:0]  -- B[k,2]
--   0x18    | W      | Control: bit0 = flush (clears all 9 accumulators)
--   0x1C    | W      | Start: any write triggers one MAC operation
--   0x20    | R      | Status: bit0 = done (o_done from SA_TOP_3x3)
--   0x24    | R      | acc_00 -- C[0][0] (32-bit signed result)
--   0x28    | R      | acc_01 -- C[0][1]
--   0x2C    | R      | acc_02 -- C[0][2]
--   0x30    | R      | acc_10 -- C[1][0]
--   0x34    | R      | acc_11 -- C[1][1]
--   0x38    | R      | acc_12 -- C[1][2]
--   0x3C    | R      | acc_20 -- C[2][0]
--   0x40    | R      | acc_21 -- C[2][1]
--   0x44    | R      | acc_22 -- C[2][2]
--   0x48+   | --     | Reserved
--
-- Author  : Daniel G. Fajardo Lopez
-- Date    : 2026-05-11
-- Version : 1.1  – added done_latch register: converts 1-cycle done pulse
--                   into a held flag readable by HPS over LW bridge
-------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY IP_SA_3x3 IS
    PORT (
        -- Avalon-MM slave interface
        clock        : IN  std_logic;
        reset        : IN  std_logic;                      -- Active-high reset
        PE_write     : IN  std_logic_vector(7 DOWNTO 0);  -- Byte-enable (bit0=write active)
        PE_address   : IN  std_logic_vector(31 DOWNTO 0); -- Byte address
        PE_writedata : IN  std_logic_vector(31 DOWNTO 0); -- Write data
        PE_readdata  : OUT std_logic_vector(31 DOWNTO 0)  -- Read data
    );
END ENTITY IP_SA_3x3;

ARCHITECTURE avalonMMslave OF IP_SA_3x3 IS

    ---------------------------------------------------------------------------
    -- Component: SA_TOP_3x3
    ---------------------------------------------------------------------------
    COMPONENT SA_TOP_3x3 IS
        GENERIC (
            G_MULT_CYCLES : positive := 5;
            G_SIGN_CYCLES : positive := 2;
            G_SUM_CYCLES  : positive := 3
        );
        PORT (
            i_clk    : IN  std_logic;
            i_rst_n  : IN  std_logic;
            i_start  : IN  std_logic;
            i_flush  : IN  std_logic;
            o_done   : OUT std_logic;
            i_a_row0 : IN  std_logic_vector(7 DOWNTO 0);
            i_a_row1 : IN  std_logic_vector(7 DOWNTO 0);
            i_a_row2 : IN  std_logic_vector(7 DOWNTO 0);
            i_b_col0 : IN  std_logic_vector(7 DOWNTO 0);
            i_b_col1 : IN  std_logic_vector(7 DOWNTO 0);
            i_b_col2 : IN  std_logic_vector(7 DOWNTO 0);
            o_acc_00 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_01 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_02 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_10 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_11 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_12 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_20 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_21 : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_22 : OUT std_logic_vector(31 DOWNTO 0)
        );
    END COMPONENT SA_TOP_3x3;

    ---------------------------------------------------------------------------
    -- Write-active (bit0 of byte-enable — same as IP_PE v1.1)
    ---------------------------------------------------------------------------
    SIGNAL write_active : std_logic;

    -- Address bits [6:2] select the word register (32 slots of 4 bytes each)
    SIGNAL addr_word    : std_logic_vector(6 DOWNTO 2);

    ---------------------------------------------------------------------------
    -- Write-enable decode
    ---------------------------------------------------------------------------
    SIGNAL wr_en_a_row0 : std_logic;
    SIGNAL wr_en_a_row1 : std_logic;
    SIGNAL wr_en_a_row2 : std_logic;
    SIGNAL wr_en_b_col0 : std_logic;
    SIGNAL wr_en_b_col1 : std_logic;
    SIGNAL wr_en_b_col2 : std_logic;
    SIGNAL wr_en_ctrl   : std_logic;
    SIGNAL wr_en_start  : std_logic;

    -- Rising-edge pulse for start (mirrors IP_PE pattern)
    SIGNAL prev_wr_en_start  : std_logic;
    SIGNAL pulse_wr_en_start : std_logic;

    ---------------------------------------------------------------------------
    -- Internal data registers
    ---------------------------------------------------------------------------
    SIGNAL sig_a_row0   : std_logic_vector(7 DOWNTO 0);
    SIGNAL sig_a_row1   : std_logic_vector(7 DOWNTO 0);
    SIGNAL sig_a_row2   : std_logic_vector(7 DOWNTO 0);
    SIGNAL sig_b_col0   : std_logic_vector(7 DOWNTO 0);
    SIGNAL sig_b_col1   : std_logic_vector(7 DOWNTO 0);
    SIGNAL sig_b_col2   : std_logic_vector(7 DOWNTO 0);
    SIGNAL sig_flush    : std_logic;

    ---------------------------------------------------------------------------
    -- Outputs from SA_TOP_3x3
    ---------------------------------------------------------------------------
    SIGNAL sig_done      : std_logic;
    SIGNAL sig_done_latch: std_logic;  -- holds done pulse until next start
    SIGNAL sig_acc_00 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_01 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_02 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_10 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_11 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_12 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_20 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_21 : std_logic_vector(31 DOWNTO 0);
    SIGNAL sig_acc_22 : std_logic_vector(31 DOWNTO 0);

    -- Active-low reset for SA_TOP_3x3
    SIGNAL sig_rst_n  : std_logic;

BEGIN

    ---------------------------------------------------------------------------
    -- Signal extraction
    ---------------------------------------------------------------------------
    write_active <= PE_write(0);
    addr_word    <= PE_address(6 DOWNTO 2);
    sig_rst_n    <= NOT reset;

    ---------------------------------------------------------------------------
    -- Write-enable decoders (word-aligned addresses)
    ---------------------------------------------------------------------------
    wr_en_a_row0 <= '1' WHEN (write_active = '1' AND addr_word = "00000") ELSE '0'; -- 0x00
    wr_en_a_row1 <= '1' WHEN (write_active = '1' AND addr_word = "00001") ELSE '0'; -- 0x04
    wr_en_a_row2 <= '1' WHEN (write_active = '1' AND addr_word = "00010") ELSE '0'; -- 0x08
    wr_en_b_col0 <= '1' WHEN (write_active = '1' AND addr_word = "00011") ELSE '0'; -- 0x0C
    wr_en_b_col1 <= '1' WHEN (write_active = '1' AND addr_word = "00100") ELSE '0'; -- 0x10
    wr_en_b_col2 <= '1' WHEN (write_active = '1' AND addr_word = "00101") ELSE '0'; -- 0x14
    wr_en_ctrl   <= '1' WHEN (write_active = '1' AND addr_word = "00110") ELSE '0'; -- 0x18
    wr_en_start  <= '1' WHEN (write_active = '1' AND addr_word = "00111") ELSE '0'; -- 0x1C

    ---------------------------------------------------------------------------
    -- Operand registers (8-bit LSB of writedata)
    ---------------------------------------------------------------------------
    REG_A_ROW0: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN sig_a_row0 <= (OTHERS => '0');
        ELSIF rising_edge(clock) THEN
            IF wr_en_a_row0 = '1' THEN sig_a_row0 <= PE_writedata(7 DOWNTO 0); END IF;
        END IF;
    END PROCESS REG_A_ROW0;

    REG_A_ROW1: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN sig_a_row1 <= (OTHERS => '0');
        ELSIF rising_edge(clock) THEN
            IF wr_en_a_row1 = '1' THEN sig_a_row1 <= PE_writedata(7 DOWNTO 0); END IF;
        END IF;
    END PROCESS REG_A_ROW1;

    REG_A_ROW2: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN sig_a_row2 <= (OTHERS => '0');
        ELSIF rising_edge(clock) THEN
            IF wr_en_a_row2 = '1' THEN sig_a_row2 <= PE_writedata(7 DOWNTO 0); END IF;
        END IF;
    END PROCESS REG_A_ROW2;

    REG_B_COL0: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN sig_b_col0 <= (OTHERS => '0');
        ELSIF rising_edge(clock) THEN
            IF wr_en_b_col0 = '1' THEN sig_b_col0 <= PE_writedata(7 DOWNTO 0); END IF;
        END IF;
    END PROCESS REG_B_COL0;

    REG_B_COL1: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN sig_b_col1 <= (OTHERS => '0');
        ELSIF rising_edge(clock) THEN
            IF wr_en_b_col1 = '1' THEN sig_b_col1 <= PE_writedata(7 DOWNTO 0); END IF;
        END IF;
    END PROCESS REG_B_COL1;

    REG_B_COL2: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN sig_b_col2 <= (OTHERS => '0');
        ELSIF rising_edge(clock) THEN
            IF wr_en_b_col2 = '1' THEN sig_b_col2 <= PE_writedata(7 DOWNTO 0); END IF;
        END IF;
    END PROCESS REG_B_COL2;

    ---------------------------------------------------------------------------
    -- Flush register (level held until overwritten — bit0 of ctrl word)
    ---------------------------------------------------------------------------
    REG_FLUSH: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN sig_flush <= '0';
        ELSIF rising_edge(clock) THEN
            IF wr_en_ctrl = '1' THEN sig_flush <= PE_writedata(0); END IF;
        END IF;
    END PROCESS REG_FLUSH;

    ---------------------------------------------------------------------------
    -- Start edge detector — single-cycle pulse on first write assertion
    ---------------------------------------------------------------------------
    EDGE_DETECT_START: PROCESS(clock, reset) BEGIN
        IF reset = '1' THEN
            prev_wr_en_start  <= '0';
            pulse_wr_en_start <= '0';
        ELSIF rising_edge(clock) THEN
            prev_wr_en_start  <= wr_en_start;
            IF wr_en_start = '1' AND prev_wr_en_start = '0' THEN
                pulse_wr_en_start <= '1';
            ELSE
                pulse_wr_en_start <= '0';
            END IF;
        END IF;
    END PROCESS EDGE_DETECT_START;

    ---------------------------------------------------------------------------
    -- Done latch register
    -- Set  : on the rising edge of sig_done (1-cycle pulse from SA_TOP_3x3)
    -- Clear: on reset OR on a new start pulse (ready for next operation)
    --
    -- NOTE: No auto-clear on read. The HPS polling loop may read the status
    -- register multiple times before seeing done=1. If we cleared on the
    -- first read, subsequent reads would see 0 and the poll would never exit.
    -- The latch is cleared only when the next start pulse arrives, which the
    -- driver always sends before polling — guaranteeing a clean state.
    ---------------------------------------------------------------------------
    DONE_LATCH : PROCESS (clock, reset)
    BEGIN
        IF reset = '1' THEN
            sig_done_latch <= '0';
        ELSIF rising_edge(clock) THEN
            IF pulse_wr_en_start = '1' THEN
                -- New operation: pre-clear before pipeline runs
                sig_done_latch <= '0';
            ELSIF sig_done = '1' THEN
                -- Pipeline complete: latch and hold until next start
                sig_done_latch <= '1';
            END IF;
        END IF;
    END PROCESS DONE_LATCH;

    ---------------------------------------------------------------------------
    -- SA_TOP_3x3 instantiation
    ---------------------------------------------------------------------------
    u_SA_TOP_3x3 : SA_TOP_3x3
        GENERIC MAP (G_MULT_CYCLES => 5, G_SIGN_CYCLES => 2, G_SUM_CYCLES => 3)
        PORT MAP (
            i_clk    => clock,       i_rst_n  => sig_rst_n,
            i_start  => pulse_wr_en_start,
            i_flush  => sig_flush,   o_done   => sig_done,
            i_a_row0 => sig_a_row0,  i_a_row1 => sig_a_row1,  i_a_row2 => sig_a_row2,
            i_b_col0 => sig_b_col0,  i_b_col1 => sig_b_col1,  i_b_col2 => sig_b_col2,
            o_acc_00 => sig_acc_00,  o_acc_01 => sig_acc_01,  o_acc_02 => sig_acc_02,
            o_acc_10 => sig_acc_10,  o_acc_11 => sig_acc_11,  o_acc_12 => sig_acc_12,
            o_acc_20 => sig_acc_20,  o_acc_21 => sig_acc_21,  o_acc_22 => sig_acc_22
        );

    ---------------------------------------------------------------------------
    -- Read multiplexer
    ---------------------------------------------------------------------------
    READ_MUX: PROCESS(addr_word, sig_done_latch,
                      sig_acc_00, sig_acc_01, sig_acc_02,
                      sig_acc_10, sig_acc_11, sig_acc_12,
                      sig_acc_20, sig_acc_21, sig_acc_22)
    BEGIN
        CASE addr_word IS
            WHEN "01000" => PE_readdata <= (0 => sig_done_latch, OTHERS => '0'); -- 0x20 status
            WHEN "01001" => PE_readdata <= sig_acc_00;  -- 0x24 C[0][0]
            WHEN "01010" => PE_readdata <= sig_acc_01;  -- 0x28 C[0][1]
            WHEN "01011" => PE_readdata <= sig_acc_02;  -- 0x2C C[0][2]
            WHEN "01100" => PE_readdata <= sig_acc_10;  -- 0x30 C[1][0]
            WHEN "01101" => PE_readdata <= sig_acc_11;  -- 0x34 C[1][1]
            WHEN "01110" => PE_readdata <= sig_acc_12;  -- 0x38 C[1][2]
            WHEN "01111" => PE_readdata <= sig_acc_20;  -- 0x3C C[2][0]
            WHEN "10000" => PE_readdata <= sig_acc_21;  -- 0x40 C[2][1]
            WHEN "10001" => PE_readdata <= sig_acc_22;  -- 0x44 C[2][2]
            WHEN OTHERS  => PE_readdata <= (OTHERS => '0');
        END CASE;
    END PROCESS READ_MUX;

END ARCHITECTURE avalonMMslave;