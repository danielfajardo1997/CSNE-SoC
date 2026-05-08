
-- =============================================================================
-- Module      : PE_OnChip_Tester
-- Project     : Processing Element (PE) – On-Chip Verification
-- File        : PE_OnChip_Tester.vhd
--
-- Description :
--   Hardware FSM that replicates the five stimulus sequences defined in
--   tb_IP_PE.vhd, but runs entirely on an FPGA without a simulator.
--   The module drives the Avalon-MM slave interface of IP_PE directly,
--   captures the acc_out result for each test, compares it against a
--   pre-loaded expected value, and reports pass/fail on individual LEDs
--   or a 5-bit bus.
--
--   External pins required on the FPGA board:
--     i_clk    – board oscillator (assumed 50 MHz; adjust C_CLK_FREQ_HZ)
--     i_rst_n  – active-low push-button reset (debounced externally or via
--                the C_DEBOUNCE_CYCLES generic)
--     i_start  – push-button to begin the test sequence (active-high pulse)
--
--   All other signals are internal and connect directly to IP_PE.
--
-- Interface to IP_PE (Avalon-MM, byte-addressed, 32-bit):
--   Offset 0x0  W  : operand A (8-bit, sign-extended by IP)
--   Offset 0x2  W  : operand B (8-bit, sign-extended by IP)
--   Offset 0x4  W  : control  – bit0 = flush
--   Offset 0x6  W  : start computation (any write value)
--   Offset 0x9  R  : flag_ready  (bit 0)
--   Offset 0xA  R  : acc_out     (32-bit signed result)
--   Offset 0xB  R  : a_out       (8-bit forwarded, zero-extended)
--   Offset 0xC  R  : b_out       (8-bit forwarded, zero-extended)
--
-- Test vectors (match tb_IP_PE.vhd exactly):
--   #  A      B     Expected acc_out
--   1    5    3      15
--   2   -2    4      -8
--   3   10   -3     -30
--   4  127    1     127
--   5 -128   -1     128
--
-- Output status:
--   o_test_pass(4:0) – bit N is '1' if test N+1 passed (acc == expected)
--   o_test_fail(4:0) – bit N is '1' if test N+1 failed
--   o_all_pass       – '1' when all 5 tests passed
--   o_busy           – '1' while the tester is running
--   o_done           – single-cycle pulse when the sequence finishes
--
-- Design decisions:
--   1. Single-process FSM (state + output registers) avoids combinational
--      glitches on the Avalon bus – important for reliable edge detection
--      inside IP_PE.
--   2. All outputs to IP_PE are registered; the combinational address
--      decoder inside IP_PE sees only stable, glitch-free signals.
--   3. A configurable READY_TIMEOUT watchdog (C_READY_TIMEOUT_CYCLES)
--      prevents the tester from hanging if IP_PE never asserts ip_ready.
--   4. Signed comparison on acc_out is performed with std_logic_vector
--      using a 32-bit signed constant to avoid numeric_std pitfalls.
--   5. The input push-button is debounced in hardware using a saturating
--      counter (length set by generic G_DEBOUNCE_CYCLES).
--
-- Standards : VHDL-2008, IEEE Std 1076-2008
--             Naming convention (VSG / IEC 61508):
--               ports    -> i_<name> / o_<name>
--               signals  -> s_<name>
--               types    -> t_<name>
--               constants-> C_<NAME>
--               generics -> G_<NAME>
--
-- Author  : DAniel Fajardo
-- Date    : 2026-05-07
-- Version : 1.0
-- =============================================================================
 
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
 
-- =============================================================================
ENTITY PE_OnChip_Tester IS
    GENERIC (
        -- Clock frequency in Hz (used only to size timeout counter)
        G_CLK_FREQ_HZ        : positive := 50_000_000;
 
        -- Debounce length in clock cycles for i_rst_n and i_start
        -- 50 MHz * 10 ms = 500 000 cycles
        G_DEBOUNCE_CYCLES    : positive := 500_000;
 
        -- Maximum cycles to wait for ip_ready before declaring timeout/fail
        -- 256 cycles at 50 MHz is ~5 µs; the pipeline needs at most ~20 cycles
        G_READY_TIMEOUT_CYCLES : positive := 256
    );
    PORT (
        -- ----------------------------------------------------------------
        -- Board-level inputs (only three external pins needed)
        -- ----------------------------------------------------------------
        i_clk      : IN  std_logic;  -- Board oscillator
        i_rst_n    : IN  std_logic;  -- Active-low async reset (push-button)
        i_start    : IN  std_logic;  -- Active-high start (push-button)
		  --------------------------------------------------------------------
		  probe_PE_write     : OUT  std_logic;
        probe_PE_address   : OUT  std_logic_vector(31 DOWNTO 0);
        probe_PE_writedata : OUT  std_logic_vector(31 DOWNTO 0);
        probe_PE_readdata  : OUT  std_logic_vector(31 DOWNTO 0);
        -- ----------------------------------------------------------------
        -- Result / status outputs (connect to LEDs or logic analyser)
        -- ----------------------------------------------------------------
        o_test_pass : OUT std_logic_vector(4 DOWNTO 0);  -- Pass per test
        o_test_fail : OUT std_logic_vector(4 DOWNTO 0);  -- Fail per test
        o_all_pass  : OUT std_logic;                     -- All 5 passed
        o_busy      : OUT std_logic;                     -- Sequence running
        o_done      : OUT std_logic                      -- End-of-sequence pulse
    );
END ENTITY PE_OnChip_Tester;
-- =============================================================================
 
ARCHITECTURE rtl OF PE_OnChip_Tester IS
 
    -- =========================================================================
    -- Component declaration : IP_PE (Avalon-MM slave wrapper for PE_TOP)
    -- =========================================================================
    COMPONENT IP_PE IS
        PORT (
            clock        : IN  std_logic;
            reset        : IN  std_logic;
            PE_write     : IN  std_logic;
            PE_address   : IN  std_logic_vector(31 DOWNTO 0);
            PE_writedata : IN  std_logic_vector(31 DOWNTO 0);
            PE_readdata  : OUT std_logic_vector(31 DOWNTO 0)
        );
    END COMPONENT IP_PE;
 
    -- =========================================================================
    -- Constants
    -- =========================================================================
 
    -- Number of test cases
    CONSTANT C_NUM_TESTS : positive := 5;
 
    -- Avalon-MM byte offsets (as 4-bit address field used inside IP_PE)
    CONSTANT C_ADDR_A     : std_logic_vector(31 DOWNTO 0) := x"00000000"; -- Write A
    CONSTANT C_ADDR_B     : std_logic_vector(31 DOWNTO 0) := x"00000002"; -- Write B
    CONSTANT C_ADDR_CTRL  : std_logic_vector(31 DOWNTO 0) := x"00000004"; -- Flush ctrl
    CONSTANT C_ADDR_START : std_logic_vector(31 DOWNTO 0) := x"00000006"; -- Start
    CONSTANT C_ADDR_READY : std_logic_vector(31 DOWNTO 0) := x"00000009"; -- Poll ready
    CONSTANT C_ADDR_ACC   : std_logic_vector(31 DOWNTO 0) := x"0000000A"; -- Read acc
    CONSTANT C_ADDR_AOUT  : std_logic_vector(31 DOWNTO 0) := x"0000000B"; -- Read a_out
    CONSTANT C_ADDR_BOUT  : std_logic_vector(31 DOWNTO 0) := x"0000000C"; -- Read b_out
 
    -- =========================================================================
    -- Test vector ROM (matches tb_IP_PE.vhd exactly)
    --   a_vec  : 8-bit signed operand A (zero/sign fill handled by IP_PE)
    --   b_vec  : 8-bit signed operand B
    --   exp_vec: expected 32-bit signed accumulator result
    -- =========================================================================
    TYPE t_byte_rom  IS ARRAY (0 TO C_NUM_TESTS - 1) OF std_logic_vector(7  DOWNTO 0);
    TYPE t_word_rom  IS ARRAY (0 TO C_NUM_TESTS - 1) OF std_logic_vector(31 DOWNTO 0);
 
    -- Operand A: 5, -2(0xFE), 10, 127(0x7F), -128(0x80)
    CONSTANT C_A_ROM : t_byte_rom := (
        x"05", x"FE", x"0A", x"7F", x"80"
    );
 
    -- Operand B: 3, 4, -3(0xFD), 1, -1(0xFF)
    CONSTANT C_B_ROM : t_byte_rom := (
        x"03", x"04", x"FD", x"01", x"FF"
    );
 
    -- Expected acc_out: 15, -8, -30, 127, 128
    CONSTANT C_EXP_ROM : t_word_rom := (
        std_logic_vector(to_signed(  15, 32)),
        std_logic_vector(to_signed(  -8, 32)),
        std_logic_vector(to_signed( -30, 32)),
        std_logic_vector(to_signed( 127, 32)),
        std_logic_vector(to_signed( 128, 32))
    );
 
    -- =========================================================================
    -- FSM state type
    -- =========================================================================
    TYPE t_tester_state IS (
        ST_IDLE,          -- Waiting for i_start pulse
        ST_FLUSH,         -- Assert flush for one cycle before first test
        ST_FLUSH_CLEAR,   -- De-assert flush (control register back to 0)
        ST_WRITE_A,       -- Drive addr=0x0, data=A, write='1'
        ST_WRITE_A_GAP,   -- De-assert write (one idle cycle, matches TB)
        ST_WRITE_B,       -- Drive addr=0x2, data=B, write='1'
        ST_WRITE_B_GAP,   -- De-assert write
        ST_WRITE_START,   -- Drive addr=0x6, data=1, write='1'
        ST_WRITE_START_GAP, -- De-assert write
        ST_POLL_SETUP,    -- Drive addr=0x9, write='0' (setup read address)
        ST_POLL_READ,     -- Sample PE_readdata; check bit0 for ip_ready
        ST_READ_ACC,      -- Drive addr=0xA; sample acc result next cycle
        ST_LATCH_ACC,     -- Latch PE_readdata, compare with expected
        ST_WAIT_GAP,      -- Inter-test gap (≈5 cycles, mirrors TB 100 ns)
        ST_DONE           -- All tests complete; hold status outputs
    );
 
    -- =========================================================================
    -- Internal signal declarations
    -- =========================================================================
 
    -- FSM
    SIGNAL s_state      : t_tester_state := ST_IDLE;
 
    -- Test-vector index (0 .. C_NUM_TESTS-1)
    SIGNAL s_test_idx   : integer RANGE 0 TO C_NUM_TESTS - 1 := 0;
 
    -- Avalon-MM bus drive registers (all registered to avoid glitches)
    SIGNAL s_av_write   : std_logic                     := '0';
    SIGNAL s_av_addr    : std_logic_vector(31 DOWNTO 0) := (OTHERS => '0');
    SIGNAL s_av_wdata   : std_logic_vector(31 DOWNTO 0) := (OTHERS => '0');
    SIGNAL s_av_rdata   : std_logic_vector(31 DOWNTO 0);  -- from IP_PE
 
    -- Result registers
    SIGNAL s_test_pass  : std_logic_vector(C_NUM_TESTS - 1 DOWNTO 0) := (OTHERS => '0');
    SIGNAL s_test_fail  : std_logic_vector(C_NUM_TESTS - 1 DOWNTO 0) := (OTHERS => '0');
 
    -- Watchdog counter for ip_ready polling
    SIGNAL s_timeout_cnt : integer RANGE 0 TO G_READY_TIMEOUT_CYCLES := 0;
 
    -- Inter-test gap counter (5 cycles replicates the TB 100 ns @ 50 MHz)
    CONSTANT C_GAP_CYCLES : integer := 5;
    SIGNAL s_gap_cnt      : integer RANGE 0 TO C_GAP_CYCLES := 0;
 
    -- Active-low reset synchronised to i_clk (two-FF synchroniser)
    SIGNAL s_rst_meta   : std_logic := '1';
    SIGNAL s_rst_sync   : std_logic := '1';  -- active-high after sync
 
    -- Debounced and synchronised start button
    SIGNAL s_start_meta    : std_logic := '0';
    SIGNAL s_start_sync    : std_logic := '0';
    SIGNAL s_start_prev    : std_logic := '0';
    SIGNAL s_start_pulse   : std_logic := '0';  -- single-cycle rising-edge pulse
    SIGNAL s_dbnc_cnt      : integer RANGE 0 TO G_DEBOUNCE_CYCLES := 0;
    SIGNAL s_start_dbnc    : std_logic := '0';  -- debounced start level
 
    -- IP_PE reset (active-high, derived from synchronised board reset)
    SIGNAL s_ip_reset   : std_logic := '1';
 
    -- Done pulse (single cycle)
    SIGNAL s_done_pulse : std_logic := '0';
 
BEGIN
 
    -- =========================================================================
    -- Reset synchroniser (two flip-flop, async assert / sync de-assert)
    -- =========================================================================
    RESET_SYNC : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            s_rst_meta <= '1';
            s_rst_sync <= '1';
        ELSIF rising_edge(i_clk) THEN
            s_rst_meta <= '0';
            s_rst_sync <= s_rst_meta;
        END IF;
    END PROCESS RESET_SYNC;
 
    -- IP_PE uses active-high reset
    s_ip_reset <= s_rst_sync;
 
    -- =========================================================================
    -- Start-button debounce and edge detect
    --   The raw i_start pin is double-flopped then run through a saturating
    --   counter. s_start_pulse is a guaranteed single-cycle rising-edge event.
    -- =========================================================================
    START_DEBOUNCE : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            s_start_meta  <= '0';
            s_start_sync  <= '0';
            s_start_prev  <= '0';
            s_start_pulse <= '0';
            s_start_dbnc  <= '0';
            s_dbnc_cnt    <= 0;
        ELSIF rising_edge(i_clk) THEN
            -- Two-FF synchroniser
            s_start_meta <= not i_start;
            s_start_sync <= s_start_meta;
 
            -- Saturating debounce counter
            IF s_start_sync = '1' THEN
                IF s_dbnc_cnt = G_DEBOUNCE_CYCLES THEN
                    s_start_dbnc <= '1';
                ELSE
                    s_dbnc_cnt <= s_dbnc_cnt + 1;
                END IF;
            ELSE
                s_dbnc_cnt   <= 0;
                s_start_dbnc <= '0';
            END IF;
 
            -- Rising-edge detect on debounced level
            s_start_prev  <= s_start_dbnc;
            IF s_start_dbnc = '1' AND s_start_prev = '0' THEN
                s_start_pulse <= '1';
            ELSE
                s_start_pulse <= '0';
            END IF;
        END IF;
    END PROCESS START_DEBOUNCE;
 
    -- =========================================================================
    -- Main tester FSM  (single-process: state + registered Avalon outputs)
    -- All Avalon outputs to IP_PE are updated synchronously to guarantee
    -- the edge-detect logic inside IP_PE sees clean, glitch-free transitions.
    -- =========================================================================
    TESTER_FSM : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            -- ----------------------------------------------------------------
            -- Asynchronous reset: safe default state
            -- ----------------------------------------------------------------
            s_state      <= ST_IDLE;
            s_test_idx   <= 0;
            s_av_write   <= '0';
            s_av_addr    <= (OTHERS => '0');
            s_av_wdata   <= (OTHERS => '0');
            s_test_pass  <= (OTHERS => '0');
            s_test_fail  <= (OTHERS => '0');
            s_timeout_cnt <= 0;
            s_gap_cnt    <= 0;
            s_done_pulse <= '0';
 
        ELSIF rising_edge(i_clk) THEN
            -- ----------------------------------------------------------------
            -- Default: de-assert single-cycle signals
            -- ----------------------------------------------------------------
            s_av_write   <= '0';
            s_done_pulse <= '0';
 
            CASE s_state IS
 
                -- ------------------------------------------------------------
                WHEN ST_IDLE =>
                -- ------------------------------------------------------------
                -- Remain here until the debounced start pulse arrives.
                -- On entry: reset result registers so a re-run starts clean.
                    s_test_pass  <= (OTHERS => '0');
                    s_test_fail  <= (OTHERS => '0');
                    s_test_idx   <= 0;
 
                    IF s_start_pulse = '1' THEN
                        -- Issue a flush before the first test to clear the
                        -- accumulator inside PE_TOP (mirrors good TB practice).
                        s_av_write <= '1';
                        s_av_addr  <= C_ADDR_CTRL;
                        s_av_wdata <= x"00000001";  -- bit0 = flush
                        s_state    <= ST_FLUSH;
                    END IF;
 
                -- ------------------------------------------------------------
                WHEN ST_FLUSH =>
                -- ------------------------------------------------------------
                    -- De-assert flush (write 0 to ctrl register)
                    s_av_write <= '1';
                    s_av_addr  <= C_ADDR_CTRL;
                    s_av_wdata <= x"00000000";
                    s_state    <= ST_FLUSH_CLEAR;
 
                -- ------------------------------------------------------------
                WHEN ST_FLUSH_CLEAR =>
                -- ------------------------------------------------------------
                    -- One idle cycle; proceed to first test operand A
                    s_av_write <= '1';
                    s_av_addr  <= C_ADDR_A;
                    s_av_wdata <= x"000000" &
                                  C_A_ROM(s_test_idx);  -- zero-extended 8-bit A
                    s_state    <= ST_WRITE_A;
 
                -- ------------------------------------------------------------
                WHEN ST_WRITE_A =>
                -- ------------------------------------------------------------
                    -- Write strobe held for one cycle (IP_PE edge-detects it)
                    s_av_write <= '0';
                    s_state    <= ST_WRITE_A_GAP;
 
                -- ------------------------------------------------------------
                WHEN ST_WRITE_A_GAP =>
                -- ------------------------------------------------------------
                    -- One idle cycle between writes (mirrors TB wait for CLK_PERIOD)
                    s_av_write <= '1';
                    s_av_addr  <= C_ADDR_B;
                    s_av_wdata <= x"000000" &
                                  C_B_ROM(s_test_idx);  -- zero-extended 8-bit B
                    s_state    <= ST_WRITE_B;
 
                -- ------------------------------------------------------------
                WHEN ST_WRITE_B =>
                -- ------------------------------------------------------------
                    s_av_write <= '0';
                    s_state    <= ST_WRITE_B_GAP;
 
                -- ------------------------------------------------------------
                WHEN ST_WRITE_B_GAP =>
                -- ------------------------------------------------------------
                    s_av_write <= '1';
                    s_av_addr  <= C_ADDR_START;
                    s_av_wdata <= x"00000001";
                    s_state    <= ST_WRITE_START;
 
                -- ------------------------------------------------------------
                WHEN ST_WRITE_START =>
                -- ------------------------------------------------------------
                    s_av_write    <= '0';
                    s_timeout_cnt <= 0;       -- arm the watchdog
                    s_state       <= ST_WRITE_START_GAP;
 
                -- ------------------------------------------------------------
                WHEN ST_WRITE_START_GAP =>
                -- ------------------------------------------------------------
                    -- Set up read address for ip_ready polling
                    s_av_addr  <= C_ADDR_READY;
                    s_av_write <= '0';
                    s_state    <= ST_POLL_SETUP;
 
                -- ------------------------------------------------------------
                WHEN ST_POLL_SETUP =>
                -- ------------------------------------------------------------
                    -- Address is now stable; sample readdata next cycle
                    s_state <= ST_POLL_READ;
 
                -- ------------------------------------------------------------
                WHEN ST_POLL_READ =>
                -- ------------------------------------------------------------
                    -- s_av_rdata is valid here (registered output from IP_PE)
                    IF s_av_rdata(0) = '1' THEN
                        -- ip_ready asserted: proceed to read the result
                        s_av_addr  <= C_ADDR_ACC;
                        s_av_write <= '0';
                        s_state    <= ST_READ_ACC;
                    ELSIF s_timeout_cnt = G_READY_TIMEOUT_CYCLES THEN
                        -- Watchdog expired: mark as fail and advance
                        s_test_fail(s_test_idx) <= '1';
                        s_gap_cnt <= 0;
                        s_state   <= ST_WAIT_GAP;
                    ELSE
                        -- Keep polling
                        s_timeout_cnt <= s_timeout_cnt + 1;
                        s_av_addr     <= C_ADDR_READY;
                        s_state       <= ST_POLL_SETUP;  -- re-setup
                    END IF;
 
                -- ------------------------------------------------------------
                WHEN ST_READ_ACC =>
                -- ------------------------------------------------------------
                    -- Address C_ADDR_ACC is stable; latch readdata next cycle
                    s_state <= ST_LATCH_ACC;
 
                -- ------------------------------------------------------------
                WHEN ST_LATCH_ACC =>
                -- ------------------------------------------------------------
                    -- Compare captured acc_out against expected value
                    IF s_av_rdata = C_EXP_ROM(s_test_idx) THEN
                        s_test_pass(s_test_idx) <= '1';
                    ELSE
                        s_test_fail(s_test_idx) <= '1';
                    END IF;
 
                    s_gap_cnt <= 0;
                    s_state   <= ST_WAIT_GAP;
 
                -- ------------------------------------------------------------
                WHEN ST_WAIT_GAP =>
                -- ------------------------------------------------------------
                    -- Short inter-test gap (≈ 100 ns at 50 MHz = 5 cycles)
                    IF s_gap_cnt = C_GAP_CYCLES - 1 THEN
                        IF s_test_idx = C_NUM_TESTS - 1 THEN
                            -- All tests done
                            s_done_pulse <= '1';
                            s_state      <= ST_DONE;
                        ELSE
                            -- Advance to next test vector
                            s_test_idx <= s_test_idx + 1;
                            s_av_write <= '1';
                            s_av_addr  <= C_ADDR_A;
                            s_av_wdata <= x"000000" &
                                          C_A_ROM(s_test_idx + 1);
                            s_state    <= ST_WRITE_A;
                        END IF;
                    ELSE
                        s_gap_cnt <= s_gap_cnt + 1;
                    END IF;
 
                -- ------------------------------------------------------------
                WHEN ST_DONE =>
                -- ------------------------------------------------------------
                    -- Hold result outputs; return to IDLE on a new start pulse
                    IF s_start_pulse = '1' THEN
                        s_state <= ST_IDLE;
                    END IF;
 
                -- ------------------------------------------------------------
                WHEN OTHERS =>
                    s_state <= ST_IDLE;
 
            END CASE;
        END IF;
    END PROCESS TESTER_FSM;
 
    -- =========================================================================
    -- IP_PE instantiation
    -- All bus signals are driven exclusively by the registered FSM outputs.
    -- =========================================================================
    u_ip_pe : IP_PE
        PORT MAP (
            clock        => i_clk,
            reset        => s_ip_reset,
            PE_write     => s_av_write,
            PE_address   => s_av_addr,
            PE_writedata => s_av_wdata,
            PE_readdata  => s_av_rdata
        );
 
    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    o_test_pass <= s_test_pass;
    o_test_fail <= s_test_fail;
    o_all_pass  <= '1' WHEN s_test_pass = "11111" ELSE '0';
    o_busy      <= '0' WHEN (s_state = ST_IDLE OR s_state = ST_DONE) ELSE '1';
    o_done      <= s_done_pulse;
	 probe_PE_write     <= s_av_write;
    probe_PE_address   <= s_av_addr;
    probe_PE_writedata <= s_av_wdata;
    probe_PE_readdata  <= s_av_rdata;
	  
 
END ARCHITECTURE rtl;
