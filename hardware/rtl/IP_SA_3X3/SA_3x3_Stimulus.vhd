-- =============================================================================
-- Module      : SA_3x3_Stimulus
-- Project     : CSNE-SoC – Configurable Systolic Neural Engine
-- File        : SA_3x3_Stimulus.vhd
--
-- Description :
--   Hardware FSM that replicates the two test cases defined in tb_SA_TOP_3x3.vhd
--   and drives the SA_TOP_3x3 interface directly on FPGA silicon.
--   No simulator required — validation runs autonomously at full clock speed.
--
--   Each test case performs a full 3×3 matrix multiply C = A × B using 3
--   sequential MAC operations (one per k-slice of the shared dimension).
--
--   Test Case 1 — All positive integers:
--     A = [[ 1, 2, 3],   B = [[ 7, 1, 4],
--          [ 4, 5, 6],        [ 8, 2, 5],
--          [ 7, 8, 9]]        [ 9, 3, 6]]
--     Expected C:
--       C[0][0]= 50   C[0][1]= 14   C[0][2]= 32
--       C[1][0]=122   C[1][1]= 32   C[1][2]= 77
--       C[2][0]=194   C[2][1]= 50   C[2][2]=122
--
--   Test Case 2 — Signed (mixed +/-):
--     A = [[-1, 2,-3],   B = [[ 1,-2, 3],
--          [ 4,-5, 6],        [-4, 5,-6],
--          [-7, 8,-9]]        [ 7,-8, 9]]
--     Expected C:
--       C[0][0]= -30  C[0][1]=  36  C[0][2]= -42
--       C[1][0]=  66  C[1][1]= -81  C[1][2]=  96
--       C[2][0]=-102  C[2][1]= 126  C[2][2]=-150
--
--   Stimulus protocol per k-slice:
--     1. Assert o_flush for 1 cycle (first k-slice of each test only)
--     2. Drive o_a_row* and o_b_col* with stable operand values
--     3. Assert o_start for 1 cycle → SA_TOP_3x3 captures inputs
--     4. Wait for i_done to arrive → accumulator updated
--     5. Repeat for k = 0, 1, 2
--     6. Compare all 9 accumulators against expected values
--     7. Assert o_tc_pass or o_tc_fail for the test case
--
--   FSM state sequence:
--     IDLE → FLUSH → WAIT_FLUSH_DONE →
--     LOAD_K0 → START_K0 → WAIT_K0 →
--     LOAD_K1 → START_K1 → WAIT_K1 →
--     LOAD_K2 → START_K2 → WAIT_K2 →
--     CHECK → GAP → [next test or DONE]
--
--   External interface (connects to SA_TOP_3x3 and result LEDs):
--     i_clk, i_rst_n, i_start  — board pins
--     i_done                   — from SA_TOP_3x3.o_done
--     i_acc_rc                 — from SA_TOP_3x3.o_acc_rc (9 signals)
--     o_flush, o_start         — to SA_TOP_3x3
--     o_a_row*, o_b_col*       — to SA_TOP_3x3 operand ports
--     o_tc1_pass, o_tc1_fail   — test case 1 LED result
--     o_tc2_pass, o_tc2_fail   — test case 2 LED result
--     o_all_pass               — all 18 checks passed
--     o_busy                   — sequence running
--     o_done_seq               — single-cycle pulse when sequence complete
--
-- Generics:
--   G_CLK_FREQ_HZ    — board oscillator frequency (for debounce sizing)
--   G_DEBOUNCE_CYCLES — push-button debounce length in clock cycles
--   G_GAP_CYCLES     — idle cycles between test cases
--
-- Standards   : VHDL-2008, IEEE Std 1076-2008
-- Author      : Daniel G. Fajardo Lopez
-- Institution : Pontificia Universidad Javeriana, Bogotá D.C., Colombia
-- Date        : 2026-05-11
-- Version     : 1.0
-- =============================================================================

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY SA_3x3_Stimulus IS
    GENERIC (
        G_CLK_FREQ_HZ     : positive := 50_000_000;
        G_DEBOUNCE_CYCLES : positive := 500_000;
        G_GAP_CYCLES      : positive := 10
    );
    PORT (
        -- ----------------------------------------------------------------
        -- Board-level inputs
        -- ----------------------------------------------------------------
        i_clk       : IN  std_logic;
        i_rst_n     : IN  std_logic;
        i_start     : IN  std_logic;  -- push-button: begin test sequence

        -- ----------------------------------------------------------------
        -- Interface FROM SA_TOP_3x3
        -- ----------------------------------------------------------------
        i_done      : IN  std_logic;  -- computation complete pulse

        -- 9 accumulator results [row][col]
        i_acc_00    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_01    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_02    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_10    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_11    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_12    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_20    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_21    : IN  std_logic_vector(31 DOWNTO 0);
        i_acc_22    : IN  std_logic_vector(31 DOWNTO 0);

        -- ----------------------------------------------------------------
        -- Interface TO SA_TOP_3x3
        -- ----------------------------------------------------------------
        o_flush     : OUT std_logic;
        o_start     : OUT std_logic;

        o_a_row0    : OUT std_logic_vector(7 DOWNTO 0);
        o_a_row1    : OUT std_logic_vector(7 DOWNTO 0);
        o_a_row2    : OUT std_logic_vector(7 DOWNTO 0);
        o_b_col0    : OUT std_logic_vector(7 DOWNTO 0);
        o_b_col1    : OUT std_logic_vector(7 DOWNTO 0);
        o_b_col2    : OUT std_logic_vector(7 DOWNTO 0);

        -- ----------------------------------------------------------------
        -- Result outputs (connect to LEDs or logic analyser)
        -- ----------------------------------------------------------------
        o_tc1_pass  : OUT std_logic;  -- Test Case 1: all 9 correct
        o_tc1_fail  : OUT std_logic;  -- Test Case 1: at least 1 wrong
        o_tc2_pass  : OUT std_logic;  -- Test Case 2: all 9 correct
        o_tc2_fail  : OUT std_logic;  -- Test Case 2: at least 1 wrong
        o_all_pass  : OUT std_logic;  -- both test cases passed
        o_busy      : OUT std_logic;  -- sequence running
        o_done_seq  : OUT std_logic   -- single-cycle end pulse
    );
END ENTITY SA_3x3_Stimulus;

ARCHITECTURE rtl OF SA_3x3_Stimulus IS

    -- =========================================================================
    -- FSM state type
    -- =========================================================================
    TYPE t_stim_state IS (
        ST_IDLE,         -- Wait for debounced start pulse
        ST_FLUSH,        -- Assert flush for 1 cycle
        ST_LOAD_K0,      -- Drive k=0 operands; assert start
        ST_WAIT_K0,      -- Wait for i_done (k=0 complete)
        ST_LOAD_K1,      -- Drive k=1 operands; assert start
        ST_WAIT_K1,      -- Wait for i_done (k=1 complete)
        ST_LOAD_K2,      -- Drive k=2 operands; assert start
        ST_WAIT_K2,      -- Wait for i_done (k=2 complete)
        ST_CHECK,        -- Compare 9 accumulators vs expected
        ST_GAP,          -- Inter-test idle gap
        ST_DONE          -- Both tests complete; hold results
    );

    -- =========================================================================
    -- Test vector ROM types
    -- =========================================================================
    TYPE t_byte_vec  IS ARRAY (0 TO 2) OF std_logic_vector(7 DOWNTO 0);
    TYPE t_kslice_a  IS ARRAY (0 TO 2) OF t_byte_vec; -- [k][row]
    TYPE t_kslice_b  IS ARRAY (0 TO 2) OF t_byte_vec; -- [k][col]
    TYPE t_acc_rom   IS ARRAY (0 TO 8) OF std_logic_vector(31 DOWNTO 0);
    TYPE t_tc_a_rom  IS ARRAY (0 TO 1) OF t_kslice_a;
    TYPE t_tc_b_rom  IS ARRAY (0 TO 1) OF t_kslice_b;
    TYPE t_tc_e_rom  IS ARRAY (0 TO 1) OF t_acc_rom;

    -- =========================================================================
    -- Helper: convert signed integer to 8-bit std_logic_vector
    -- =========================================================================
    FUNCTION s8(v : integer) RETURN std_logic_vector IS
    BEGIN RETURN std_logic_vector(to_signed(v, 8)); END FUNCTION;

    -- =========================================================================
    -- Helper: convert signed integer to 32-bit std_logic_vector
    -- =========================================================================
    FUNCTION s32(v : integer) RETURN std_logic_vector IS
    BEGIN RETURN std_logic_vector(to_signed(v, 32)); END FUNCTION;

    -- =========================================================================
    -- Test vector ROMs
    -- C_A_ROM[tc][k][row] = A[row][k]  (column k of matrix A, test tc)
    -- C_B_ROM[tc][k][col] = B[k][col]  (row k of matrix B, test tc)
    -- =========================================================================

    -- TC0: A=[[1,2,3],[4,5,6],[7,8,9]]
    -- TC1: A=[[-1,2,-3],[4,-5,6],[-7,8,-9]]
    CONSTANT C_A_ROM : t_tc_a_rom := (
        -- TC0: k=0 col={1,4,7}  k=1 col={2,5,8}  k=2 col={3,6,9}
        ( (s8( 1),s8( 4),s8( 7)),
          (s8( 2),s8( 5),s8( 8)),
          (s8( 3),s8( 6),s8( 9)) ),
        -- TC1: k=0 col={-1,4,-7}  k=1 col={2,-5,8}  k=2 col={-3,6,-9}
        ( (s8(-1),s8( 4),s8(-7)),
          (s8( 2),s8(-5),s8( 8)),
          (s8(-3),s8( 6),s8(-9)) )
    );

    -- TC0: B=[[7,1,4],[8,2,5],[9,3,6]]
    -- TC1: B=[[1,-2,3],[-4,5,-6],[7,-8,9]]
    CONSTANT C_B_ROM : t_tc_b_rom := (
        -- TC0: k=0 row={7,1,4}  k=1 row={8,2,5}  k=2 row={9,3,6}
        ( (s8( 7),s8( 1),s8( 4)),
          (s8( 8),s8( 2),s8( 5)),
          (s8( 9),s8( 3),s8( 6)) ),
        -- TC1: k=0 row={1,-2,3}  k=1 row={-4,5,-6}  k=2 row={7,-8,9}
        ( (s8( 1),s8(-2),s8( 3)),
          (s8(-4),s8( 5),s8(-6)),
          (s8( 7),s8(-8),s8( 9)) )
    );

    -- Expected results: C_EXP[tc][r*3+c] = C[r][c]
    CONSTANT C_EXP_ROM : t_tc_e_rom := (
        -- TC0: C=[[50,14,32],[122,32,77],[194,50,122]]
        ( s32( 50),s32( 14),s32( 32),
          s32(122),s32( 32),s32( 77),
          s32(194),s32( 50),s32(122) ),
        -- TC1: C=[[-30,36,-42],[66,-81,96],[-102,126,-150]]
        ( s32( -30),s32(  36),s32( -42),
          s32(  66),s32( -81),s32(  96),
          s32(-102),s32( 126),s32(-150) )
    );

    -- =========================================================================
    -- Internal signals
    -- =========================================================================
    SIGNAL s_state       : t_stim_state;
    SIGNAL s_tc_idx      : integer RANGE 0 TO 1;       -- current test case
    SIGNAL s_gap_cnt     : integer RANGE 0 TO G_GAP_CYCLES;

    -- Result registers
    SIGNAL s_tc1_pass    : std_logic;
    SIGNAL s_tc1_fail    : std_logic;
    SIGNAL s_tc2_pass    : std_logic;
    SIGNAL s_tc2_fail    : std_logic;

    -- Button debounce
    SIGNAL s_btn_meta    : std_logic;
    SIGNAL s_btn_sync    : std_logic;
    SIGNAL s_btn_prev    : std_logic;
    SIGNAL s_btn_dbnc    : std_logic;
    SIGNAL s_btn_pulse   : std_logic;
    SIGNAL s_dbnc_cnt    : integer RANGE 0 TO G_DEBOUNCE_CYCLES;

    -- Reset synchroniser
    SIGNAL s_rst_meta    : std_logic;
    SIGNAL s_rst_sync    : std_logic;

    -- Accumulator array for comparison
    TYPE t_acc_array IS ARRAY (0 TO 8) OF std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc         : t_acc_array;

BEGIN

    -- =========================================================================
    -- Reset synchroniser (async assert, sync de-assert)
    -- =========================================================================
    RESET_SYNC : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            s_rst_meta <= '1'; s_rst_sync <= '1';
        ELSIF rising_edge(i_clk) THEN
            s_rst_meta <= '0'; s_rst_sync <= s_rst_meta;
        END IF;
    END PROCESS RESET_SYNC;

    -- =========================================================================
    -- Button debounce (saturating counter)
    -- =========================================================================
    DEBOUNCE : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            s_btn_meta  <= '0'; s_btn_sync  <= '0';
            s_btn_prev  <= '0'; s_btn_dbnc  <= '0';
            s_btn_pulse <= '0'; s_dbnc_cnt  <= 0;
        ELSIF rising_edge(i_clk) THEN
            s_btn_meta <= i_start;
            s_btn_sync <= s_btn_meta;

            IF s_btn_sync = '1' THEN
                IF s_dbnc_cnt = G_DEBOUNCE_CYCLES THEN
                    s_btn_dbnc <= '1';
                ELSE
                    s_dbnc_cnt <= s_dbnc_cnt + 1;
                END IF;
            ELSE
                s_dbnc_cnt <= 0; s_btn_dbnc <= '0';
            END IF;

            s_btn_prev  <= s_btn_dbnc;
            s_btn_pulse <= s_btn_dbnc AND NOT s_btn_prev;
        END IF;
    END PROCESS DEBOUNCE;

    -- =========================================================================
    -- Collect accumulators into indexable array for comparison
    -- =========================================================================
    s_acc(0) <= i_acc_00; s_acc(1) <= i_acc_01; s_acc(2) <= i_acc_02;
    s_acc(3) <= i_acc_10; s_acc(4) <= i_acc_11; s_acc(5) <= i_acc_12;
    s_acc(6) <= i_acc_20; s_acc(7) <= i_acc_21; s_acc(8) <= i_acc_22;

    -- =========================================================================
    -- Main stimulus FSM (single-process: state + registered outputs)
    -- All outputs to SA_TOP_3x3 are registered for glitch-free operation.
    -- =========================================================================
    MAIN_FSM : PROCESS (i_clk, i_rst_n)
        VARIABLE v_all_match : boolean;
    BEGIN
        IF i_rst_n = '0' THEN
            s_state    <= ST_IDLE;
            s_tc_idx   <= 0;
            s_gap_cnt  <= 0;
            s_tc1_pass <= '0'; s_tc1_fail <= '0';
            s_tc2_pass <= '0'; s_tc2_fail <= '0';
            o_flush    <= '0'; o_start    <= '0';
            o_a_row0   <= (OTHERS => '0'); o_a_row1 <= (OTHERS => '0');
            o_a_row2   <= (OTHERS => '0'); o_b_col0 <= (OTHERS => '0');
            o_b_col1   <= (OTHERS => '0'); o_b_col2 <= (OTHERS => '0');
            o_done_seq <= '0';

        ELSIF rising_edge(i_clk) THEN
            -- Default: de-assert single-cycle signals
            o_flush    <= '0';
            o_start    <= '0';
            o_done_seq <= '0';

            CASE s_state IS

                -- ------------------------------------------------------------
                WHEN ST_IDLE =>
                -- ------------------------------------------------------------
                    s_tc_idx   <= 0;
                    s_tc1_pass <= '0'; s_tc1_fail <= '0';
                    s_tc2_pass <= '0'; s_tc2_fail <= '0';

                    IF s_btn_pulse = '1' THEN
                        s_state <= ST_FLUSH;
                    END IF;

                -- ------------------------------------------------------------
                WHEN ST_FLUSH =>
                -- ------------------------------------------------------------
                    o_flush <= '1';
                    s_state <= ST_LOAD_K0;

                -- ------------------------------------------------------------
                WHEN ST_LOAD_K0 =>
                -- ------------------------------------------------------------
                    o_a_row0 <= C_A_ROM(s_tc_idx)(0)(0);
                    o_a_row1 <= C_A_ROM(s_tc_idx)(0)(1);
                    o_a_row2 <= C_A_ROM(s_tc_idx)(0)(2);
                    o_b_col0 <= C_B_ROM(s_tc_idx)(0)(0);
                    o_b_col1 <= C_B_ROM(s_tc_idx)(0)(1);
                    o_b_col2 <= C_B_ROM(s_tc_idx)(0)(2);
                    o_start  <= '1';
                    s_state  <= ST_WAIT_K0;

                -- ------------------------------------------------------------
                WHEN ST_WAIT_K0 =>
                -- ------------------------------------------------------------
                    IF i_done = '1' THEN
                        s_state <= ST_LOAD_K1;
                    END IF;

                -- ------------------------------------------------------------
                WHEN ST_LOAD_K1 =>
                -- ------------------------------------------------------------
                    o_a_row0 <= C_A_ROM(s_tc_idx)(1)(0);
                    o_a_row1 <= C_A_ROM(s_tc_idx)(1)(1);
                    o_a_row2 <= C_A_ROM(s_tc_idx)(1)(2);
                    o_b_col0 <= C_B_ROM(s_tc_idx)(1)(0);
                    o_b_col1 <= C_B_ROM(s_tc_idx)(1)(1);
                    o_b_col2 <= C_B_ROM(s_tc_idx)(1)(2);
                    o_start  <= '1';
                    s_state  <= ST_WAIT_K1;

                -- ------------------------------------------------------------
                WHEN ST_WAIT_K1 =>
                -- ------------------------------------------------------------
                    IF i_done = '1' THEN
                        s_state <= ST_LOAD_K2;
                    END IF;

                -- ------------------------------------------------------------
                WHEN ST_LOAD_K2 =>
                -- ------------------------------------------------------------
                    o_a_row0 <= C_A_ROM(s_tc_idx)(2)(0);
                    o_a_row1 <= C_A_ROM(s_tc_idx)(2)(1);
                    o_a_row2 <= C_A_ROM(s_tc_idx)(2)(2);
                    o_b_col0 <= C_B_ROM(s_tc_idx)(2)(0);
                    o_b_col1 <= C_B_ROM(s_tc_idx)(2)(1);
                    o_b_col2 <= C_B_ROM(s_tc_idx)(2)(2);
                    o_start  <= '1';
                    s_state  <= ST_WAIT_K2;

                -- ------------------------------------------------------------
                WHEN ST_WAIT_K2 =>
                -- ------------------------------------------------------------
                    IF i_done = '1' THEN
                        s_state <= ST_CHECK;
                    END IF;

                -- ------------------------------------------------------------
                WHEN ST_CHECK =>
                -- ------------------------------------------------------------
                    -- Compare all 9 accumulators against expected values
                    v_all_match := TRUE;
                    FOR i IN 0 TO 8 LOOP
                        IF s_acc(i) /= C_EXP_ROM(s_tc_idx)(i) THEN
                            v_all_match := FALSE;
                        END IF;
                    END LOOP;

                    IF s_tc_idx = 0 THEN
                        IF v_all_match THEN s_tc1_pass <= '1';
                        ELSE                s_tc1_fail <= '1';
                        END IF;
                    ELSE
                        IF v_all_match THEN s_tc2_pass <= '1';
                        ELSE                s_tc2_fail <= '1';
                        END IF;
                    END IF;

                    s_gap_cnt <= 0;
                    s_state   <= ST_GAP;

                -- ------------------------------------------------------------
                WHEN ST_GAP =>
                -- ------------------------------------------------------------
                    IF s_gap_cnt = G_GAP_CYCLES - 1 THEN
                        IF s_tc_idx = 1 THEN
                            -- Both tests done
                            o_done_seq <= '1';
                            s_state    <= ST_DONE;
                        ELSE
                            -- Advance to TC1
                            s_tc_idx  <= 1;
                            s_state   <= ST_FLUSH;
                        END IF;
                    ELSE
                        s_gap_cnt <= s_gap_cnt + 1;
                    END IF;

                -- ------------------------------------------------------------
                WHEN ST_DONE =>
                -- ------------------------------------------------------------
                    -- Hold results; restart on new button press
                    IF s_btn_pulse = '1' THEN
                        s_state <= ST_IDLE;
                    END IF;

                WHEN OTHERS =>
                    s_state <= ST_IDLE;

            END CASE;
        END IF;
    END PROCESS MAIN_FSM;

    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    o_tc1_pass <= s_tc1_pass;
    o_tc1_fail <= s_tc1_fail;
    o_tc2_pass <= s_tc2_pass;
    o_tc2_fail <= s_tc2_fail;
    o_all_pass <= s_tc1_pass AND s_tc2_pass;
    o_busy     <= '0' WHEN (s_state = ST_IDLE OR s_state = ST_DONE) ELSE '1';

END ARCHITECTURE rtl;
