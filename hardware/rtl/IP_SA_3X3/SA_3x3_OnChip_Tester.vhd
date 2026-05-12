-- =============================================================================
-- Module      : SA_3x3_OnChip_Tester
-- Project     : CSNE-SoC – Configurable Systolic Neural Engine
-- File        : SA_3x3_OnChip_Tester.vhd
--
-- Description :
--   Top-level on-chip tester for the 3×3 MAC array (SA_TOP_3x3).
--   Instantiates and wires:
--     1. SA_3x3_Stimulus  — hardware FSM that replicates tb_SA_TOP_3x3.vhd
--     2. SA_TOP_3x3       — the DUT (3×3 systolic array + FSM_Global + timers)
--
--   External pins required on the FPGA board (DE10-Nano):
--     i_clk      — board oscillator (50 MHz)
--     i_rst_n    — active-low reset push-button (KEY[0])
--     i_start    — active-high start push-button (KEY[1], debounced internally)
--
--   Result outputs (connect to LEDs):
--     o_tc1_pass — LED: Test Case 1 passed (9/9 correct)
--     o_tc1_fail — LED: Test Case 1 failed
--     o_tc2_pass — LED: Test Case 2 passed (9/9 correct)
--     o_tc2_fail — LED: Test Case 2 failed
--     o_all_pass — LED: both test cases passed
--     o_busy     — LED: sequence running
--
--   Test Cases (identical to tb_SA_TOP_3x3.vhd):
--
--     TC1 — All positive:
--       A=[[1,2,3],[4,5,6],[7,8,9]]  B=[[7,1,4],[8,2,5],[9,3,6]]
--       Expected: C=[[50,14,32],[122,32,77],[194,50,122]]
--
--     TC2 — Signed:
--       A=[[-1,2,-3],[4,-5,6],[-7,8,-9]]  B=[[1,-2,3],[-4,5,-6],[7,-8,9]]
--       Expected: C=[[-30,36,-42],[66,-81,96],[-102,126,-150]]
--
--   Operation:
--     1. Press i_rst_n to reset all logic
--     2. Press i_start to begin the sequence
--     3. o_busy goes HIGH while running (~3 × pipeline_depth cycles per test)
--     4. After completion, o_tc1_pass / o_tc2_pass / o_all_pass show results
--     5. Press i_start again to re-run
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

ENTITY SA_3x3_OnChip_Tester IS
    GENERIC (
        G_CLK_FREQ_HZ     : positive := 50_000_000;
        G_DEBOUNCE_CYCLES : positive := 500_000;   -- 10 ms @ 50 MHz
        G_GAP_CYCLES      : positive := 10;
        G_MULT_CYCLES     : positive := 5;
        G_SIGN_CYCLES     : positive := 2;
        G_SUM_CYCLES      : positive := 3
    );
    PORT (
        -- ----------------------------------------------------------------
        -- Board pins (only 3 external inputs needed)
        -- ----------------------------------------------------------------
        i_clk       : IN  std_logic;
        i_rst_n     : IN  std_logic;   -- KEY[0] active-low reset
        i_start     : IN  std_logic;   -- KEY[1] active-high start

        -- ----------------------------------------------------------------
        -- Result LEDs
        -- ----------------------------------------------------------------
        o_tc1_pass  : OUT std_logic;   -- LED[1]: TC1 all 9 accumulators correct
        o_tc1_fail  : OUT std_logic;   -- LED[2]: TC1 at least 1 wrong
        o_tc2_pass  : OUT std_logic;   -- LED[3]: TC2 all 9 correct
        o_tc2_fail  : OUT std_logic;   -- LED[4]: TC2 at least 1 wrong
        o_all_pass  : OUT std_logic;   -- LED[5]: both TCs passed
        o_busy      : OUT std_logic;   -- LED[0]: sequence running

        -- ----------------------------------------------------------------
        -- Accumulator outputs (for SignalTap / PIO observation)
        -- Valid after o_busy goes LOW following each test case.
        -- ----------------------------------------------------------------
        o_acc_00    : OUT std_logic_vector(31 DOWNTO 0);  -- C[0][0]
        o_acc_01    : OUT std_logic_vector(31 DOWNTO 0);  -- C[0][1]
        o_acc_02    : OUT std_logic_vector(31 DOWNTO 0)  -- C[0][2]
       -- o_acc_10    : OUT std_logic_vector(31 DOWNTO 0);  -- C[1][0]
       -- o_acc_11    : OUT std_logic_vector(31 DOWNTO 0);  -- C[1][1]
       -- o_acc_12    : OUT std_logic_vector(31 DOWNTO 0);  -- C[1][2]
       -- o_acc_20    : OUT std_logic_vector(31 DOWNTO 0);  -- C[2][0]
       -- o_acc_21    : OUT std_logic_vector(31 DOWNTO 0);  -- C[2][1]
       -- o_acc_22    : OUT std_logic_vector(31 DOWNTO 0)   -- C[2][2]
    );
END ENTITY SA_3x3_OnChip_Tester;

ARCHITECTURE rtl OF SA_3x3_OnChip_Tester IS

    -- =========================================================================
    -- Component : SA_3x3_Stimulus
    -- =========================================================================
    COMPONENT SA_3x3_Stimulus IS
        GENERIC (
            G_CLK_FREQ_HZ     : positive;
            G_DEBOUNCE_CYCLES : positive;
            G_GAP_CYCLES      : positive
        );
        PORT (
            i_clk      : IN  std_logic;
            i_rst_n    : IN  std_logic;
            i_start    : IN  std_logic;
            i_done     : IN  std_logic;
            i_acc_00   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_01   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_02   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_10   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_11   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_12   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_20   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_21   : IN  std_logic_vector(31 DOWNTO 0);
            i_acc_22   : IN  std_logic_vector(31 DOWNTO 0);
            o_flush    : OUT std_logic;
            o_start    : OUT std_logic;
            o_a_row0   : OUT std_logic_vector(7 DOWNTO 0);
            o_a_row1   : OUT std_logic_vector(7 DOWNTO 0);
            o_a_row2   : OUT std_logic_vector(7 DOWNTO 0);
            o_b_col0   : OUT std_logic_vector(7 DOWNTO 0);
            o_b_col1   : OUT std_logic_vector(7 DOWNTO 0);
            o_b_col2   : OUT std_logic_vector(7 DOWNTO 0);
            o_tc1_pass : OUT std_logic;
            o_tc1_fail : OUT std_logic;
            o_tc2_pass : OUT std_logic;
            o_tc2_fail : OUT std_logic;
            o_all_pass : OUT std_logic;
            o_busy     : OUT std_logic;
            o_done_seq : OUT std_logic
        );
    END COMPONENT SA_3x3_Stimulus;

    -- =========================================================================
    -- Component : SA_TOP_3x3
    -- =========================================================================
    COMPONENT SA_TOP_3x3 IS
        GENERIC (
            G_MULT_CYCLES : positive;
            G_SIGN_CYCLES : positive;
            G_SUM_CYCLES  : positive
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

    -- =========================================================================
    -- Internal interconnect
    -- =========================================================================

    -- Stimulus → DUT
    SIGNAL s_flush   : std_logic;
    SIGNAL s_start   : std_logic;
    SIGNAL s_a_row0  : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_a_row1  : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_a_row2  : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_b_col0  : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_b_col1  : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_b_col2  : std_logic_vector(7 DOWNTO 0);

    -- DUT → Stimulus
    SIGNAL s_done    : std_logic;
    SIGNAL s_acc_00  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_01  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_02  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_10  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_11  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_12  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_20  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_21  : std_logic_vector(31 DOWNTO 0);
    SIGNAL s_acc_22  : std_logic_vector(31 DOWNTO 0);

    -- Done sequence (unused at top level but available for debugging)
    SIGNAL s_done_seq : std_logic;

BEGIN

    -- =========================================================================
    -- Stimulus FSM instantiation
    -- =========================================================================
    u_stimulus : SA_3x3_Stimulus
        GENERIC MAP (
            G_CLK_FREQ_HZ     => G_CLK_FREQ_HZ,
            G_DEBOUNCE_CYCLES => G_DEBOUNCE_CYCLES,
            G_GAP_CYCLES      => G_GAP_CYCLES
        )
        PORT MAP (
            i_clk      => i_clk,
            i_rst_n    => i_rst_n,
            i_start    => i_start,
            i_done     => s_done,
            i_acc_00   => s_acc_00,  i_acc_01 => s_acc_01,  i_acc_02 => s_acc_02,
            i_acc_10   => s_acc_10,  i_acc_11 => s_acc_11,  i_acc_12 => s_acc_12,
            i_acc_20   => s_acc_20,  i_acc_21 => s_acc_21,  i_acc_22 => s_acc_22,
            o_flush    => s_flush,
            o_start    => s_start,
            o_a_row0   => s_a_row0,  o_a_row1 => s_a_row1,  o_a_row2 => s_a_row2,
            o_b_col0   => s_b_col0,  o_b_col1 => s_b_col1,  o_b_col2 => s_b_col2,
            o_tc1_pass => o_tc1_pass,
            o_tc1_fail => o_tc1_fail,
            o_tc2_pass => o_tc2_pass,
            o_tc2_fail => o_tc2_fail,
            o_all_pass => o_all_pass,
            o_busy     => o_busy,
            o_done_seq => s_done_seq
        );

    -- =========================================================================
    -- DUT instantiation (3×3 systolic array)
    -- =========================================================================
    u_sa_top : SA_TOP_3x3
        GENERIC MAP (
            G_MULT_CYCLES => G_MULT_CYCLES,
            G_SIGN_CYCLES => G_SIGN_CYCLES,
            G_SUM_CYCLES  => G_SUM_CYCLES
        )
        PORT MAP (
            i_clk    => i_clk,
            i_rst_n  => i_rst_n,
            i_start  => s_start,
            i_flush  => s_flush,
            o_done   => s_done,
            i_a_row0 => s_a_row0,  i_a_row1 => s_a_row1,  i_a_row2 => s_a_row2,
            i_b_col0 => s_b_col0,  i_b_col1 => s_b_col1,  i_b_col2 => s_b_col2,
            o_acc_00 => s_acc_00,  o_acc_01 => s_acc_01,  o_acc_02 => s_acc_02,
            o_acc_10 => s_acc_10,  o_acc_11 => s_acc_11,  o_acc_12 => s_acc_12,
            o_acc_20 => s_acc_20,  o_acc_21 => s_acc_21,  o_acc_22 => s_acc_22
        );

    -- =========================================================================
    -- Accumulator outputs — direct pass-through from DUT to top-level ports
    -- =========================================================================
    o_acc_00 <= s_acc_00;  o_acc_01 <= s_acc_01;  o_acc_02 <= s_acc_02;
  --  o_acc_10 <= s_acc_10;  o_acc_11 <= s_acc_11;  o_acc_12 <= s_acc_12;
  --  o_acc_20 <= s_acc_20;  o_acc_21 <= s_acc_21;  o_acc_22 <= s_acc_22;

END ARCHITECTURE rtl;