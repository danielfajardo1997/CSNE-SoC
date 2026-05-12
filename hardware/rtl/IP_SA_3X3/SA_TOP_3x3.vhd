-- =============================================================================
-- Module      : SA_TOP_3x3
-- Project     : CSNE-SoC – Configurable Systolic Neural Engine
-- File        : SA_TOP_3x3.vhd
--
-- Description :
--   Top-level integration module for the 3×3 Output-Stationary systolic array.
--   Instantiates and connects:
--     1. FSM_Global        – single global controller (orchestrator)
--     2. SystolicArray_3x3 – 9 PEs with structural data skewing
--     3. LatencyTimer x3   – shared pipeline timers (MULT / SIGN-EXT / SUM)
--
--   This is the module that IP_PE (Avalon-MM wrapper) or the HPS driverSA_3x3_OnChip_SA_3x3_OnChip_Tester.vhdTester.vhd
--   communicates with. It exposes a clean, minimal port interface:
--
--     Inputs : clock, reset, start, flush
--              3 × 8-bit activation rows  (i_a_row*)
--              3 × 8-bit weight columns   (i_b_col*)
--     Outputs: 9 × 32-bit accumulator results (o_acc_RC)
--              1-bit done flag
--
--   Pipeline latency (from i_start to o_done):
--     MULT:     5 cycles  (G_MULT_CYCLES)
--     SIGN-EXT: 2 cycles  (G_SIGN_CYCLES)
--     SUM:      3 cycles  (G_SUM_CYCLES)
--     FSM OH:  ~4 cycles  (load/unload transitions)
--     Total:  ~14 cycles from start to done pulse
--
--   Data skewing adds up to 2 additional cycles at the array boundary,
--   so the valid output window begins 2 cycles after the first o_done.
--   For a sustained matrix-vector product, feed new data every (pipeline
--   depth + skew) cycles.
--
-- Generics:
--   G_MULT_CYCLES – multiplier pipeline depth    (default 5)
--   G_SIGN_CYCLES – sign-extender pipeline depth (default 2)
--   G_SUM_CYCLES  – accumulator pipeline depth   (default 3)
--
-- Standards   : VHDL-2008, IEEE Std 1076-2008
--
-- Author      : Daniel G. Fajardo Lopez
-- Institution : Pontificia Universidad Javeriana, Bogotá D.C., Colombia
-- Date        : 2026-05-07
-- Version     : 1.0
-- =============================================================================

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

-- -----------------------------------------------------------------------------
ENTITY SA_TOP_3x3 IS
    GENERIC (
        G_MULT_CYCLES : positive := 5;  -- Multiplier latency in clock cycles
        G_SIGN_CYCLES : positive := 2;  -- Sign-extender latency
        G_SUM_CYCLES  : positive := 3   -- Accumulator latency
    );
    PORT (
        -- ----------------------------------------------------------------
        -- Global signals
        -- ----------------------------------------------------------------
        i_clk       : IN  std_logic;   -- System clock (50 MHz)
        i_rst_n     : IN  std_logic;   -- Asynchronous reset, active-low
        i_start     : IN  std_logic;   -- Start one MAC operation (1 cycle)
        i_flush     : IN  std_logic;   -- Flush all PE accumulators
        o_done      : OUT std_logic;   -- Computation complete (1 cycle pulse)

        -- ----------------------------------------------------------------
        -- Activation inputs (INT8 signed) — one per row
        -- ----------------------------------------------------------------
        i_a_row0    : IN  std_logic_vector(7 DOWNTO 0);
        i_a_row1    : IN  std_logic_vector(7 DOWNTO 0);
        i_a_row2    : IN  std_logic_vector(7 DOWNTO 0);

        -- ----------------------------------------------------------------
        -- Weight inputs (INT8 signed) — one per column
        -- ----------------------------------------------------------------
        i_b_col0    : IN  std_logic_vector(7 DOWNTO 0);
        i_b_col1    : IN  std_logic_vector(7 DOWNTO 0);
        i_b_col2    : IN  std_logic_vector(7 DOWNTO 0);

        -- ----------------------------------------------------------------
        -- Accumulator outputs (INT32 signed) — one per PE [row][col]
        -- ----------------------------------------------------------------
        o_acc_00    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_01    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_02    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_10    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_11    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_12    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_20    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_21    : OUT std_logic_vector(31 DOWNTO 0);
        o_acc_22    : OUT std_logic_vector(31 DOWNTO 0)
    );
END ENTITY SA_TOP_3x3;
-- -----------------------------------------------------------------------------

ARCHITECTURE rtl OF SA_TOP_3x3 IS

    -- =========================================================================
    -- Component : FSM_Global
    -- =========================================================================
    COMPONENT FSM_Global IS
        PORT (
            i_clk             : IN  std_logic;
            i_rst_n           : IN  std_logic;
            i_start           : IN  std_logic;
            o_done            : OUT std_logic;
            i_timer_done_mult : IN  std_logic;
            i_timer_done_sign : IN  std_logic;
            i_timer_done_sum  : IN  std_logic;
            o_ack_timer_mult  : OUT std_logic;
            o_ack_timer_sign  : OUT std_logic;
            o_ack_timer_sum   : OUT std_logic;
            o_clear_timers    : OUT std_logic;
            o_en_in_mult      : OUT std_logic;
            o_en_out_mult     : OUT std_logic;
            o_en_in_sign      : OUT std_logic;
            o_en_out_sign     : OUT std_logic;
            o_en_in_sum       : OUT std_logic;
            o_en_out_sum      : OUT std_logic
        );
    END COMPONENT FSM_Global;

    -- =========================================================================
    -- Component : SystolicArray_3x3
    -- =========================================================================
    COMPONENT SystolicArray_3x3 IS
        PORT (
            i_clk         : IN  std_logic;
            i_rst_n       : IN  std_logic;
            i_start       : IN  std_logic;  -- capture pulse: latch inputs
            i_flush       : IN  std_logic;
            i_en_in_mult  : IN  std_logic;
            i_en_out_mult : IN  std_logic;
            i_en_in_sign  : IN  std_logic;
            i_en_out_sign : IN  std_logic;
            i_en_in_sum   : IN  std_logic;
            i_en_out_sum  : IN  std_logic;
            i_a_row0      : IN  std_logic_vector(7 DOWNTO 0);
            i_a_row1      : IN  std_logic_vector(7 DOWNTO 0);
            i_a_row2      : IN  std_logic_vector(7 DOWNTO 0);
            i_b_col0      : IN  std_logic_vector(7 DOWNTO 0);
            i_b_col1      : IN  std_logic_vector(7 DOWNTO 0);
            i_b_col2      : IN  std_logic_vector(7 DOWNTO 0);
            o_acc_00      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_01      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_02      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_10      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_11      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_12      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_20      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_21      : OUT std_logic_vector(31 DOWNTO 0);
            o_acc_22      : OUT std_logic_vector(31 DOWNTO 0)
        );
    END COMPONENT SystolicArray_3x3;

    -- =========================================================================
    -- Component : LatencyTimer (generic, used for all three pipeline stages)
    -- =========================================================================
    COMPONENT LatencyTimer IS
        GENERIC (
            G_CYCLES : positive := 5
        );
        PORT (
            i_clk   : IN  std_logic;
            i_rst_n : IN  std_logic;
            i_clear : IN  std_logic;
            i_start : IN  std_logic;
            o_done  : OUT std_logic
        );
    END COMPONENT LatencyTimer;

    -- =========================================================================
    -- Internal interconnect signals
    -- =========================================================================

    -- FSM → timers
    SIGNAL s_ack_timer_mult  : std_logic;
    SIGNAL s_ack_timer_sign  : std_logic;
    SIGNAL s_ack_timer_sum   : std_logic;
    SIGNAL s_clear_timers    : std_logic;

    -- Timers → FSM
    SIGNAL s_timer_done_mult : std_logic;
    SIGNAL s_timer_done_sign : std_logic;
    SIGNAL s_timer_done_sum  : std_logic;

    -- FSM → array (enable bus, broadcast to all 9 PEs)
    SIGNAL s_en_in_mult      : std_logic;
    SIGNAL s_en_out_mult     : std_logic;
    SIGNAL s_en_in_sign      : std_logic;
    SIGNAL s_en_out_sign     : std_logic;
    SIGNAL s_en_in_sum       : std_logic;
    SIGNAL s_en_out_sum      : std_logic;

    -- Start delayed 1 cycle so the array capture register has time to latch
    -- before the FSM fires en_in_mult on ST_LOAD_MULT.
    --
    -- Timing:
    --   Cycle 0: i_start='1'   — HPS presents data on ports
    --   Cycle 1: s_start_d1='1'— array latches inputs AND FSM starts
    --            FSM: IDLE -> ST_LOAD_MULT (en_in_mult fires next cycle)
    --   Cycle 2: FSM: ST_LOAD_MULT -> en_in_mult='1'
    --            PE input regs latch captured data ✓
    --            All 9 PEs identical — no skew compensation needed
    SIGNAL s_start_d1        : std_logic;
    SIGNAL s_fsm_done        : std_logic;

BEGIN

    -- =========================================================================
    -- Start delay register (1 cycle)
    -- =========================================================================
    START_DELAY : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            s_start_d1 <= '0';
        ELSIF rising_edge(i_clk) THEN
            s_start_d1 <= i_start;
        END IF;
    END PROCESS START_DELAY;

    -- All 9 PEs finish at the same cycle (broadcast, no skew).
    -- o_done connects directly from FSM — no extra delay needed.
    o_done <= s_fsm_done;

    -- =========================================================================
    -- FSM_Global instantiation
    -- The single orchestrator for all 9 PEs.
    -- =========================================================================
    u_fsm_global : FSM_Global
        PORT MAP (
            i_clk             => i_clk,
            i_rst_n           => i_rst_n,
            i_start           => s_start_d1,
            o_done            => s_fsm_done,
            i_timer_done_mult => s_timer_done_mult,
            i_timer_done_sign => s_timer_done_sign,
            i_timer_done_sum  => s_timer_done_sum,
            o_ack_timer_mult  => s_ack_timer_mult,
            o_ack_timer_sign  => s_ack_timer_sign,
            o_ack_timer_sum   => s_ack_timer_sum,
            o_clear_timers    => s_clear_timers,
            o_en_in_mult      => s_en_in_mult,
            o_en_out_mult     => s_en_out_mult,
            o_en_in_sign      => s_en_in_sign,
            o_en_out_sign     => s_en_out_sign,
            o_en_in_sum       => s_en_in_sum,
            o_en_out_sum      => s_en_out_sum
        );

    -- =========================================================================
    -- Shared LatencyTimer instances
    -- One set serves all 9 PEs because they run in lockstep.
    -- G_CYCLES matches the actual IP pipeline depth.
    -- =========================================================================

    u_timer_mult : LatencyTimer
        GENERIC MAP (G_CYCLES => G_MULT_CYCLES)
        PORT MAP (
            i_clk   => i_clk,
            i_rst_n => i_rst_n,
            i_clear => s_clear_timers,
            i_start => s_ack_timer_mult,
            o_done  => s_timer_done_mult
        );

    u_timer_sign : LatencyTimer
        GENERIC MAP (G_CYCLES => G_SIGN_CYCLES)
        PORT MAP (
            i_clk   => i_clk,
            i_rst_n => i_rst_n,
            i_clear => s_clear_timers,
            i_start => s_ack_timer_sign,
            o_done  => s_timer_done_sign
        );

    u_timer_sum : LatencyTimer
        GENERIC MAP (G_CYCLES => G_SUM_CYCLES)
        PORT MAP (
            i_clk   => i_clk,
            i_rst_n => i_rst_n,
            i_clear => s_clear_timers,
            i_start => s_ack_timer_sum,
            o_done  => s_timer_done_sum
        );

    -- =========================================================================
    -- SystolicArray_3x3 instantiation
    -- Receives enable bus from FSM and data from external ports.
    -- =========================================================================
    u_systolic_array : SystolicArray_3x3
        PORT MAP (
            i_clk         => i_clk,
            i_rst_n       => i_rst_n,
            i_start       => s_start_d1,  -- 1-cycle delayed: captures on same cycle FSM starts
            i_flush       => i_flush,
            i_en_in_mult  => s_en_in_mult,
            i_en_out_mult => s_en_out_mult,
            i_en_in_sign  => s_en_in_sign,
            i_en_out_sign => s_en_out_sign,
            i_en_in_sum   => s_en_in_sum,
            i_en_out_sum  => s_en_out_sum,
            i_a_row0      => i_a_row0,
            i_a_row1      => i_a_row1,
            i_a_row2      => i_a_row2,
            i_b_col0      => i_b_col0,
            i_b_col1      => i_b_col1,
            i_b_col2      => i_b_col2,
            o_acc_00      => o_acc_00,
            o_acc_01      => o_acc_01,
            o_acc_02      => o_acc_02,
            o_acc_10      => o_acc_10,
            o_acc_11      => o_acc_11,
            o_acc_12      => o_acc_12,
            o_acc_20      => o_acc_20,
            o_acc_21      => o_acc_21,
            o_acc_22      => o_acc_22
        );

END ARCHITECTURE rtl;