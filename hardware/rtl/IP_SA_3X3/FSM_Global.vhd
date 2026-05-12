-- =============================================================================
-- Module      : FSM_Global
-- Project     : CSNE-SoC – Configurable Systolic Neural Engine
-- File        : FSM_Global.vhd
--
-- Description :
--   Global Moore FSM controller for the 3×3 Output-Stationary systolic array.
--   This module acts as the single orchestrator for all 9 Processing Elements
--   (PEs), driving their enable signals in lockstep so that the systolic
--   pipeline operates correctly.
--
--   All 9 PEs share the same enable bus because in an Output-Stationary
--   systolic array every PE executes the same operation at the same cycle —
--   only the data differs (skewing is handled structurally in the array).
--
--   Pipeline sequence per MAC operation:
--     IDLE → LOAD_MULT → WAIT_MULT → UNLOAD_MULT
--          → LOAD_SIGN → WAIT_SIGN → UNLOAD_SIGN
--          → LOAD_SUM  → WAIT_SUM  → UNLOAD_SUM → DONE → IDLE
--
--   Timer handshake:
--     FSM asserts ack_timer_* to arm the corresponding timer.
--     FSM waits for timer_done_* before advancing the pipeline.
--     Timers are shared (one set) because all PEs are in lockstep.
--
-- Port description:
--   i_clk           – system clock (rising-edge triggered)
--   i_rst_n         – asynchronous active-low reset
--   i_start         – one-cycle pulse to begin one MAC operation
--   o_done          – one-cycle pulse when UNLOAD_SUM completes
--   timer_done_*    – completion flags from the three shared timers
--   ack_timer_*     – arm/clear signals to the three shared timers
--   o_clear_timers  – clears all timers simultaneously (asserted in IDLE)
--   o_en_in_mult    – enable data input  to MULT stage  (all 9 PEs)
--   o_en_out_mult   – enable data output from MULT stage (all 9 PEs)
--   o_en_in_sign    – enable data input  to SIGN-EXT stage
--   o_en_out_sign   – enable data output from SIGN-EXT stage
--   o_en_in_sum     – enable data input  to SUM stage
--   o_en_out_sum    – enable data output from SUM stage
--
-- Standards   : VHDL-2008, IEEE Std 1076-2008
--               Naming: i_/o_ ports, s_ internals, C_ constants, t_ types
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
ENTITY FSM_Global IS
    PORT (
        -- ----------------------------------------------------------------
        -- Global signals
        -- ----------------------------------------------------------------
        i_clk           : IN  std_logic;  -- System clock
        i_rst_n         : IN  std_logic;  -- Asynchronous reset, active-low
        i_start         : IN  std_logic;  -- Start pulse (one cycle)
        o_done          : OUT std_logic;  -- Done pulse  (one cycle)

        -- ----------------------------------------------------------------
        -- Shared timer interface (one set serves all 9 PEs in lockstep)
        -- ----------------------------------------------------------------
        i_timer_done_mult : IN  std_logic;  -- MULT timer expired
        i_timer_done_sign : IN  std_logic;  -- SIGN-EXT timer expired
        i_timer_done_sum  : IN  std_logic;  -- SUM timer expired
        o_ack_timer_mult  : OUT std_logic;  -- Arm / clear MULT timer
        o_ack_timer_sign  : OUT std_logic;  -- Arm / clear SIGN-EXT timer
        o_ack_timer_sum   : OUT std_logic;  -- Arm / clear SUM timer
        o_clear_timers    : OUT std_logic;  -- Clear all timers (IDLE state)

        -- ----------------------------------------------------------------
        -- Enable bus — broadcast to all 9 PEs simultaneously
        -- ----------------------------------------------------------------
        o_en_in_mult    : OUT std_logic;  -- Latch data into multiplier
        o_en_out_mult   : OUT std_logic;  -- Move mult result downstream
        o_en_in_sign    : OUT std_logic;  -- Latch data into sign-extender
        o_en_out_sign   : OUT std_logic;  -- Move sign-ext result downstream
        o_en_in_sum     : OUT std_logic;  -- Latch data into accumulator
        o_en_out_sum    : OUT std_logic   -- Output accumulator result
    );
END ENTITY FSM_Global;
-- -----------------------------------------------------------------------------

ARCHITECTURE rtl OF FSM_Global IS

    -- =========================================================================
    -- FSM state type
    -- =========================================================================
    TYPE t_global_state IS (
        ST_IDLE,            -- Waiting for i_start
        ST_LOAD_MULT,       -- en_in_mult = '1'; arm MULT timer
        ST_WAIT_MULT,       -- Hold until MULT timer expires
        ST_UNLOAD_MULT,     -- en_out_mult = '1'; pass result to SIGN-EXT
        ST_LOAD_SIGN,       -- en_in_sign = '1'; arm SIGN-EXT timer
        ST_WAIT_SIGN,       -- Hold until SIGN-EXT timer expires
        ST_UNLOAD_SIGN,     -- en_out_sign = '1'; pass result to SUM
        ST_LOAD_SUM,        -- en_in_sum = '1'; arm SUM timer
        ST_WAIT_SUM,        -- Hold until SUM timer expires
        ST_UNLOAD_SUM,      -- en_out_sum = '1'; accumulator result valid
        ST_DONE             -- One-cycle done pulse; return to IDLE
    );

    SIGNAL s_state      : t_global_state;
    SIGNAL s_next_state : t_global_state;

BEGIN

    -- =========================================================================
    -- STATE REGISTER
    -- Asynchronous active-low reset forces FSM to IDLE.
    -- =========================================================================
    STATE_REGISTER : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            s_state <= ST_IDLE;
        ELSIF rising_edge(i_clk) THEN
            s_state <= s_next_state;
        END IF;
    END PROCESS STATE_REGISTER;

    -- =========================================================================
    -- NEXT-STATE LOGIC
    -- =========================================================================
    NEXT_STATE_LOGIC : PROCESS (
        s_state,
        i_start,
        i_timer_done_mult,
        i_timer_done_sign,
        i_timer_done_sum
    )
    BEGIN
        -- Default: hold current state
        s_next_state <= s_state;

        CASE s_state IS

            WHEN ST_IDLE =>
                IF i_start = '1' THEN
                    s_next_state <= ST_LOAD_MULT;
                END IF;

            WHEN ST_LOAD_MULT =>
                s_next_state <= ST_WAIT_MULT;

            WHEN ST_WAIT_MULT =>
                IF i_timer_done_mult = '1' THEN
                    s_next_state <= ST_UNLOAD_MULT;
                END IF;

            WHEN ST_UNLOAD_MULT =>
                s_next_state <= ST_LOAD_SIGN;

            WHEN ST_LOAD_SIGN =>
                s_next_state <= ST_WAIT_SIGN;

            WHEN ST_WAIT_SIGN =>
                IF i_timer_done_sign = '1' THEN
                    s_next_state <= ST_UNLOAD_SIGN;
                END IF;

            WHEN ST_UNLOAD_SIGN =>
                s_next_state <= ST_LOAD_SUM;

            WHEN ST_LOAD_SUM =>
                s_next_state <= ST_WAIT_SUM;

            WHEN ST_WAIT_SUM =>
                IF i_timer_done_sum = '1' THEN
                    s_next_state <= ST_UNLOAD_SUM;
                END IF;

            WHEN ST_UNLOAD_SUM =>
                s_next_state <= ST_DONE;

            WHEN ST_DONE =>
                s_next_state <= ST_IDLE;

            WHEN OTHERS =>
                s_next_state <= ST_IDLE;

        END CASE;
    END PROCESS NEXT_STATE_LOGIC;

    -- =========================================================================
    -- OUTPUT LOGIC  (Moore — outputs depend only on current state)
    -- All outputs listed in defaults to prevent latch inference.
    -- =========================================================================
    OUTPUT_LOGIC : PROCESS (s_state)
    BEGIN
        -- Safe defaults: deassert everything
        o_en_in_mult     <= '0';
        o_en_out_mult    <= '0';
        o_en_in_sign     <= '0';
        o_en_out_sign    <= '0';
        o_en_in_sum      <= '0';
        o_en_out_sum     <= '0';
        o_ack_timer_mult <= '0';
        o_ack_timer_sign <= '0';
        o_ack_timer_sum  <= '0';
        o_clear_timers   <= '0';
        o_done           <= '0';

        CASE s_state IS

            WHEN ST_IDLE =>
                o_clear_timers <= '1';  -- Keep timers reset while idle

            WHEN ST_LOAD_MULT =>
                o_en_in_mult     <= '1';
                o_ack_timer_mult <= '1';  -- Arm MULT timer

            WHEN ST_WAIT_MULT =>
                NULL;  -- Wait for timer; no outputs

            WHEN ST_UNLOAD_MULT =>
                o_en_out_mult <= '1';

            WHEN ST_LOAD_SIGN =>
                o_en_in_sign     <= '1';
                o_ack_timer_sign <= '1';  -- Arm SIGN-EXT timer

            WHEN ST_WAIT_SIGN =>
                NULL;

            WHEN ST_UNLOAD_SIGN =>
                o_en_out_sign <= '1';

            WHEN ST_LOAD_SUM =>
                o_en_in_sum     <= '1';
                o_ack_timer_sum <= '1';   -- Arm SUM timer

            WHEN ST_WAIT_SUM =>
                NULL;

            WHEN ST_UNLOAD_SUM =>
                o_en_out_sum <= '1';

            WHEN ST_DONE =>
                o_done <= '1';  -- Single-cycle completion pulse

            WHEN OTHERS =>
                NULL;

        END CASE;
    END PROCESS OUTPUT_LOGIC;

END ARCHITECTURE rtl;
