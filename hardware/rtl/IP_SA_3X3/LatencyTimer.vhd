-- =============================================================================
-- Module      : LatencyTimer
-- Project     : Processing Element (PE) Control Subsystem
-- Description : Generic fixed-latency timer implemented as a Moore FSM.
--               After a rising edge on i_start, the timer counts G_CYCLES
--               clock cycles and then asserts o_done, holding it high until
--               a synchronous i_clear pulse returns the unit to IDLE.
--
--               The same source file is used for all three pipeline stages
--               (multiplier, sign-extender, adder) by setting the generic
--               G_CYCLES at instantiation time:
--                   Multiplier  : G_CYCLES = 5  (counts 0 .. 4)
--                   Sign-extend : G_CYCLES = 2  (counts 0 .. 1)
--                   Adder       : G_CYCLES = 3  (counts 0 .. 2)
--
-- Generics    : G_CYCLES   – number of clock cycles to wait before asserting
--                            o_done (minimum value: 1)
--
-- Ports
--   i_clk     – system clock (rising-edge triggered)
--   i_rst_n   – asynchronous active-low reset
--   i_clear   – synchronous active-high clear; returns FSM to IDLE immediately
--   i_start   – one-cycle pulse that arms the timer; ignored while counting
--   o_done    – asserted high when the count has elapsed; held until i_clear
--
-- Latency     : o_done is asserted exactly G_CYCLES clock cycles after the
--               rising edge of i_start (assuming i_clear is not asserted).
--
-- Standards   : VHDL-2008, IEEE Std 1076-2008
--               Naming follows the VHDL Style Guide (VSG) conventions:
--                 signals  -> snake_case with i_/o_ prefix for ports
--                 types    -> t_<name>
--                 constants-> C_<NAME>
--                 generics -> G_<NAME>
-- =============================================================================

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

-- -----------------------------------------------------------------------------
ENTITY LatencyTimer IS
    GENERIC (
        G_CYCLES : positive := 5  -- Number of clock cycles before o_done
    );
    PORT (
        i_clk   : IN  std_logic;  -- System clock
        i_rst_n : IN  std_logic;  -- Asynchronous reset, active-low
        i_clear : IN  std_logic;  -- Synchronous clear, active-high
        i_start : IN  std_logic;  -- Start pulse, active-high (one cycle)
        o_done  : OUT std_logic   -- Completion flag, held until i_clear
    );
END ENTITY LatencyTimer;
-- -----------------------------------------------------------------------------

ARCHITECTURE rtl OF LatencyTimer IS

    -- -------------------------------------------------------------------------
    -- Internal type definitions
    -- -------------------------------------------------------------------------

    -- FSM state encoding
    TYPE t_timer_state IS (
        ST_IDLE,      -- Waiting for i_start
        ST_COUNTING,  -- Counting down pipeline latency cycles
        ST_DONE       -- Latency elapsed; holding o_done until i_clear
    );

    -- -------------------------------------------------------------------------
    -- Constant definitions
    -- -------------------------------------------------------------------------

    -- Fixed 8-bit counter: supports G_CYCLES up to 255 — sufficient for all
    -- three pipeline stages (MULT=5, SIGN=2, SUM=3) and future extensions.
    -- Using a fixed width avoids the ieee.math_real dependency which some
    -- synthesizers do not support in elaboration-time constants.
    CONSTANT C_COUNT_WIDTH : positive := 8;

    -- Terminal count value
    CONSTANT C_COUNT_MAX : unsigned(C_COUNT_WIDTH - 1 DOWNTO 0) :=
        to_unsigned(G_CYCLES - 1, C_COUNT_WIDTH);

    -- -------------------------------------------------------------------------
    -- Internal signal declarations
    -- -------------------------------------------------------------------------

    SIGNAL r_state      : t_timer_state;                          -- Current FSM state
    SIGNAL r_next_state : t_timer_state;                          -- Next FSM state
    SIGNAL r_count      : unsigned(C_COUNT_WIDTH - 1 DOWNTO 0);  -- Cycle counter
    SIGNAL r_next_count : unsigned(C_COUNT_WIDTH - 1 DOWNTO 0);  -- Next counter value

BEGIN

    -- =========================================================================
    -- Process : STATE_REGISTER
    -- Type    : Sequential (clocked)
    -- Purpose : State and counter registers. Asynchronous active-low reset
    --           forces both the FSM and the counter to their initial values.
    -- =========================================================================
    STATE_REGISTER : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            r_state <= ST_IDLE;
            r_count <= (OTHERS => '0');
        ELSIF rising_edge(i_clk) THEN
            r_state <= r_next_state;
            r_count <= r_next_count;
        END IF;
    END PROCESS STATE_REGISTER;

    -- =========================================================================
    -- Process : NEXT_STATE_LOGIC
    -- Type    : Combinational
    -- Purpose : Computes the next FSM state and the next counter value.
    --           Synchronous i_clear has the highest priority and overrides
    --           all other transitions.
    -- =========================================================================
    NEXT_STATE_LOGIC : PROCESS (r_state, r_count, i_start, i_clear)
    BEGIN
        -- Default: hold current values (prevents unintended latches)
        r_next_state <= r_state;
        r_next_count <= r_count;

        -- Synchronous clear takes priority over normal operation
        IF i_clear = '1' THEN
            r_next_state <= ST_IDLE;
            r_next_count <= (OTHERS => '0');
        ELSE
            CASE r_state IS

                WHEN ST_IDLE =>
                    IF i_start = '1' THEN
                        r_next_state <= ST_COUNTING;
                        r_next_count <= (OTHERS => '0');
                    END IF;
                    -- No i_start: remain in ST_IDLE (handled by default above)

                WHEN ST_COUNTING =>
                    IF r_count = C_COUNT_MAX THEN
                        -- Final cycle reached; move to completion state
                        r_next_state <= ST_DONE;
                    ELSE
                        r_next_count <= r_count + 1;
                    END IF;

                WHEN ST_DONE =>
                    -- Hold completion until synchronous clear (handled above)
                    NULL;

                WHEN OTHERS =>
                    -- Unreachable; safe fallback
                    r_next_state <= ST_IDLE;
                    r_next_count <= (OTHERS => '0');

            END CASE;
        END IF;
    END PROCESS NEXT_STATE_LOGIC;

    -- =========================================================================
    -- Process : OUTPUT_LOGIC
    -- Type    : Combinational (Moore output — depends only on current state)
    -- Purpose : Asserts o_done only when the FSM is in ST_DONE.
    -- =========================================================================
    OUTPUT_LOGIC : PROCESS (r_state)
    BEGIN
        CASE r_state IS
            WHEN ST_DONE    => o_done <= '1';
            WHEN OTHERS     => o_done <= '0';
        END CASE;
    END PROCESS OUTPUT_LOGIC;

END ARCHITECTURE rtl;
