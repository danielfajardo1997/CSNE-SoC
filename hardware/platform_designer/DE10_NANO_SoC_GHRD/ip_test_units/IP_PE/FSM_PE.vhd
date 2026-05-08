library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
 
-- ============================================================
-- FSM_PE: Finite State Machine for the Processing Element (PE)
-- Controls the datapath enable signals for:
--   MULT  -> Multiplier module
--   SIGN  -> Sign-extension module
--   SUM   -> Accumulator/adder module
-- ============================================================
 
entity FSM_PE is
    port(
        -- --------------------------------------------------------
        -- Global signals
        -- --------------------------------------------------------
        clock           : in  std_logic;
        reset           : in  std_logic;
        start           : in  std_logic;
 
        -- --------------------------------------------------------
        -- Datapath enable signals (PE control)
        -- --------------------------------------------------------
        en_in_mult      : out std_logic;   -- Enable data input  to MULT
        en_out_mult     : out std_logic;   -- Enable data output from MULT
        en_in_sign      : out std_logic;   -- Enable data input  to SIGN-EXT
        en_out_sign     : out std_logic;   -- Enable data output from SIGN-EXT
        en_in_sum       : out std_logic;   -- Enable data input  to SUM
        en_out_sum      : out std_logic;   -- Enable data output from SUM
 
        -- --------------------------------------------------------
        -- Timer / ready handshake signals
        -- NOTE: These must be driven by external timer modules.
        --       Declare them here if they are true ports, or
        --       move them to internal signals if generated inside
        --       this entity.
        -- --------------------------------------------------------
		  ip_ready        : out std_logic;
        timer_done_mult : in  std_logic;   -- '1' when MULT operation is done
        ack_timer_mult  : out std_logic;   -- Acknowledge / clear MULT timer
 
        timer_done_sign : in  std_logic;   -- '1' when SIGN-EXT operation is done
        ack_timer_sign  : out std_logic;   -- Acknowledge / clear SIGN-EXT timer
 
        timer_done_sum  : in  std_logic;   -- '1' when SUM  operation is done
        ack_timer_sum   : out std_logic;   -- Acknowledge / clear SUM timer
		  clear_timers   : out std_logic
 

    );
end FSM_PE;
 
architecture FSM_PE_arch of FSM_PE is
 
    -- ============================================================
    -- State encoding
    -- Sequence: IDLE -> load MULT inputs -> wait MULT -> unload
    --           MULT -> load SIGN inputs -> wait SIGN -> unload
    --           SIGN -> load SUM  inputs -> wait SUM  -> unload
    --           SUM  -> IDLE
    -- ============================================================
    type state_type is (
        IDLE,               -- Wait for start
        LOAD_MULT_INPUT,    -- Assert en_in_mult; latch data into multiplier
        WAIT_MULT_DONE,     -- Hold until multiplier signals completion
        UNLOAD_MULT_OUTPUT, -- Assert en_out_mult; move result downstream
        LOAD_SIGN_INPUT,    -- Assert en_in_sign; latch data into sign-extender
        WAIT_SIGN_DONE,     -- Hold until sign-extender signals completion
        UNLOAD_SIGN_OUTPUT, -- Assert en_out_sign; move result downstream
        LOAD_SUM_INPUT,     -- Assert en_in_sum;  latch data into adder
        WAIT_SUM_DONE,      -- Hold until adder signals completion
        UNLOAD_SUM_OUTPUT   -- Assert en_out_sum; write final result; return to IDLE
    );
 
    signal current_state : state_type;
    signal next_state    : state_type;
 
begin
 
    -- ============================================================
    -- STATE REGISTER
    -- ============================================================
    STATE_REGISTER : process(clock, reset)
    begin
        if reset = '1' then
            current_state <= IDLE;
        elsif rising_edge(clock) then
            current_state <= next_state;
        end if;
    end process STATE_REGISTER;
 
    -- ============================================================
    -- NEXT-STATE LOGIC
    -- ============================================================
    NEXT_STATE_LOGIC : process(
        current_state,
        start,
        timer_done_mult,
        timer_done_sign,
        timer_done_sum
    )
    begin
        -- Default: stay in current state (prevents unintended latches)
        next_state <= current_state;
 
        case current_state is
 
            when IDLE =>
                if start = '1' then
                    next_state <= LOAD_MULT_INPUT;
                end if;
 
            when LOAD_MULT_INPUT =>
                next_state <= WAIT_MULT_DONE;
 
            when WAIT_MULT_DONE =>
                if timer_done_mult = '1' then
                    next_state <= UNLOAD_MULT_OUTPUT;
                end if;
 
            when UNLOAD_MULT_OUTPUT =>
                next_state <= LOAD_SIGN_INPUT;
 
            when LOAD_SIGN_INPUT =>
                next_state <= WAIT_SIGN_DONE;
 
            when WAIT_SIGN_DONE =>
                if timer_done_sign = '1' then
                    next_state <= UNLOAD_SIGN_OUTPUT;
                end if;
 
            when UNLOAD_SIGN_OUTPUT =>
                next_state <= LOAD_SUM_INPUT;
 
            when LOAD_SUM_INPUT =>
                next_state <= WAIT_SUM_DONE;
 
            when WAIT_SUM_DONE =>
                if timer_done_sum = '1' then
                    next_state <= UNLOAD_SUM_OUTPUT;
                end if;
 
            when UNLOAD_SUM_OUTPUT =>
                next_state <= IDLE;
 
            when others =>
                next_state <= IDLE;
 
        end case;
    end process NEXT_STATE_LOGIC;
 
    -- ============================================================
    -- OUTPUT LOGIC  (Moore machine — outputs depend only on state)
    -- ============================================================
    OUTPUT_LOGIC : process(current_state)
    begin
        -- --------------------------------------------------------
        -- Safe defaults: deassert everything.
        -- This block MUST list every output driven here so that
        -- synthesis does not infer unwanted latches.
        -- --------------------------------------------------------
        en_in_mult     <= '0';
        en_out_mult    <= '0';
        en_in_sign     <= '0';
        en_out_sign    <= '0';
        en_in_sum      <= '0';
        en_out_sum     <= '0';
        ack_timer_mult <= '0';
        ack_timer_sign <= '0';
        ack_timer_sum  <= '0';
        clear_timers  <= '0';
		  ip_ready       <='0';

 
        case current_state is
 
            when IDLE =>
                clear_timers <= '1';         -- Reset any stale flags on entry
					 ip_ready<='1';
 
            when LOAD_MULT_INPUT =>
                en_in_mult     <= '1';
                ack_timer_mult <= '1';      -- Arm / reset the MULT timer
 
            when WAIT_MULT_DONE =>
                null;                       -- Wait; no outputs to assert
 
            when UNLOAD_MULT_OUTPUT =>
                en_out_mult <= '1';
 
            when LOAD_SIGN_INPUT =>
                en_in_sign     <= '1';
                ack_timer_sign <= '1';      -- Arm / reset the SIGN-EXT timer
 
            when WAIT_SIGN_DONE =>
                null;
 
            when UNLOAD_SIGN_OUTPUT =>
                en_out_sign <= '1';
 
            when LOAD_SUM_INPUT =>
                en_in_sum     <= '1';
                ack_timer_sum <= '1';       -- Arm / reset the SUM timer
 
            when WAIT_SUM_DONE =>
                null;
 
            when UNLOAD_SUM_OUTPUT =>
                en_out_sum <= '1';
 
            when others =>
                null;
 
        end case;
    end process OUTPUT_LOGIC;
 
end FSM_PE_arch;
