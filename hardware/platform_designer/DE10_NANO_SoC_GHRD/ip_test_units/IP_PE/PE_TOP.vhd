-------------------------------------------------------------------------------
-- PE_TOP.vhd
-- Top-level entity that integrates:
--   1. FSM_PE (Finite State Machine controller)
--   2. timer_MULT, timer_SIG, timer_ADD (timing modules)
--   3. PE (Processing Element datapath)
--
-- Architecture: Sequential control with timer-based handshaking
-- Target:      FPGA / ASIC (synthesizable)
-- Standard:    IEEE VHDL 1076-2008
--
-- Author:      Doctoral Researcher Daniel Fajardo
-- Date:        2026-05-06
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-------------------------------------------------------------------------------
-- ENTITY DECLARATION
-------------------------------------------------------------------------------
entity PE_TOP is
    port (
        -- ============================================================
        -- Global signals
        -- ============================================================
        clock               : in  std_logic;    -- Master clock
        reset               : in  std_logic;    -- Asynchronous reset (active high)
        start               : in  std_logic;    -- Start pulse for convolution
        
        -- ============================================================
        -- Data inputs (8-bit signed integers)
        -- ============================================================
        a_in                : in  std_logic_vector(7 downto 0);  -- Activation / pixel
        b_in                : in  std_logic_vector(7 downto 0);  -- Weight / kernel
        
        -- ============================================================
        -- Control / Status
        -- ============================================================
        flush               : in  std_logic;    -- Flush pending operations
		  ip_ready            : out std_logic;
        
        -- ============================================================
        -- Data outputs
        -- ============================================================
        acc_out             : out std_logic_vector(31 downto 0);  -- Accumulator result
        a_out               : out std_logic_vector(7 downto 0);   -- Forwarded activation
        b_out               : out std_logic_vector(7 downto 0)    -- Forwarded weight
        
        -- ============================================================
        -- Debug / Test points (optional, comment if not needed)
        -- ============================================================
        -- debug_state        : out std_logic_vector(3 downto 0);
        -- debug_timer_mult   : out std_logic;
        -- debug_timer_sig    : out std_logic;
        -- debug_timer_add    : out std_logic
    );
end entity PE_TOP;

-------------------------------------------------------------------------------
-- ARCHITECTURE
-------------------------------------------------------------------------------
architecture rtl of PE_TOP is

    ---------------------------------------------------------------------------
    -- Component declarations
    ---------------------------------------------------------------------------
    
    -- FSM Controller
    component FSM_PE is
        port (
            clock           : in  std_logic;
            reset           : in  std_logic;
            start           : in  std_logic;
            
            -- Datapath enable signals
            en_in_mult      : out std_logic;
            en_out_mult     : out std_logic;
            en_in_sign      : out std_logic;
            en_out_sign     : out std_logic;
            en_in_sum       : out std_logic;
            en_out_sum      : out std_logic;
            ip_ready        : out std_logic;
            -- Timer handshake
            timer_done_mult : in  std_logic;
            ack_timer_mult  : out std_logic;
            timer_done_sign : in  std_logic;
            ack_timer_sign  : out std_logic;
            timer_done_sum  : in  std_logic;
            ack_timer_sum   : out std_logic;
            clear_timers    : out std_logic
        );
    end component;
    
    -- Multiplier timer (5 cycles for INT8 multiplication)
    component timer_MULT is
        port (
            clock           : in  std_logic;
            reset           : in  std_logic;
            clear           : in  std_logic;
            start           : in  std_logic;
            done            : out std_logic
        );
    end component;
    
    -- Sign-extension timer (2 cycles for 16-bit to 32-bit conversion)
    component timer_SIG is
        port (
            clock           : in  std_logic;
            reset           : in  std_logic;
            clear           : in  std_logic;
            start           : in  std_logic;
            done            : out std_logic
        );
    end component;
    
    -- Adder/Accumulator timer (3 cycles for 32-bit addition)
    component timer_ADD is
        port (
            clock           : in  std_logic;
            reset           : in  std_logic;
            clear           : in  std_logic;
            start           : in  std_logic;
            done            : out std_logic
        );
    end component;
    
    -- Processing Element datapath
    component PE is
        port (
            clock           : in  std_logic;
            reset           : in  std_logic;
            en_in_mult      : in  std_logic;
            en_in_sum       : in  std_logic;
            en_in_sign      : in  std_logic;
            en_out_mult     : in  std_logic;
            en_out_sum      : in  std_logic;
            en_out_sign     : in  std_logic;
            flush           : in  std_logic;
            a               : in  std_logic_vector(7 downto 0);
            b               : in  std_logic_vector(7 downto 0);
            acc             : out std_logic_vector(31 downto 0);
            output_a        : out std_logic_vector(7 downto 0);
            output_b        : out std_logic_vector(7 downto 0)
        );
    end component;
    
    ---------------------------------------------------------------------------
    -- Internal signal declarations
    ---------------------------------------------------------------------------
    
    -- FSM to Datapath enables
    signal sig_en_in_mult      : std_logic;
    signal sig_en_out_mult     : std_logic;
    signal sig_en_in_sign      : std_logic;
    signal sig_en_out_sign     : std_logic;
    signal sig_en_in_sum       : std_logic;
    signal sig_en_out_sum      : std_logic;
	 signal sig_ip_ready        : std_logic;
    
    -- FSM to Timer handshake
    signal sig_timer_done_mult : std_logic;
    signal sig_ack_timer_mult  : std_logic;
    signal sig_timer_done_sign : std_logic;
    signal sig_ack_timer_sign  : std_logic;
    signal sig_timer_done_sum  : std_logic;
    signal sig_ack_timer_sum   : std_logic;
    signal sig_clear_timers    : std_logic;
    
    -- Timer start signals (derived from FSM acknowledges)
    signal sig_start_timer_mult : std_logic;
    signal sig_start_timer_sign : std_logic;
    signal sig_start_timer_sum  : std_logic;
    
    -- PE internal result (unused in top, but kept for clarity)
    signal sig_acc_internal    : std_logic_vector(31 downto 0);
    signal sig_a_passthrough   : std_logic_vector(7 downto 0);
    signal sig_b_passthrough   : std_logic_vector(7 downto 0);
    
    -- Done flag (when all three timers are idle and FSM back to IDLE)
    signal sig_fsm_idle        : std_logic;
    
    -- debug: state encoding (optional)
    -- signal debug_state_vector : std_logic_vector(3 downto 0);
    
begin

    ---------------------------------------------------------------------------
    -- Component instantiations
    ---------------------------------------------------------------------------
    
    -- ================================================================
    -- 1. FSM CONTROLLER
    -- ================================================================
    -- Manages the sequence of operations:
    --   MULT -> SIGN-EXT -> SUM
    -- Each stage waits for its respective timer to assert 'done'
    --------------------------------------------------------------------
    u_FSM_PE : FSM_PE
        port map (
            clock           => clock,
            reset           => reset,
            start           => start,
            
            -- Datapath control lines
            en_in_mult      => sig_en_in_mult,
            en_out_mult     => sig_en_out_mult,
            en_in_sign      => sig_en_in_sign,
            en_out_sign     => sig_en_out_sign,
            en_in_sum       => sig_en_in_sum,
            en_out_sum      => sig_en_out_sum,
				
				ip_ready        => sig_ip_ready,
            
            -- Timer status inputs
            timer_done_mult => sig_timer_done_mult,
            timer_done_sign => sig_timer_done_sign,
            timer_done_sum  => sig_timer_done_sum,
            
            -- Timer acknowledge outputs
            ack_timer_mult  => sig_ack_timer_mult,
            ack_timer_sign  => sig_ack_timer_sign,
            ack_timer_sum   => sig_ack_timer_sum,
            
            -- Global timer clear
            clear_timers    => sig_clear_timers
        );
    
    ---------------------------------------------------------------------------
    -- Timer start signal generation
    -- The FSM's ack_timer_* pulses are used as 'start' for each timer
    -- This is a 1-cycle pulse that triggers the counting sequence
    ---------------------------------------------------------------------------
    sig_start_timer_mult <= sig_ack_timer_mult;
    sig_start_timer_sign <= sig_ack_timer_sign;
    sig_start_timer_sum  <= sig_ack_timer_sum;
    
    -- ================================================================
    -- 2. MULTIPLIER TIMER
    -- ================================================================
    -- Counts clock cycles for INT8 multiplication operation.
    -- Latency: 5 clock cycles (counts 0,1,2,3,4)
    --   - Cycle 0: Setup (optional, based on pipeline depth)
    --   - Cycle 1-4: DSP slice / logic delay
    --------------------------------------------------------------------
    u_timer_mult : timer_MULT
        port map (
            clock           => clock,
            reset           => reset,
            clear           => sig_clear_timers,
            start           => sig_start_timer_mult,
            done            => sig_timer_done_mult
        );
    
    -- ================================================================
    -- 3. SIGN-EXTENSION TIMER
    -- ================================================================
    -- Counts clock cycles for 16-bit to 32-bit signed extension.
    -- Latency: 2 clock cycles (counts 0,1)
    --   - Cycle 0: Latch input
    --   - Cycle 1: Output valid
    --------------------------------------------------------------------
    u_timer_sign : timer_SIG
        port map (
            clock           => clock,
            reset           => reset,
            clear           => sig_clear_timers,
            start           => sig_start_timer_sign,
            done            => sig_timer_done_sign
        );
    
    -- ================================================================
    -- 4. ADDER / ACCUMULATOR TIMER
    -- ================================================================
    -- Counts clock cycles for 32-bit addition with accumulation.
    -- Latency: 3 clock cycles (counts 0,1,2)
    --   - Cycle 0: Input latchsig_timer_done_mult
    --   - Cycle 1: Addition operation
    --   - Cycle 2: Result valid
    --------------------------------------------------------------------
    u_timer_add : timer_ADD
        port map (
            clock           => clock,
            reset           => reset,
            clear           => sig_clear_timers,
            start           => sig_start_timer_sum,
            done            => sig_timer_done_sum
        );
    
    -- ================================================================
    -- 5. PROCESSING ELEMENT (DATAPATH)
    -- ================================================================
    -- Contains the actual computational modules:
    --   - INT8 multiplier
    --   - Signed 16-to-32 bit extender
    --   - 32-bit accumulator
    --------------------------------------------------------------------
    u_PE : PE
        port map (
            clock           => clock,
            reset           => reset,
            en_in_mult      => sig_en_in_mult,
            en_in_sum       => sig_en_in_sum,
            en_in_sign      => sig_en_in_sign,
            en_out_mult     => sig_en_out_mult,
            en_out_sum      => sig_en_out_sum,
            en_out_sign     => sig_en_out_sign,
            flush           => flush,
            a               => a_in,
            b               => b_in,
            acc             => sig_acc_internal,
            output_a        => sig_a_passthrough,
            output_b        => sig_b_passthrough
        );
    
    ---------------------------------------------------------------------------
    -- Output assignments
    ---------------------------------------------------------------------------
    acc_out <= sig_acc_internal;
    a_out   <= sig_a_passthrough;
    b_out   <= sig_b_passthrough;
    ip_ready <= sig_ip_ready;

    
end architecture rtl;

-------------------------------------------------------------------------------
-- END OF FILE PE_TOP.vhd
-------------------------------------------------------------------------------