-- =============================================================================
-- Module      : SystolicArray_3x3
-- Project     : CSNE-SoC – Configurable Systolic Neural Engine
-- File        : SystolicArray_3x3.vhd
-- Version     : 3.0
--
-- Description :
--   3x3 MAC array implementing C = A x B via 3 sequential k-slice operations.
--
--   Architecture — Capture-Broadcast:
--   Each start pulse captures ONE k-slice of the matrix multiply:
--     i_a_row r = A[r][k]  for r = 0,1,2
--     i_b_col c = B[k][c]  for c = 0,1,2
--
--   Captured values are BROADCAST simultaneously to all 9 PEs:
--     PE(r,c) receives a=A[r][k] and b=B[k][c]
--     PE(r,c) computes: acc += A[r][k] * B[k][c]
--
--   After K=3 start pulses:
--     PE(r,c).acc = sum_k( A[r][k]*B[k][c] ) = C[r][c]  (correct result)
--
--   All 9 PEs operate in LOCKSTEP — same enables, same timing, no skew needed.
--   output_a and output_b ports of each PE are unused (OPEN) in this design.
--
--   Why no skew registers:
--     Skew is needed in STREAMING systolic arrays where data flows PE-to-PE
--     each cycle. In this CAPTURE design, each PE directly receives its own
--     operands from the broadcast registers. No inter-PE data dependency.
--
--   HPS driver protocol (per matrix multiply):
--     For k = 0 to 2:
--       1. Load i_a_row* with A[:,k]  and  i_b_col* with B[k,:]
--       2. Pulse i_start (1 cycle) — hardware captures all 6 inputs
--       3. Wait for o_done from SA_TOP_3x3
--     After k=2: read 9 accumulators = matrix C
--
-- Change log:
--   v1.0 – streaming systolic with skew (incorrect for this PE design)
--   v2.0 – capture registers added, skew kept (still incorrect)
--   v3.0 – removed skew, correct broadcast architecture
--
-- Standards   : VHDL-2008, IEEE Std 1076-2008
-- Author      : Daniel G. Fajardo Lopez
-- Institution : Pontificia Universidad Javeriana, Bogota D.C., Colombia
-- Date        : 2026-05-11
-- =============================================================================

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY SystolicArray_3x3 IS
    PORT (
        i_clk         : IN  std_logic;
        i_rst_n       : IN  std_logic;
        i_start       : IN  std_logic;
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
END ENTITY SystolicArray_3x3;

ARCHITECTURE rtl OF SystolicArray_3x3 IS

    COMPONENT PE IS
        PORT (
            clock       : IN  std_logic;
            reset       : IN  std_logic;
            en_in_mult  : IN  std_logic;
            en_in_sum   : IN  std_logic;
            en_in_sign  : IN  std_logic;
            en_out_mult : IN  std_logic;
            en_out_sum  : IN  std_logic;
            en_out_sign : IN  std_logic;
            flush       : IN  std_logic;
            a           : IN  std_logic_vector(7 DOWNTO 0);
            b           : IN  std_logic_vector(7 DOWNTO 0);
            acc         : OUT std_logic_vector(31 DOWNTO 0);
            output_a    : OUT std_logic_vector(7 DOWNTO 0);
            output_b    : OUT std_logic_vector(7 DOWNTO 0)
        );
    END COMPONENT PE;

    SIGNAL s_reset      : std_logic;

    -- Input capture registers — latched on i_start, broadcast to all PEs
    SIGNAL s_cap_a_row0 : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_cap_a_row1 : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_cap_a_row2 : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_cap_b_col0 : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_cap_b_col1 : std_logic_vector(7 DOWNTO 0);
    SIGNAL s_cap_b_col2 : std_logic_vector(7 DOWNTO 0);

BEGIN

    s_reset <= NOT i_rst_n;

    -- =========================================================================
    -- Input capture registers
    -- Latch all 6 inputs on i_start. Values remain stable for ~16 cycles
    -- until the pipeline completes. Broadcast to all 9 PEs simultaneously.
    -- =========================================================================
    INPUT_CAPTURE : PROCESS (i_clk, i_rst_n)
    BEGIN
        IF i_rst_n = '0' THEN
            s_cap_a_row0 <= (OTHERS => '0');
            s_cap_a_row1 <= (OTHERS => '0');
            s_cap_a_row2 <= (OTHERS => '0');
            s_cap_b_col0 <= (OTHERS => '0');
            s_cap_b_col1 <= (OTHERS => '0');
            s_cap_b_col2 <= (OTHERS => '0');
        ELSIF rising_edge(i_clk) THEN
            IF i_start = '1' THEN
                s_cap_a_row0 <= i_a_row0;
                s_cap_a_row1 <= i_a_row1;
                s_cap_a_row2 <= i_a_row2;
                s_cap_b_col0 <= i_b_col0;
                s_cap_b_col1 <= i_b_col1;
                s_cap_b_col2 <= i_b_col2;
            END IF;
        END IF;
    END PROCESS INPUT_CAPTURE;

    -- =========================================================================
    -- PE grid — broadcast: PE(r,c) gets a=A[r,k] and b=B[k,c]
    -- All PEs use the same enable bus (lockstep operation).
    -- output_a and output_b are unused in this architecture.
    -- =========================================================================

    -- Row 0
    u_pe_00: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row0, b=>s_cap_b_col0, acc=>o_acc_00,
        output_a=>OPEN, output_b=>OPEN);

    u_pe_01: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row0, b=>s_cap_b_col1, acc=>o_acc_01,
        output_a=>OPEN, output_b=>OPEN);

    u_pe_02: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row0, b=>s_cap_b_col2, acc=>o_acc_02,
        output_a=>OPEN, output_b=>OPEN);

    -- Row 1
    u_pe_10: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row1, b=>s_cap_b_col0, acc=>o_acc_10,
        output_a=>OPEN, output_b=>OPEN);

    u_pe_11: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row1, b=>s_cap_b_col1, acc=>o_acc_11,
        output_a=>OPEN, output_b=>OPEN);

    u_pe_12: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row1, b=>s_cap_b_col2, acc=>o_acc_12,
        output_a=>OPEN, output_b=>OPEN);

    -- Row 2
    u_pe_20: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row2, b=>s_cap_b_col0, acc=>o_acc_20,
        output_a=>OPEN, output_b=>OPEN);

    u_pe_21: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row2, b=>s_cap_b_col1, acc=>o_acc_21,
        output_a=>OPEN, output_b=>OPEN);

    u_pe_22: PE PORT MAP (clock=>i_clk, reset=>s_reset, flush=>i_flush,
        en_in_mult=>i_en_in_mult, en_out_mult=>i_en_out_mult,
        en_in_sign=>i_en_in_sign, en_out_sign=>i_en_out_sign,
        en_in_sum=>i_en_in_sum,   en_out_sum=>i_en_out_sum,
        a=>s_cap_a_row2, b=>s_cap_b_col2, acc=>o_acc_22,
        output_a=>OPEN, output_b=>OPEN);

END ARCHITECTURE rtl;