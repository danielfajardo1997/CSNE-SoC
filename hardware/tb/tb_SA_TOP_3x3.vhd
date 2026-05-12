-- =============================================================================
-- Module      : tb_SA_TOP_3x3
-- Project     : CSNE-SoC – Configurable Systolic Neural Engine
-- File        : tb_SA_TOP_3x3.vhd  (v2.0)
--
-- Description :
--   Functional testbench for SA_TOP_3x3 v2.0 with input capture registers.
--
--   Protocol:
--     1. Present stable data on i_a_row* and i_b_col*
--     2. Pulse i_start for ONE clock cycle  -> hardware latches all inputs
--     3. Wait for o_done
--     4. Read the 9 accumulators
--
--   For a full 3x3 matrix multiply C = A x B, 3 MAC operations are needed
--   (one per shared dimension k = 0, 1, 2). Each pulses start once and
--   accumulates inside the PEs. After k=2 the accumulators hold C.
--
--   Test Case 1: All positive
--     A = [[1,2,3],[4,5,6],[7,8,9]]
--     B = [[7,1,4],[8,2,5],[9,3,6]]
--     Expected C:
--       [50,  14,  32]
--       [122, 32,  77]
--       [194, 50, 122]
--
--   Test Case 2: Signed (mixed +/-)
--     A = [[-1,2,-3],[4,-5,6],[-7,8,-9]]
--     B = [[1,-2,3],[-4,5,-6],[7,-8,9]]
--     Expected C:
--       [-30,  36,  -42]
--       [ 66, -81,   96]
--       [-102, 126, -150]
--
-- Standards   : VHDL-2008
-- Author      : Daniel G. Fajardo Lopez
-- Institution : Pontificia Universidad Javeriana, Bogota D.C., Colombia
-- Date        : 2026-05-07
-- Version     : 2.0
-- =============================================================================

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY tb_SA_TOP_3x3 IS
END ENTITY tb_SA_TOP_3x3;

ARCHITECTURE sim OF tb_SA_TOP_3x3 IS

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

    CONSTANT C_CLK_PERIOD : time := 20 ns;

    SIGNAL tb_clk    : std_logic := '0';
    SIGNAL tb_rst_n  : std_logic := '0';
    SIGNAL tb_start  : std_logic := '0';
    SIGNAL tb_flush  : std_logic := '0';
    SIGNAL tb_done   : std_logic;

    SIGNAL tb_a_row0 : std_logic_vector(7 DOWNTO 0) := (OTHERS => '0');
    SIGNAL tb_a_row1 : std_logic_vector(7 DOWNTO 0) := (OTHERS => '0');
    SIGNAL tb_a_row2 : std_logic_vector(7 DOWNTO 0) := (OTHERS => '0');
    SIGNAL tb_b_col0 : std_logic_vector(7 DOWNTO 0) := (OTHERS => '0');
    SIGNAL tb_b_col1 : std_logic_vector(7 DOWNTO 0) := (OTHERS => '0');
    SIGNAL tb_b_col2 : std_logic_vector(7 DOWNTO 0) := (OTHERS => '0');

    SIGNAL tb_acc_00 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_01 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_02 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_10 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_11 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_12 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_20 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_21 : std_logic_vector(31 DOWNTO 0);
    SIGNAL tb_acc_22 : std_logic_vector(31 DOWNTO 0);

    -- -------------------------------------------------------------------------
    FUNCTION to_slv8(val : integer) RETURN std_logic_vector IS
    BEGIN
        RETURN std_logic_vector(to_signed(val, 8));
    END FUNCTION;

    PROCEDURE check_result(
        SIGNAL   acc_sig  : IN  std_logic_vector(31 DOWNTO 0);
        CONSTANT expected : IN  integer;
        CONSTANT name     : IN  string
    ) IS
        VARIABLE measured : integer;
    BEGIN
        measured := to_integer(signed(acc_sig));
        IF measured = expected THEN
            report "[PASS] " & name & " = " & integer'image(measured) &
                   "  (expected " & integer'image(expected) & ")" severity note;
        ELSE
            report "[FAIL] " & name & " = " & integer'image(measured) &
                   "  (expected " & integer'image(expected) & ")" severity error;
        END IF;
    END PROCEDURE check_result;

    -- -------------------------------------------------------------------------
    -- send_mac_op: loads one k-slice, pulses start, waits for done.
    -- Call once per column of A / row of B.
    -- -------------------------------------------------------------------------
    PROCEDURE send_mac_op(
        CONSTANT a0 : IN integer; CONSTANT a1 : IN integer; CONSTANT a2 : IN integer;
        CONSTANT b0 : IN integer; CONSTANT b1 : IN integer; CONSTANT b2 : IN integer;
        SIGNAL clk      : IN  std_logic;
        SIGNAL done_sig : IN  std_logic;
        SIGNAL a_row0   : OUT std_logic_vector(7 DOWNTO 0);
        SIGNAL a_row1   : OUT std_logic_vector(7 DOWNTO 0);
        SIGNAL a_row2   : OUT std_logic_vector(7 DOWNTO 0);
        SIGNAL b_col0   : OUT std_logic_vector(7 DOWNTO 0);
        SIGNAL b_col1   : OUT std_logic_vector(7 DOWNTO 0);
        SIGNAL b_col2   : OUT std_logic_vector(7 DOWNTO 0);
        SIGNAL start    : OUT std_logic
    ) IS
    BEGIN
        -- 1. Present stable data
        a_row0 <= to_slv8(a0); a_row1 <= to_slv8(a1); a_row2 <= to_slv8(a2);
        b_col0 <= to_slv8(b0); b_col1 <= to_slv8(b1); b_col2 <= to_slv8(b2);
        WAIT UNTIL rising_edge(clk);

        -- 2. One-cycle start pulse -> hardware latches inputs
        start <= '1';
        WAIT UNTIL rising_edge(clk);
        start <= '0';

        -- 3. Wait for pipeline to complete.
        -- All 9 PEs finish at the same cycle (broadcast architecture, no skew).
        -- One extra cycle after done for output register stability.
        WAIT UNTIL done_sig = '1';
        WAIT UNTIL rising_edge(clk);
    END PROCEDURE send_mac_op;

BEGIN

    u_dut : SA_TOP_3x3
        GENERIC MAP (G_MULT_CYCLES => 5, G_SIGN_CYCLES => 2, G_SUM_CYCLES => 3)
        PORT MAP (
            i_clk    => tb_clk,    i_rst_n  => tb_rst_n,
            i_start  => tb_start,  i_flush  => tb_flush,
            o_done   => tb_done,
            i_a_row0 => tb_a_row0, i_a_row1 => tb_a_row1, i_a_row2 => tb_a_row2,
            i_b_col0 => tb_b_col0, i_b_col1 => tb_b_col1, i_b_col2 => tb_b_col2,
            o_acc_00 => tb_acc_00, o_acc_01 => tb_acc_01, o_acc_02 => tb_acc_02,
            o_acc_10 => tb_acc_10, o_acc_11 => tb_acc_11, o_acc_12 => tb_acc_12,
            o_acc_20 => tb_acc_20, o_acc_21 => tb_acc_21, o_acc_22 => tb_acc_22
        );

    CLK_GEN : PROCESS
    BEGIN
        tb_clk <= '0'; WAIT FOR C_CLK_PERIOD / 2;
        tb_clk <= '1'; WAIT FOR C_CLK_PERIOD / 2;
    END PROCESS CLK_GEN;

    STIMULUS : PROCESS
    BEGIN
        -- Reset
        tb_rst_n <= '0'; WAIT FOR 100 ns;
        tb_rst_n <= '1'; WAIT FOR C_CLK_PERIOD * 2;

        -- ================================================================
        -- TEST CASE 1: All positive
        -- A = [[1,2,3],[4,5,6],[7,8,9]]  B = [[7,1,4],[8,2,5],[9,3,6]]
        -- ================================================================
        report "==========================================" severity note;
        report "TEST CASE 1 - All positive integers"       severity note;
        report "==========================================" severity note;

        tb_flush <= '1'; WAIT FOR C_CLK_PERIOD;
        tb_flush <= '0'; WAIT FOR C_CLK_PERIOD;

        -- k=0: A[:,0]={1,4,7}  B[0,:]={7,1,4}
        send_mac_op(1,4,7,  7,1,4,
            tb_clk, tb_done,
            tb_a_row0,tb_a_row1,tb_a_row2,
            tb_b_col0,tb_b_col1,tb_b_col2, tb_start);

        -- k=1: A[:,1]={2,5,8}  B[1,:]={8,2,5}
        send_mac_op(2,5,8,  8,2,5,
            tb_clk, tb_done,
            tb_a_row0,tb_a_row1,tb_a_row2,
            tb_b_col0,tb_b_col1,tb_b_col2, tb_start);

        -- k=2: A[:,2]={3,6,9}  B[2,:]={9,3,6}
        send_mac_op(3,6,9,  9,3,6,
            tb_clk, tb_done,
            tb_a_row0,tb_a_row1,tb_a_row2,
            tb_b_col0,tb_b_col1,tb_b_col2, tb_start);

        report "--- Results Test Case 1 ---" severity note;
        check_result(tb_acc_00,  50, "C[0][0]");
        check_result(tb_acc_01,  14, "C[0][1]");
        check_result(tb_acc_02,  32, "C[0][2]");
        check_result(tb_acc_10, 122, "C[1][0]");
        check_result(tb_acc_11,  32, "C[1][1]");
        check_result(tb_acc_12,  77, "C[1][2]");
        check_result(tb_acc_20, 194, "C[2][0]");
        check_result(tb_acc_21,  50, "C[2][1]");
        check_result(tb_acc_22, 122, "C[2][2]");

        WAIT FOR C_CLK_PERIOD * 5;

        -- ================================================================
        -- TEST CASE 2: Signed values
        -- A = [[-1,2,-3],[4,-5,6],[-7,8,-9]]
        -- B = [[1,-2,3],[-4,5,-6],[7,-8,9]]
        -- ================================================================
        report "==========================================" severity note;
        report "TEST CASE 2 - Signed (mixed +/-)"          severity note;
        report "==========================================" severity note;

        tb_flush <= '1'; WAIT FOR C_CLK_PERIOD;
        tb_flush <= '0'; WAIT FOR C_CLK_PERIOD;

        -- k=0: A[:,0]={-1,4,-7}  B[0,:]={1,-2,3}
        send_mac_op(-1,4,-7,  1,-2,3,
            tb_clk, tb_done,
            tb_a_row0,tb_a_row1,tb_a_row2,
            tb_b_col0,tb_b_col1,tb_b_col2, tb_start);

        -- k=1: A[:,1]={2,-5,8}  B[1,:]={-4,5,-6}
        send_mac_op(2,-5,8,  -4,5,-6,
            tb_clk, tb_done,
            tb_a_row0,tb_a_row1,tb_a_row2,
            tb_b_col0,tb_b_col1,tb_b_col2, tb_start);

        -- k=2: A[:,2]={-3,6,-9}  B[2,:]={7,-8,9}
        send_mac_op(-3,6,-9,  7,-8,9,
            tb_clk, tb_done,
            tb_a_row0,tb_a_row1,tb_a_row2,
            tb_b_col0,tb_b_col1,tb_b_col2, tb_start);

        report "--- Results Test Case 2 ---" severity note;
        check_result(tb_acc_00,  -30, "C[0][0]");
        check_result(tb_acc_01,   36, "C[0][1]");
        check_result(tb_acc_02,  -42, "C[0][2]");
        check_result(tb_acc_10,   66, "C[1][0]");
        check_result(tb_acc_11,  -81, "C[1][1]");
        check_result(tb_acc_12,   96, "C[1][2]");
        check_result(tb_acc_20, -102, "C[2][0]");
        check_result(tb_acc_21,  126, "C[2][1]");
        check_result(tb_acc_22, -150, "C[2][2]");

        WAIT FOR C_CLK_PERIOD * 5;
        report "==========================================" severity note;
        report "SIMULATION COMPLETE"                        severity note;
        report "==========================================" severity note;
        WAIT;

    END PROCESS STIMULUS;

END ARCHITECTURE sim;