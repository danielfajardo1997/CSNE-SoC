library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity int8_mult_wrapped is
    Port (
        clock       : in  STD_LOGIC;
        reset     : in  STD_LOGIC;  -- Activo alto
        flush     : in  STD_LOGIC;
        en_in     : in  STD_LOGIC;  -- Enable para entrada
        en_out    : in  STD_LOGIC;  -- Enable para salida
        a         : in  STD_LOGIC_VECTOR(7 downto 0);
        b         : in  STD_LOGIC_VECTOR(7 downto 0);
        p         : out STD_LOGIC_VECTOR(15 downto 0)
    );
end entity;

architecture Behavioral of int8_mult_wrapped is
    
    COMPONENT int8_mult
        PORT (
            dataa   : IN STD_LOGIC_VECTOR (7 DOWNTO 0);
            datab   : IN STD_LOGIC_VECTOR (7 DOWNTO 0);
            result  : OUT STD_LOGIC_VECTOR (15 DOWNTO 0)
        );
    END COMPONENT;
    
    signal a_reg, b_reg : STD_LOGIC_VECTOR(7 downto 0);
    signal mult_result  : STD_LOGIC_VECTOR(15 downto 0);
    signal p_reg        : STD_LOGIC_VECTOR(15 downto 0);
    
begin
    
    -- Registrar entradas con enable_in
    process(clock, reset)
    begin
        if reset = '1' then
            a_reg <= (others => '0');
            b_reg <= (others => '0');
        elsif rising_edge(clock) then
            if flush = '1' then
                a_reg <= (others => '0');
                b_reg <= (others => '0');
            else
                if en_in = '1' then
                    a_reg <= a;
                    b_reg <= b;
                end if;
            end if;
        end if;
    end process;
    
    -- Multiplicador combinacional
    mult_inst : int8_mult
        port map (
            dataa   => a_reg,
            datab   => b_reg,
            result  => mult_result
        );
    
    -- Registrar salida con enable_out
    process(clock, reset)
    begin
        if reset = '1' then
            p_reg <= (others => '0');
        elsif rising_edge(clock) then
            if flush = '1' then
                p_reg <= (others => '0');  -- ¡CORREGIDO! Debe ser vector de 16 bits
            else
                if en_out = '1' then
                    p_reg <= mult_result;
                end if;
            end if;
        end if;
    end process;
    
    p <= p_reg;
    
end Behavioral;