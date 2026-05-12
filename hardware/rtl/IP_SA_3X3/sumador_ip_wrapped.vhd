library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity sumador_ip_wrapped is
    Port (
        clock    : in  STD_LOGIC;
        reset  : in  STD_LOGIC;
        flush  : in  STD_LOGIC;    -- Nueva señal para limpiar registros
        en_in  : in  STD_LOGIC;    -- Enable para entrada
        en_out : in  STD_LOGIC;    -- Enable para salida
        a      : in  STD_LOGIC_VECTOR(31 downto 0);
        b      : in  STD_LOGIC_VECTOR(31 downto 0);
        s      : out STD_LOGIC_VECTOR(31 downto 0)
    );
end entity;

architecture Behavioral of sumador_ip_wrapped is
    component int32_add
        port (
            dataa  : in  STD_LOGIC_VECTOR(31 downto 0);
            datab  : in  STD_LOGIC_VECTOR(31 downto 0);
            result : out STD_LOGIC_VECTOR(31 downto 0)
        );
    end component;
    
    signal a_reg, b_reg : STD_LOGIC_VECTOR(31 downto 0);
    signal s_int        : STD_LOGIC_VECTOR(31 downto 0);
    signal s_reg        : STD_LOGIC_VECTOR(31 downto 0);
    
begin
    -- Registrar entradas con enable_in y flush
    process(clock, reset)
    begin
        if reset = '1' then
            a_reg <= (others => '0');
            b_reg <= (others => '0');
        elsif rising_edge(clock) then
            if flush = '1' then
                a_reg <= (others => '0');
                b_reg <= (others => '0');
            elsif en_in = '1' then
                a_reg <= a;
                b_reg <= b;
            end if;
        end if;
    end process;
    
    -- Instanciar IP combinacional
    sum_inst : int32_add
        port map (
            dataa  => a_reg,
            datab  => b_reg,
            result => s_int
        );
    
    -- Registrar salida con enable_out y flush
    process(clock, reset)
    begin
        if reset = '1' then
            s_reg <= (others => '0');
        elsif rising_edge(clock) then
            if flush = '1' then
                s_reg <= (others => '0');
            elsif en_out = '1' then
                s_reg <= s_int;
            end if;
        end if;
    end process;
    
    s <= s_reg;
    
end Behavioral;