library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity signed_16_to_32_ip_wrapped is
    Port (
        clock       : in  STD_LOGIC;
        reset       : in  STD_LOGIC;
        flush       : in  STD_LOGIC;
        en_in       : in  STD_LOGIC;
        en_out      : in  STD_LOGIC;
        data_16bit  : in  STD_LOGIC_VECTOR(15 downto 0);
        data_32bit  : out STD_LOGIC_VECTOR(31 downto 0)
    );
end entity;

architecture Behavioral of signed_16_to_32_ip_wrapped is
    
    signal data_16bit_reg : STD_LOGIC_VECTOR(15 downto 0);
    signal data_32bit_int : STD_LOGIC_VECTOR(31 downto 0);
    signal data_32bit_reg : STD_LOGIC_VECTOR(31 downto 0);
    
begin
    
    -- Registrar entradas
    process(clock, reset)
    begin
        if reset = '1' then
            data_16bit_reg <= (others => '0');
        elsif rising_edge(clock) then
            if flush = '1' then
                data_16bit_reg <= (others => '0');
            elsif en_in = '1' then
                data_16bit_reg <= data_16bit;
            end if;
        end if;
    end process;
    
    -- Conversión combinacional (extensión de signo)
    data_32bit_int <= (31 downto 16 => data_16bit_reg(15)) & data_16bit_reg;
    
    -- Registrar salida
    process(clock, reset)
    begin
        if reset = '1' then
            data_32bit_reg <= (others => '0');
        elsif rising_edge(clock) then
            if flush = '1' then
                data_32bit_reg <= (others => '0');
            elsif en_out = '1' then
                data_32bit_reg <= data_32bit_int;
            end if;
        end if;
    end process;
    
    data_32bit <= data_32bit_reg;
    
end Behavioral;