library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pio32_in is
  port (
    clk               : in  std_logic;
    reset             : in  std_logic;
    avs_s0_read       : in  std_logic;
    avs_s0_readdata   : out std_logic_vector(31 downto 0);
    pio_in            : in  std_logic_vector(31 downto 0)
  );
end entity pio32_in;

architecture rtl of pio32_in is
  signal registered_pio : std_logic_vector(31 downto 0);
begin

  -- Proceso de registro sincronizado
  process(clk, reset)
  begin
    if reset = '1' then
      registered_pio <= (others => '0');
    elsif rising_edge(clk) then
      registered_pio <= pio_in;
    end if;
  end process;

  -- Salida continua (sin 'X')
  avs_s0_readdata <= registered_pio;

end architecture rtl;