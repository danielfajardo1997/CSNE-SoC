library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pio32_out is
  port (
    clk            : in  std_logic;
    reset          : in  std_logic;
    avs_s0_write   : in  std_logic;
    avs_s0_writedata : in  std_logic_vector(31 downto 0);
    pio_out        : out std_logic_vector(31 downto 0)
  );
end entity pio32_out;

architecture rtl of pio32_out is
  signal pio_out_reg : std_logic_vector(31 downto 0);
begin
  
  pio_out <= pio_out_reg;

process(clk)
begin
  if rising_edge(clk) then
    if reset = '1' then
      pio_out_reg <= (others => '0');
    elsif avs_s0_write = '1' then
      pio_out_reg <= avs_s0_writedata;
    end if;
  end if;
end process;

end architecture rtl;