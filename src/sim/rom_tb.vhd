
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.env.stop;

entity rom_tb is
end entity;

architecture rtl of rom_tb is
  signal clk : std_logic := '0';
  signal clk2 : std_logic := '0';

  signal address_a : unsigned (14 downto 0) := 15b"0";
  signal address_b : unsigned (13 downto 0) := 14b"0";
  signal data_a : unsigned (7 downto 0) := 8b"0";
  signal data_b : unsigned (15 downto 0) := 16b"0";

  signal wren_a : std_logic := '0';

  signal output_a : unsigned (7 downto 0);
  signal output_b : unsigned (15 downto 0);

  constant period : time := 140 ns;
  constant half_period : time := period / 2;

  constant period2 : time := 1000 ns;
  constant half_period2 : time := period2 / 2;
begin
  clk <= not clk after half_period;
  clk2 <= not clk2 after half_period2;

  UUT : entity work.rom port map (
    address_a => address_a,
    address_b => address_b,

    clock_a => clk,
    clock_b => clk2,

    data_a => data_a,
    data_b => data_b,
    wren_a => wren_a,
    wren_b => '0',

    q_a => output_a,
    q_b => output_b
    );

  process
  begin
    for i in 0 to 255 loop
      wren_a <= '1';
      address_a <= to_unsigned(i, 15);
      data_a <= to_unsigned(i, 8);

      wait for period;
      wren_a <= '0';
      wait for period;
    end loop;

    wait for 2 * period;

    for i in 0 to 255 loop
      address_a <= to_unsigned(i, 15);
      wait for 2 * period;

      assert output_a = to_unsigned(i, 8) report "Error at 0x" & to_hstring(address_a);
    end loop;

    for i in 0 to 127 loop
      address_b <= to_unsigned(i, 14);
      wait for 2 * period2;

      assert output_b(7 downto 0) = to_unsigned(i * 2, 8) and output_b(15 downto 8) = to_unsigned(i * 2 + 1, 8) report "Error2 at 0x" & to_hstring(address_b);
    end loop;
    -- wren_a <= '1';
    -- wait for period;
    -- wren_a <= '0';

    -- address_a <= address_a + 1;
    -- wait for period;

    -- wren_a <= '1';
    -- wait for period;
    -- wren_a <= '0';
    stop;
  end process;

end architecture;