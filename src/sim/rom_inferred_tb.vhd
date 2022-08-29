library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.stop;

entity rom_inferred_tb is
end entity;

architecture rtl of rom_inferred_tb is
  signal clk : std_logic := '0';
  signal clk2 : std_logic := '0';

  constant period : time := 140 ns;
  constant half_period : time := period / 2;

  constant period2 : time := 100 ns;
  constant half_period2 : time := period2 / 2;

  signal write_addr : unsigned (13 downto 0) := 14b"0";
  signal read_addr : unsigned (13 downto 0) := 14b"0";

  signal write_en : std_logic;
  signal write_data : unsigned (15 downto 0) := 16b"0";
  -- signal data_b : unsigned (15 downto 0) := 16b"0";

  signal output_a : unsigned (15 downto 0);
  signal read_data : unsigned (15 downto 0);
begin
  clk <= not clk after half_period;
  clk2 <= not clk2 after half_period2;

  INFERRED : entity work.inferred_rom port map (
    clk_write => clk,
    clk_read => clk2,

    write_en => write_en,
    write_addr => write_addr,
    write_data => write_data,

    read_addr => read_addr,
    read_data => read_data
    );

  -- ROM : entity work.rom port map (
--   address_a => write_addr,
--   address_b => read_addr,

--   clock_a => clk,
--   clock_b => clk2,

--   data_a => write_data,
--   data_b => 16b"0",
--   wren_a => write_en,
--   wren_b => '0',

--   q_a => output_a,
--   q_b => read_data
--   );

  process
  begin
    for i in 0 to 15 loop
      write_addr <= to_unsigned(i, 14);
      write_data <= to_unsigned(i + 16, 8) & to_unsigned(i, 8);
      write_en <= '1';

      wait for period;

      write_en <= '0';
      write_addr <= 14x"3F";
      write_data <= 16x"FF";

      wait for period;
    end loop;

    for i in 0 to 15 loop
      read_addr <= to_unsigned(i, 14);

      wait for 4 * period2;

      assert read_data = to_unsigned(i + 16, 8) & to_unsigned(i, 8) report "Mismatch at " & integer'image(i);
    end loop;

    stop;
  end process;
end architecture;