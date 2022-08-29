library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.stop;

entity core_test_tb is
end entity;

architecture rtl of core_test_tb is
  signal clk : std_logic := '0';
  signal clk2 : std_logic := '0';

  constant period : time := 140 ns;
  constant half_period : time := period / 2;

  constant period2 : time := 1000 ns;
  constant half_period2 : time := period2 / 2;

  signal reset_n : std_logic := '0';

  signal bridge_wr : std_logic := '0';
  signal bridge_endian_little : std_logic := '0';
  signal bridge_addr : unsigned (31 downto 0);
  signal bridge_rd_data : unsigned (31 downto 0);
  signal bridge_wr_data : unsigned (31 downto 0);

  constant apf_word_time : time := 4000 ns;

  shared variable apf_write_buffer : unsigned (31 downto 0) := 32b"0";
  shared variable apf_write_buffer_fill : integer := 0;
begin
  clk <= not clk after half_period;
  clk2 <= not clk2 after half_period2;

  UUT : entity work.core_test port map (
    clk_74a => clk,
    clk_avr_16 => clk2,

    reset_n => reset_n,

    bridge_endian_little => bridge_endian_little,
    bridge_addr => bridge_addr,
    bridge_rd => '0',
    bridge_rd_data => bridge_rd_data,
    bridge_wr => bridge_wr,
    bridge_wr_data => bridge_wr_data
    );

  process
    -- Writes a 4 byte word to APF
    procedure apf_write_word(word : unsigned (31 downto 0)) is
    begin
      -- report "Sending 0x" & to_hstring(apf_write_buffer);

      bridge_wr <= '1';
      bridge_wr_data <= word;

      wait for period;

      bridge_wr <= '0';
      bridge_wr_data <= 32b"0";
      bridge_addr <= bridge_addr + 4;

      wait for apf_word_time;
    end procedure;

    -- Buffers a byte for writing to APF. Will send 4 byte word when full
    procedure apf_prepare_write_byte(byte : unsigned (7 downto 0)) is
    begin
      apf_write_buffer(7 downto 0) := byte;

      if apf_write_buffer_fill = 3 then
        apf_write_word(apf_write_buffer);

        apf_write_buffer_fill := 0;
        apf_write_buffer := 32b"0";
      else
        apf_write_buffer := shift_left(apf_write_buffer, 8);
        apf_write_buffer_fill := apf_write_buffer_fill + 1;
      end if;
    end procedure;

    -- Sends any buffered writes over APF
    procedure apf_finalize is
    begin
      if apf_write_buffer_fill > 0 then
        -- Unshift last shift
        apf_write_buffer := shift_right(apf_write_buffer, 8);
        apf_write_word(apf_write_buffer);
      end if;
    end procedure;

    procedure read_file is
      type file_type is file of integer;
      file rom_file : file_type open read_mode is "castleboy.bin";
      variable output : integer;
      variable output_vec : signed (31 downto 0);
    begin
      bridge_endian_little <= '1';

      while not endfile(rom_file) loop
        read(rom_file, output);
        output_vec := to_signed(output, 32);

        apf_write_word(unsigned(output_vec));
      end loop;
    end procedure;
  begin
    bridge_addr <= 32b"0";

    wait for period * 2;

    read_file;

    wait for period * 2;

    reset_n <= '1';

    wait for 100 ms;

    stop;
  end process;
end architecture;