module core_test (
    input wire clk_74a,
    input wire clk_avr_16,

    input wire reset_n,

    input wire bridge_endian_little,
    input wire [31:0] bridge_addr,
    input wire bridge_rd,
    output reg [31:0] bridge_rd_data,
    input wire bridge_wr,
    input wire [31:0] bridge_wr_data,

    output wire [13:0] pgm_addr_out,
    output wire [15:0] pgm_data_out
  );

  // System ROM
  reg  [1:0][7:0] rom[16384];

  wire [13:0] pgm_addr;
  reg [15:0] pgm_data;

  assign pgm_addr_out = pgm_addr;
  assign pgm_data_out = pgm_data;

  always @ (posedge clk_avr_16) pgm_data <= rom[pgm_addr];

  wire write_en;
  wire [13:0] write_addr2;
  wire [15:0] write_data2;

  data_loader_16 #(.ADDRESS_MASK_UPPER_4(4'h0)) data_loader_16 (
                   .clk_74a(clk_74a),
                   .bridge_wr(bridge_wr),
                   .bridge_endian_little(bridge_endian_little),
                   .bridge_addr(bridge_addr),
                   .bridge_wr_data(bridge_wr_data),

                   .write_en(write_en),
                   .write_addr(write_addr2),
                   .write_data(write_data2)
                 );

  always @(posedge clk_74a)
  begin
    if(write_en)
    begin
      rom[write_addr2] <= write_data2;
    end
  end

  wire Buzzer1, Buzzer2;
  wire oled_dc, oled_clk, oled_data;

  // atmega32u4 #(
  //              .USE_UART_1("FALSE"),
  //              .USE_EEPROM("FALSE")
  //            ) atmega32u4
  //            (
  //              .clk(clk_avr_16),
  //              .rst(~reset_n),
  //              .pgm_addr(pgm_addr),
  //              .pgm_data(pgm_data),
  //              .buttons({6{1'b1}}),
  //              .joystick_analog(8'd0),
  //              // Select ADC option (CONF_STR "OFG")? What is ADC?
  //              .status(2'h0),
  //              .Buzzer1(Buzzer1),
  //              .Buzzer2(Buzzer2),
  //              .DC(oled_dc),
  //              .spi_scl(oled_clk),
  //              .spi_mosi(oled_data)
  //            );

  wire ssd1306_ss, ssd1306_rst;

  atmega32u4_arduboy #(
                       .PLATFORM("XILINX"),
                       .RAM_TYPE("BLOCK"), // I don't think this does anything
                       .ROM_ADDR_WIDTH(14),
                       .BUS_ADDR_DATA_LEN(12),
                       .RAM_ADDR_WIDTH(12),
                       .USE_UART_1("FALSE"),
                       .USE_EEPROM("FALSE"),
                       .USE_TWI_1("FALSE")
                     ) atmega32u4
                     (
                       .clk(clk_avr_16),
                       .clk_pll(clk_avr_16),
                       .core_rst(~reset_n),
                       .dev_rst(~reset_n),

                       .pgm_addr(pgm_addr),
                       .pgm_data(pgm_data),
                       .buttons(buttons),
                       //  .joystick_analog(8'd0),
                       // Select ADC option (CONF_STR "OFG")? What is ADC?
                       //  .status(2'h0),
                       .Buzzer1(Buzzer1),
                       .Buzzer2(Buzzer2),

                       .nmi_sig(nmi_sig),

                       .OledDC(oled_dc),
                       .OledCS(ssd1306_ss),
                       .OledRST(ssd1306_rst),
                       .spi_scl(oled_clk),
                       .spi_mosi(oled_data),
                     );

endmodule
