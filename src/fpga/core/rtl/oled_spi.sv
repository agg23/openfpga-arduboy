// Module for processing the Arduboy's SSD1306 OLED SPI protocol.
// https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
module oled_spi
  (
    input wire clk_oled,
    input wire clk_read_mem,

    input wire reset_n,

    input wire oled_dc, // Data or command toggle
    input wire oled_data,

    input wire [9:0] read_addr,
    output reg [7:0] read_data,
    output reg invert_video
  );

  // We will load in data one byte at a time
  reg [7:0] mem [128 * 64 / 8];

  reg [9:0] write_addr;
  reg [7:0] shift_reg;
  reg [2:0] shift_count;

  wire [7:0] shift_result = {shift_reg[6:0], oled_data};

  // Features
  reg [2:0] start_page;

  always @ (posedge clk_oled or negedge reset_n)
  begin
    if(~reset_n)
    begin
      shift_reg <= 0;
      shift_count <= 0;

      invert_video <= 0;
      start_page <= 0;
    end
    else
    begin
      if(shift_count == 7)
      begin
        if(oled_dc)
        begin
          // Data received
          // Write data
          mem[write_addr] <= shift_result;

          write_addr <= write_addr + 1;
        end
        else
        begin
          // Command received
          case (shift_result)
            8'h22: // Set page address
            begin
              // https://github.com/akkera102/08_gamebuino
              // TODO: This doesn't seem quite right
              write_addr <= {start_page, 7'b0};

              start_page <= start_page + 1;

              if(start_page == 5)
              begin
                start_page <= 0;
              end
            end
            8'hA6, 8'hA7: // Control display inversion
              invert_video <= shift_result[0];
          endcase

          // Quartus is stupid, and doesn't support `case inside`, which would allow for this range inside the case
          if (shift_result >= 8'hB0 && shift_result <= 8'hB7)
          begin
            // Page start address 0-7
            // Each page has 8 rows of data
            write_addr <= {shift_result[2:0], 7'b0};
          end
        end
      end

      shift_reg <= shift_result;
      shift_count <= shift_count + 1;
    end
  end

  wire [9:0] read_addr_s;
  synch_2 #(.WIDTH(10)) mem_s (read_addr, read_addr_s, clk_read_mem);

  always @ (posedge clk_read_mem)
  begin
    read_data <= mem[read_addr];
  end

endmodule
