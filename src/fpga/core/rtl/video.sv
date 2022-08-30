module video (
    input wire clk_oled,
    input wire clk_pixel_double,

    input wire reset_n,

    input wire oled_dc,
    input wire oled_data,

    output wire v_sync,
    output wire h_sync,
    output wire video_en,
    output reg video
  );

  reg [9:0] read_addr;
  reg [7:0] read_data;

  wire invert_video;

  oled_spi oled_spi (
             .clk_oled(clk_oled),
             .clk_read_mem(clk_pixel_double),

             .reset_n(reset_n),

             .oled_dc(oled_dc),
             .oled_data(oled_data),

             .read_addr(read_addr),
             .read_data(read_data),
             .invert_video(invert_video)
           );

  // 128 pixels, 288 with blanking
  reg [8:0] h_count;

  // 64 pixels, 83 with blanking
  reg [6:0] v_count;

  reg [1:0] clock_div = 3;

  reg invert_frame = 0;

  localparam [7:0] h_front_porch = 48;
  localparam [7:0] h_back_porch = 80;

  localparam [7:0] v_front_porch = 3;
  localparam [7:0] v_back_porch = 6;

  localparam [8:0] h_total = 288;
  localparam [7:0] v_total = 83;

  always @ (posedge clk_pixel_double)
  begin
    // Local vars
    reg [8:0] active_h;
    reg [6:0] active_v;

    active_h = h_count - h_front_porch;
    active_v = v_count - v_front_porch;

    clock_div <= clock_div + 1;

    v_sync <= 0;
    h_sync <= 0;
    video_en <= 0;

    if (h_count == 0 && v_count == 0)
    begin
      v_sync <= 1;
    end
    else if (h_count == 3)
    begin
      h_sync <= 1;
    end

    if (h_count >= h_front_porch && h_count < h_total - h_back_porch
        && v_count >= v_front_porch && v_count < v_total - v_back_porch)
    begin
      video_en <= 1;
    end

    if(clock_div == 3)
    begin
      h_count <= h_count + 1;

      if (h_count == h_total - 1)
      begin
        h_count <= 0;

        v_count <= v_count + 1;

        if (v_count == v_total - 1)
        begin
          v_count <= 0;
          invert_frame <= invert_video;
        end
      end

      // Read should have completed, write the pixel
      // Lower 3 bits determine which row contains the pixel
      video <= invert_frame ^ read_data[active_v[2:0]];
    end
    else
    begin
      // Screen is 128 pixels wide, so 7 bits of horizontal counter
      // Screen is 64 pixels tall, so 6 bits of vertical counter, but the data is loaded in as bytes,
      // so the bottom 3 bits are used to choose the row within the byte
      read_addr <= {active_v[5:3], active_h[6:0]};
    end
  end

endmodule
