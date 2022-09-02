// We pixel double our video output for scaling for the Pocket. Since it's a
// non-integer scale, we use half pixels on either side for pixels 0 and 127
module video (
    input wire clk_avr_16,
    input wire clk_pixel,

    input wire oled_reset,
    input wire ss,
    input wire scl,
    input wire mosi,
    input wire dc,

    output wire v_sync,
    output wire h_sync,
    output wire video_en,
    output wire video
  );

  ssd1306 # (
            .X_OLED_SIZE(128),
            .Y_OLED_SIZE(64),
            .X_PARENT_SIZE(256),
            .Y_PARENT_SIZE(128),
            .PIXEL_INACTIVE_COLOR(1'b0),
            .PIXEL_ACTIVE_COLOR(1'b1),
            .INACTIVE_DISPLAY_COLOR(32'h10101010),
            .VRAM_BUFFERED_OUTPUT("TRUE"),
            .FULL_COLOR_OUTPUT("FALSE")
          ) ssd1306_inst (
            .rst_i(~oled_reset),
            .clk_i(clk_avr_16),

            .edge_color_i(1'b0),
            .raster_x_i(active_h + 1),
            .raster_y_i(active_v),
            .raster_clk_i(clk_pixel),
            .raster_d_o(video),

            .ss_i(ss),
            .scl_i(scl),
            .mosi_i(mosi),
            .dc_i(dc)
          );

  // 128*2 pixels, 416 with blanking
  reg [8:0] h_count;

  // 64*2 pixels, 147 with blanking
  reg [7:0] v_count;

  localparam [7:0] h_front_porch = 48;
  localparam [7:0] h_sync_length = 32;
  localparam [7:0] h_back_porch = 80 + h_sync_length; // 112

  localparam [7:0] v_front_porch = 3;
  localparam [7:0] v_sync_length = 10;
  localparam [7:0] v_back_porch = 6 + v_sync_length; // 16

  localparam [7:0] h_disabled = h_front_porch + h_back_porch; // 160
  localparam [7:0] v_disabled = v_front_porch + v_back_porch; // 19

  localparam [8:0] h_total = 416;
  localparam [7:0] v_total = 147;

  wire [8:0] active_h;
  wire [6:0] active_v;

  // +1 is a hack to handle the half pixel in scaling
  assign active_h = h_count - h_disabled;
  assign active_v = v_count - v_disabled;

  always @ (posedge clk_pixel)
  begin
    v_sync <= 0;
    h_sync <= 0;
    video_en <= 0;

    if (h_count == 0 && v_count == v_front_porch)
    begin
      v_sync <= 1;
    end
    else if (h_count == h_front_porch)
    begin
      h_sync <= 1;
    end

    // -2 is a hack to handle the half pixel in scaling
    if (h_count >= h_disabled - 2 && v_count >= v_disabled)
    begin
      video_en <= 1;
    end

    h_count <= h_count + 1;

    if (h_count == h_total - 1)
    begin
      h_count <= 0;

      v_count <= v_count + 1;

      if (v_count == v_total - 1)
      begin
        v_count <= 0;
      end
    end
  end

endmodule
