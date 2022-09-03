// We pixel double our video output for scaling for the Pocket. Since it's a
// non-integer scale, we use half pixels on either side for pixels 0
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

  wire ssd_vid;

  ssd1306 # (
            .X_OLED_SIZE(128),
            .Y_OLED_SIZE(64),
            .X_PARENT_SIZE(128),
            .Y_PARENT_SIZE(64),
            .PIXEL_INACTIVE_COLOR(1'b0),
            .PIXEL_ACTIVE_COLOR(1'b1),
            .VRAM_BUFFERED_OUTPUT("TRUE"),
            .FULL_COLOR_OUTPUT("FALSE")
          ) ssd1306_inst (
            .rst_i(~oled_reset),
            .clk_i(clk_avr_16),

            .edge_color_i(1'b0),
            .raster_x_i(current_h / 6),
            .raster_y_i(current_v / 6),
            .raster_clk_i(clk_pixel),
            .raster_d_o(ssd_vid),

            .ss_i(ss),
            .scl_i(scl),
            .mosi_i(mosi),
            .dc_i(dc)
          );

  assign video =
         h_count > h_disabled + h_spacing &&
         v_count > v_disabled + v_spacing &&
         current_h < h_active - h_spacing &&
         current_v < v_active - v_spacing
         ? ssd_vid : 0;

  reg [10:0] h_count;

  reg [9:0] v_count;

  localparam [9:0] h_front_porch = 40;
  localparam [9:0] h_sync_length = 80;
  localparam [9:0] h_back_porch = 120 + h_sync_length; // 200

  localparam [9:0] v_front_porch = 3;
  localparam [9:0] v_sync_length = 10;
  localparam [9:0] v_back_porch = 15 + v_sync_length; // 25

  localparam [9:0] h_disabled = h_front_porch + h_back_porch; // 240
  localparam [9:0] v_disabled = v_front_porch + v_back_porch; // 28

  localparam [10:0] h_total = 1040;
  localparam [9:0] v_total = 748;

  localparam [9:0] h_active = 800;
  localparam [9:0] v_active = 720;

  localparam [9:0] h_active_oled = 768;
  localparam [9:0] v_active_oled = 384;

  localparam [9:0] h_spacing = (h_active - h_active_oled) / 2;
  localparam [9:0] v_spacing = (v_active - v_active_oled) / 2;

  wire [9:0] current_h;
  wire [9:0] current_v;

  assign current_h = h_count - h_disabled - h_spacing;
  assign current_v = v_count - v_disabled - v_spacing;

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

    if (h_count >= h_disabled && v_count >= v_disabled)
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
