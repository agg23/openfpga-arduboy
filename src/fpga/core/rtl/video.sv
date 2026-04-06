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

    output logic v_sync,
    output logic h_sync,
    output logic video_en,
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

  logic [10:0] h_count = 11'd0;
  logic [9:0] v_count = 10'd0;
  logic [10:0] h_count_d = 11'd0;
  logic [9:0] v_count_d = 10'd0;

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

  localparam [10:0] h_active_start = h_disabled;
  localparam [10:0] h_active_end = h_disabled + h_active;
  localparam [9:0] v_active_start = v_disabled;
  localparam [9:0] v_active_end = v_disabled + v_active;

  localparam [10:0] h_oled_start = h_disabled + h_spacing;
  localparam [10:0] h_oled_end = h_oled_start + h_active_oled;
  localparam [9:0] v_oled_start = v_disabled + v_spacing;
  localparam [9:0] v_oled_end = v_oled_start + v_active_oled;

  wire [10:0] current_h;
  wire [9:0] current_v;
  wire oled_window_d;

  assign current_h = h_count - h_oled_start;
  assign current_v = v_count - v_oled_start;
  assign oled_window_d =
         h_count_d >= h_oled_start &&
         h_count_d < h_oled_end &&
         v_count_d >= v_oled_start &&
         v_count_d < v_oled_end;
  assign video = oled_window_d ? ssd_vid : 1'b0;

  always @ (posedge clk_pixel)
  begin
    h_count_d <= h_count;
    v_count_d <= v_count;

    v_sync <= h_count == 0 && v_count == v_front_porch;
    h_sync <= h_count == h_front_porch;
    video_en <=
      h_count >= h_active_start &&
      h_count < h_active_end &&
      v_count >= v_active_start &&
      v_count < v_active_end;

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
