/**
Descripcion,
Modulo que sincroniza las senales (hsync y vsync)
de un controlador VGA de 640x480 60hz, funciona con un reloj de 25Mhz

Ademas tiene las coordenadas de los pixeles H (eje x),
y de los pixeles V (eje y). Para enviar la senal RGB correspondiente
a cada pixel

-----------------------------------------------------------------------------
Author : Nicolas Hasbun, nhasbun@gmail.com
File   : vgaHdmi.v
Create : 2017-06-15 15:07:05
Editor : sublime text3, tab size (2)
-----------------------------------------------------------------------------
*/

// **Info Source**
// https://eewiki.net/pages/viewpage.action?pageId=15925278

module vgaHdmi
(
  input clock,
  input reset,
  input oled_dc,
  input oled_clk,
  input oled_data,

  output reg ce_pix,
  output reg hsync, vsync,
  output reg hblank, vblank,
  output     pixelValue
);

reg  [7:0] mem [1024];
reg  [9:0] waddr, raddr;

reg        invert;
reg  [2:0] shiftCount;
reg  [7:0] shiftReg;
wire [7:0] shiftLeft = {shiftReg[6:0], oled_data};
reg  [2:0] pageCount;

always @ (posedge oled_clk or posedge reset) begin
  if(reset) begin
    waddr         <= 0;
    invert        <= 0;
    shiftCount    <= 0;
    shiftReg      <= 0;
    pageCount     <= 0;
  end
  else begin
    if (oled_dc) begin // data
      if (shiftCount == 3'b111) begin
        mem[waddr] <= shiftLeft;
        waddr <= waddr + 1'b1; // Increment address
      end
      else shiftReg <= shiftLeft;
      shiftCount <= shiftCount + 1'b1;
    end
    else begin // commands
      if (shiftCount == 3'b111) begin
        if(shiftLeft[7:1] == 'b1010011) invert <= shiftLeft[0]; // invert (A6/A7)
        if(shiftLeft[7:3] == 'b10110) waddr <= {shiftLeft[2:0], 7'b000_0000}; // page 0-7 (B0-B7)
        if(shiftLeft == 8'h22) begin // for games using https://github.com/akkera102/08_gamebuino
          waddr <= {pageCount, 7'b000_0000};
          if (pageCount == 3'b101) pageCount <= 3'b000;
          else pageCount <= pageCount + 1'b1;
        end
      end
      else shiftReg <= shiftLeft;
      shiftCount <= shiftCount + 1'b1;
    end
  end
end

wire [7:0] tempByte = mem[raddr];

reg ce_pix_pre, ce_pix_int;
always @ (posedge clock) begin
    reg [3:0] div;

    div <= div + 1'd1;

    ce_pix_pre <= !div;
    ce_pix_int <= !div[2:0] & div[3];
    ce_pix     <= !div[2:0]; // twice higher pixel clock to fit OSD on VGA/DV
end

assign pixelValue = pixel & ~vblank_int;
reg pixel, vblank_int;

// Manejo de Pixeles y Sincronizacion
always @ (posedge clock) begin
    reg       invertLatched;
    reg       old_ce;
    reg       vdiv;
    reg [7:0] pixelH, pixelV;

    old_ce <= ce_pix_pre;

    if (ce_pix_pre) begin
        pixelH <= pixelH + 1'b1;
        if(pixelH == 159) begin
            pixelH <= 0;
            vdiv <= ~vdiv;
            if(vdiv) begin
                pixelV <= pixelV + 1'd1;
                if(pixelV == 130) begin
                    invertLatched <= invert;
                    pixelV <= 0;
                end
            end
        end
    end

    if (old_ce) raddr <= {pixelV[5:3],pixelH[6:0]};

    if (ce_pix_int) begin
        pixel <= invertLatched ^ tempByte[pixelV[2:0]];

        if(pixelV == 127) vblank     <= 0;
        if(pixelV == 0)   vblank_int <= 0;
        if(pixelV == 64)  vblank_int <= 1;
        if(pixelV == 68)  vblank     <= 1;

        vsync  <= (pixelV[7:1] == 47);

        if(pixelH == 0)   hblank <= 0;
        if(pixelH == 128) hblank <= 1;
        if(pixelH == 135) hsync  <= 1;
        if(pixelH == 150) hsync  <= 0;
    end
end

endmodule
