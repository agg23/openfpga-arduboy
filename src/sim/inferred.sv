module inferred_rom (
    input wire clk_write,
    input wire clk_read,

    input wire write_en,
    input wire [13:0] write_addr,
    input wire [15:0] write_data,

    input wire [13:0] read_addr,
    output reg [15:0] read_data
  );

  reg  [1:0][7:0] rom[16384];

  always @ (posedge clk_read) read_data <= rom[read_addr];

  always @(posedge clk_write)
  begin
    if(write_en)
    begin
      rom[write_addr] <= write_data;
    end
  end
endmodule
