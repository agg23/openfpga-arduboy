// MIT License
//
// Copyright (c) 2022 Adam Gastineau
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

// A data loader for consuming APF bridge writes and directing them to some
// storage medium.
module data_loader #(
    parameter ADDRESS_MASK_UPPER_4 = 0,
    parameter ADDRESS_SIZE = 28,
    parameter WRITE_MEM_CLOCK_DELAY = 4,
    parameter WRITE_MEM_EN_CYCLE_LENGTH = 1,
    parameter OUTPUT_WORD_SIZE = 1
) (
    input wire clk_74a,
    input wire clk_memory,

    input wire bridge_wr,
    input wire bridge_endian_little,
    input wire [31:0] bridge_addr,
    input wire [31:0] bridge_wr_data,

    output reg write_en = 0,
    output reg [ADDRESS_SIZE-1:0] write_addr = 0,
    output reg [8 * OUTPUT_WORD_SIZE - 1:0] write_data = 0
);

  `define MAX(x, y) ((x > y) ? x : y)

  localparam WORD_SIZE = 8 * OUTPUT_WORD_SIZE;
  localparam FIFO_SIZE = WORD_SIZE + 28;

  wire mem_empty;
  wire [FIFO_SIZE - 1:0] fifo_out;

  reg read_req = 0;
  reg write_req = 0;
  reg [31:0] shift_data;
  reg [27:0] buff_bridge_addr;

  wire [FIFO_SIZE - 1:0] fifo_in = {shift_data[WORD_SIZE-1:0], buff_bridge_addr[27:0]};

  dcfifo dcfifo_component (
      .data(fifo_in),
      .rdclk(clk_memory),
      .rdreq(read_req),
      .wrclk(clk_74a),
      .wrreq(write_req),
      .q(fifo_out),
      .rdempty(mem_empty)
  );
  defparam dcfifo_component.clocks_are_synchronized = "FALSE",
      dcfifo_component.intended_device_family = "Cyclone V",
      dcfifo_component.lpm_numwords = 4,
      dcfifo_component.lpm_showahead = "OFF",
      dcfifo_component.lpm_type = "dcfifo",
      dcfifo_component.lpm_width = FIFO_SIZE,
      dcfifo_component.lpm_widthu = 2,
      dcfifo_component.overflow_checking = "OFF",
      dcfifo_component.rdsync_delaypipe = 5,
      dcfifo_component.underflow_checking = "OFF",
      dcfifo_component.use_eab = "OFF",
      dcfifo_component.wrsync_delaypipe = 5;

  reg prev_bridge_wr = 0;
  reg [2:0] write_count = 0;
  reg [2:0] write_state = 0;

  localparam WRITE_START = 1;
  localparam WRITE_REQ_SHIFT = 2;

  always @(posedge clk_74a) begin
    prev_bridge_wr <= bridge_wr;

    if (~prev_bridge_wr && bridge_wr && bridge_addr[31:28] == ADDRESS_MASK_UPPER_4) begin
      write_state <= WRITE_REQ_SHIFT;
      write_req <= 1;
      write_count <= 0;

      shift_data <= bridge_endian_little ? bridge_wr_data : {
        bridge_wr_data[7:0],
        bridge_wr_data[15:8],
        bridge_wr_data[23:16],
        bridge_wr_data[31:24]
      };
      buff_bridge_addr <= bridge_addr[27:0];
    end

    case (write_state)
      WRITE_START: begin
        write_req <= 1;
        write_state <= WRITE_REQ_SHIFT;
      end
      WRITE_REQ_SHIFT: begin
        write_req <= 0;
        shift_data <= {8'h0, shift_data[31:WORD_SIZE]};
        buff_bridge_addr <= buff_bridge_addr + OUTPUT_WORD_SIZE;
        write_count <= write_count + 1;

        if (write_count == (4 / OUTPUT_WORD_SIZE) - 1) begin
          write_state <= 0;
        end else begin
          write_state <= WRITE_START;
        end
      end
    endcase
  end

  reg [5:0] read_state = 0;

  localparam READ_DELAY = 1;
  localparam READ_WRITE = 2;
  localparam READ_WRITE_EN_CYCLE_OFF = READ_WRITE + WRITE_MEM_EN_CYCLE_LENGTH;
  localparam READ_WRITE_END_DEFAULT = WRITE_MEM_CLOCK_DELAY - 1;
  localparam READ_WRITE_END = `MAX(READ_WRITE_END_DEFAULT, READ_WRITE_EN_CYCLE_OFF + 1);
  localparam HAS_DELAY = READ_WRITE_END_DEFAULT > READ_WRITE_EN_CYCLE_OFF;

  always @(posedge clk_memory) begin
    if (read_state != 0) begin
      read_state <= read_state + 1;
    end else if (~mem_empty) begin
      read_state <= READ_DELAY;
      read_req <= 1;
    end

    case (read_state)
      READ_DELAY: begin
        read_req <= 0;
        write_en <= 0;
      end
      READ_WRITE: begin
        write_en <= 1;
        write_addr <= fifo_out[27:0];
        write_data <= fifo_out[WORD_SIZE+27:28];
        read_req <= 0;
      end
      READ_WRITE_EN_CYCLE_OFF: begin
        write_en <= 0;

        if (!HAS_DELAY) begin
          read_state <= 0;
        end
      end
      READ_WRITE_END: begin
        read_state <= 0;
      end
    endcase
  end

  initial begin
    if (WRITE_MEM_CLOCK_DELAY < 4) begin
      $error("WRITE_MEM_CLOCK_DELAY has a minimum value of 4. Received %d", WRITE_MEM_CLOCK_DELAY);
    end

    if (WRITE_MEM_EN_CYCLE_LENGTH < 1 || WRITE_MEM_EN_CYCLE_LENGTH >= WRITE_MEM_CLOCK_DELAY - 2) begin
      $error(
          "WRITE_MEM_EN_CYCLE_LENGTH must be between 1 and %d (inclusive, based off of WRITE_MEM_CLOCK_DELAY). Received %d",
          WRITE_MEM_CLOCK_DELAY - 2 - 1, WRITE_MEM_EN_CYCLE_LENGTH);
    end

    if (OUTPUT_WORD_SIZE < 1 || OUTPUT_WORD_SIZE > 2) begin
      $error("OUTPUT_WORD_SIZE must be 1 or 2. Received %d", OUTPUT_WORD_SIZE);
    end
  end

endmodule
