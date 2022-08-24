/*
 * This IPs is the RAM memory for the Atmel MEGA CPU implementation.
 *
 * Copyright (C) 2017  Iulian Gheorghiu  (morgoth@devboard.tech)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

`timescale 1ns / 1ps

 module mega_ram  #(
    parameter ADDR_BUS_WIDTH = 12,  /* < in address lines */
    parameter DATA_BUS_WIDTH = 8,
    parameter RAM_PATH = ""
) (
    input      rst,
    input      clk,
    input      we,
    input      [ADDR_BUS_WIDTH - 1:0] a,
    input      [DATA_BUS_WIDTH - 1:0] d_in,
    output reg [DATA_BUS_WIDTH - 1:0] d_out
);

reg [DATA_BUS_WIDTH - 1:0] mem [(2**ADDR_BUS_WIDTH)-1:0];

initial begin
if (RAM_PATH != "")
     $readmemh({RAM_PATH, ".mem"}, mem);
end

always @ (posedge clk) begin
    reg [7:0] clear_cnt = 0;

    if (rst) begin
        mem[clear_cnt] <= 0;
        clear_cnt <= clear_cnt + 1'd1;
    end
    else if (we) begin
        mem[a] <= d_in;
    end
end

always @ (posedge clk) d_out <= mem[a];

endmodule
