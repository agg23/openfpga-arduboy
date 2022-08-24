/*
 * This IP is the ATMEGA PIO implementation.
 *
 * Copyright (C) 2020  Iulian Gheorghiu (morgoth@devboard.tech)
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

module atmega_pio # (
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter PORT_WIDTH = 8,
    parameter USE_CLEAR_SET = "FALSE",
    parameter PORT_OUT_ADDR = 'h20,
    parameter PORT_CLEAR_ADDR = 'h00,
    parameter PORT_SET_ADDR = 'h01,
    parameter DDR_ADDR = 'h23,
    parameter PIN_ADDR = 'h24,
    parameter PINMASK = 8'hFF,
    parameter PULLUP_MASK = 8'h0,
    parameter PULLDN_MASK = 8'h0,
    parameter INVERSE_MASK = 8'h0,
    parameter OUT_ENABLED_MASK = 8'hFF
)(
    input rst,
    input clk,

    input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
    input wr_dat,
    input rd_dat,
    input [PORT_WIDTH - 1:0]bus_dat_in,
    output reg [PORT_WIDTH - 1:0]bus_dat_out,

    input [PORT_WIDTH - 1:0]io_in,
    output [PORT_WIDTH - 1:0]io_out
    );

reg [7:0]DDR;
reg [7:0]PORT;

assign io_out[0] = DDR[0] ? PORT[0] : 1'b0;
assign io_out[1] = DDR[1] ? PORT[1] : 1'b0;
assign io_out[2] = DDR[2] ? PORT[2] : 1'b0;
assign io_out[3] = DDR[3] ? PORT[3] : 1'b0;
assign io_out[4] = DDR[4] ? PORT[4] : 1'b0;
assign io_out[5] = DDR[5] ? PORT[5] : 1'b0;
assign io_out[6] = DDR[6] ? PORT[6] : 1'b0;
assign io_out[7] = DDR[7] ? PORT[7] : 1'b0;

always @ (posedge rst or posedge clk)
begin
    if(rst)
    begin
        DDR <= 8'h00;
        PORT <= 8'h00;
    end
    else if(wr_dat)
    begin
        case(addr_dat)
        DDR_ADDR: DDR <= bus_dat_in;
        PORT_OUT_ADDR: PORT <= bus_dat_in;
        endcase
    end
end

always @ *
begin
    bus_dat_out = 8'h00;
    if(rd_dat & ~rst)
    begin
        case(addr_dat)
        PORT_OUT_ADDR: bus_dat_out = PORT;
        DDR_ADDR: bus_dat_out = DDR;
        PIN_ADDR: bus_dat_out = io_in;
        default: bus_dat_out = 8'h00;
        endcase
    end
end

endmodule
