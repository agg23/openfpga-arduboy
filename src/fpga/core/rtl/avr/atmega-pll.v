/*
 * This IP is the ATMEGA PLL implementation.
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

module atmega_pll # (
	parameter BUS_ADDR_DATA_LEN = 16,
	parameter PLLCSR_ADDR = 'h49,
	parameter PLLFRQ_ADDR = 'h52,
	parameter USE_PLL = "TRUE"// If "FALSE" tim_ck_out = clk
)(
	input rst,
	input clk,
	input clk_pll, // 192Mhz input.

	input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
	input wr_dat,
	input rd_dat,
	input [7:0]bus_dat_in,
	output reg [7:0]bus_dat_out,
	
	output pll_enabled,

	output usb_ck_out,
	output tim_ck_out
	);

reg [7:0]PLLCSR;
reg [7:0]PLLFRQ;
//reg []prescaller;

reg [4:0]fractional_value;
reg [4:0]fractional_cnt;
reg [3:0]prescaller_value;
reg [3:0]prescaller_cnt;
reg pll_clk_out_int;
reg pll_clk_del_int;

reg [1:0]tim_div_value;
reg [1:0]tim_div_cnt;
reg tim_clk_1_5;
reg tim_clk_2;
reg usb_clk_2_int;

always @ (posedge clk)
begin
	if(rst)
	begin
		PLLCSR <= 8'h00;
		PLLFRQ <= 8'h00;
	end
	else
	begin
		PLLCSR[0] <= PLLCSR[1]; // Make it ready
		if(wr_dat)
		begin
			case(addr_dat)
			PLLCSR_ADDR: PLLCSR <= bus_dat_in;
			PLLFRQ_ADDR: PLLFRQ <= bus_dat_in;
			endcase
		end
	end
end
// PLL Prescaller
always @ *
begin
	if(USE_PLL == "TRUE")
	begin
		case(PLLFRQ[3:0])
			4'b0011: // 40Mhz
			begin
				prescaller_value = 2;
				fractional_value = 5;
			end
			4'b0100: // 48Mhz
			begin
				prescaller_value = 2;
				fractional_value = 0;
			end
			4'b0101: // 56Mhz // Unable, output 64 Mhz instead
			begin
				prescaller_value = 1;
				fractional_value = 2;
			end
	
			4'b0111: // 72Mhz
			begin
				prescaller_value = 1;
				fractional_value = 3;
			end
			4'b1000: // 80Mhz
			begin
				prescaller_value = 1;
				fractional_value = 5;
			end
			4'b1001: // 88Mhz
			begin
				prescaller_value = 1;
				fractional_value = 11;
			end
			4'b1010: // 96Mhz
			begin
				prescaller_value = 0;
				fractional_value = 0;
			end
		endcase
	end
end

// TIM Prescaller
always @ *
begin
	if(USE_PLL == "TRUE")
	begin
		tim_div_value = 2'd0;
		case(PLLFRQ[5:4])
			2'b10: tim_div_value = 2'd2;
			2'b11: tim_div_value = 2'd3;
		endcase
	end
end

always @ (posedge rst or posedge clk_pll)
begin
	if(USE_PLL == "TRUE")
	begin
		if(rst)
		begin
			fractional_cnt <= 0;
			prescaller_cnt <= 0;
			pll_clk_out_int <= 0;
			pll_clk_del_int <= 0;
			tim_div_cnt <= 0;
			usb_clk_2_int <= 0;
		end
		else
		begin
			if(fractional_cnt || fractional_value == 0)
			begin
				fractional_cnt <= fractional_cnt -1;
				if(prescaller_cnt & prescaller_value != 0)
				begin
					prescaller_cnt <= prescaller_cnt - 1;
				end
				else
				begin
					prescaller_cnt <= prescaller_value - 1;
					pll_clk_out_int <= ~pll_clk_out_int;
				end
			end
			else
			begin
				fractional_cnt <= fractional_value;
			end
			//PLLTIM & PLLUSB
			pll_clk_del_int <= pll_clk_out_int;
			if(pll_clk_del_int ^ pll_clk_out_int)
			begin
				usb_clk_2_int <= ~usb_clk_2_int;
				if(tim_div_cnt)
					tim_div_cnt <= tim_div_cnt - 1;
				else
					tim_div_cnt <= tim_div_value;
			end
		end
	end
end

always @ (posedge clk)
begin
	if(rst)
		tim_clk_2 <= 1'b0;
	else
		tim_clk_2 <= ~tim_clk_2;
end

always @ *
begin
	bus_dat_out = 8'h00;
	if(rd_dat & ~rst)
	begin
		case(addr_dat)
		PLLCSR_ADDR: bus_dat_out = PLLCSR;
		PLLFRQ_ADDR: bus_dat_out = PLLFRQ;
		endcase
	end
end

assign usb_ck_out = USE_PLL != "TRUE" ? 1'b0 : (PLLFRQ[6] ? usb_clk_2_int : pll_clk_out_int);
assign tim_ck_out = USE_PLL != "TRUE" ? (PLLCSR[4] ? tim_clk_2 : clk) : (PLLFRQ[5:4] == 2'b00 ? (PLLCSR[4] ? tim_clk_2 : clk) : (PLLFRQ[5:4] == 2'b01 ? pll_clk_out_int : (PLLFRQ[5:4] == 2'b11 ? tim_div_cnt[1] : tim_div_cnt[0])));
assign pll_enabled = USE_PLL != "TRUE" ? 1'b0 : (|PLLFRQ[5:4]);
endmodule
