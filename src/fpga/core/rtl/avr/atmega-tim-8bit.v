/*
 * This IP is the ATMEGA 8bit TIMER implementation.
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

//`define TIFR0 ('h15)
`define TOV0 0
`define OCF0A 1
`define OCF0B 2

//`define GTCCR ('h23)
`define PSRSYNC 0
`define PSRASY  1
`define TSM     7

//`define TCCR0A ('h24)
`define WGM00   0
`define WGM01   1
`define COM0B0  4
`define COM0B1  5
`define COM0A0  6
`define COM0A1  7

//`define TCCR0B ('h25)
`define CS00    0
`define CS01    1
`define CS02    2
`define WGM02   3
`define FOC0B   6
`define FOC0A   7

//`define TCNT0 ('h26)
`define OCR0A ('h27)
`define OCR0B ('h28)

//`define TIMSK0 ('h6E)
`undef TOIE0
`undef OCIE0A
`undef OCIE0B
`define TOIE0   0
`define OCIE0A  1
`define OCIE0B  2

module atmega_tim_8bit # (
    parameter USE_OCRB = "TRUE",
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter GTCCR_ADDR = 'h43,
    parameter TCCRA_ADDR = 'h44,
    parameter TCCRB_ADDR = 'h45,
    parameter TCNT_ADDR = 'h46,
    parameter OCRA_ADDR = 'h47,
    parameter OCRB_ADDR = 'h48,
    parameter TIMSK_ADDR = 'h6E,
    parameter TIFR_ADDR = 'h35
)(
    input rst,
    input halt,
    input clk,
    input clk8,
    input clk64,
    input clk256,
    input clk1024,
    input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
    input wr_dat,
    input rd_dat,
    input [7:0]bus_dat_in,
    output reg [7:0]bus_dat_out,
    output tov_int,
    input tov_int_rst,
    output ocra_int,
    input ocra_int_rst,
    output ocrb_int,
    input ocrb_int_rst,
    output reg oca,
    output reg ocb,
    output oca_io_connect,
    output ocb_io_connect
    );

reg [7:0]GTCCR;
reg [7:0]TCCRA;
reg [7:0]TCCRB;
reg [7:0]TCNT;
reg [7:0]OCRA;
reg [7:0]OCRB;
reg [7:0]OCRA_int;
reg [7:0]OCRB_int;
reg [7:0]TIMSK;
reg [7:0]TIFR;

reg tov_p;
reg tov_n;
reg ocra_p;
reg ocra_n;
reg ocrb_p;
reg ocrb_n;

//reg l1;
//reg l2;
wire t0_fall = 0;
wire t0_rising = 0;
wire clk_int;
reg clk_int_del;

reg up_count;

wire clk_active = |TCCRB[`CS02:`CS00];

/* Sampling implementation */
// Accourding to timer sampling module.
/*always @ *
begin
    if(rst)
    begin
        l1 = 1'b0;
    end
    else
    if(clk)
    begin
        l1 = t;
    end
end

always @ (posedge clk)
begin
    if(rst)
    begin
        l2 = 1'b0;
    end
    else
    if(clk)
    begin
        l2 = l1;
    end
end*/
/* !Sampling implementation */

/* Prescaller selection implementation */
wire [7:0]clk_mux = {t0_rising, t0_fall, clk1024, clk256, clk64, clk8, 2'b00};
lpm_mux LPM_MUX_component (
            .data(clk_mux),
            .sel(TCCRB[`CS02:`CS00]),
            .result(clk_int),
            .aclr(),
            .clken(),
            .clock()
            );
defparam
    LPM_MUX_component.lpm_size = 8,
    LPM_MUX_component.lpm_type = "LPM_MUX",
    LPM_MUX_component.lpm_width = 1,
    LPM_MUX_component.lpm_widths = 3;

reg updt_ocr_on_top;
always @ *
begin
    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
        3'h0, 3'h2: updt_ocr_on_top = 1'b0;
        default: updt_ocr_on_top = 1'b1;
    endcase
end

reg [7:0]top_value;
always @ *
begin
    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
        3'h2, 3'h5, 3'h7: top_value = OCRA_int;
        default: top_value = 8'hff;
    endcase
end

reg [7:0]t_ovf_value;
always @ *
begin
    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
        3'd7: t_ovf_value = top_value;
        3'd0, 3'd2, 3'd3: t_ovf_value = 8'hFF;
        default: t_ovf_value = 8'h00;
    endcase
end

// Read registers.
always @ *
begin
    if(rst)
    begin
        bus_dat_out = 8'h00;
    end
    else
    begin
        bus_dat_out = 8'h00;
        if(rd_dat)
        begin
            case(addr_dat)
                TCCRA_ADDR: bus_dat_out = TCCRA;
                TCCRB_ADDR: bus_dat_out = TCCRB;
                TCNT_ADDR: bus_dat_out = TCNT;
                OCRA_ADDR: bus_dat_out = OCRA;
                OCRB_ADDR: bus_dat_out = OCRB;
                TIFR_ADDR: bus_dat_out = TIFR;
            endcase
        end
        if(rd_dat & addr_dat == TIMSK_ADDR)
            bus_dat_out = TIMSK;
    end
end

/* Set "oc" pin on specified conditions*/
always @ (posedge clk)
begin
    if(rst)
    begin
        GTCCR <= 8'h00;
        TCCRA <= 8'h00;
        TCCRB <= 8'h00;
        TCNT <= 8'h00;
        OCRA <= 8'h00;
        OCRB <= 8'h00;
        OCRA_int <= 8'h00;
        OCRB_int <= 8'h00;
        TIMSK <= 8'h00;
        TIFR <= 8'h00;
        tov_p <= 1'b0;
        tov_n <= 1'b0;
        ocra_p <= 1'b0;
        ocra_n <= 1'b0;
        ocrb_p <= 1'b0;
        ocrb_n <= 1'b0;
        oca <= 1'b0;
        ocb <= 1'b0;
        up_count <= 1'b1;
        clk_int_del <= 1'b0;
    end
    else
    begin
        if(tov_p ^ tov_n)
        begin
            TIFR[`TOV0] <= 1'b1;
            tov_n <= tov_p;
        end
        if(ocra_p ^ ocra_n)
        begin
            TIFR[`OCF0A] <= 1'b1;
            ocra_n <= ocra_p;
        end
        if(ocrb_p ^ ocrb_n)
        begin
            TIFR[`OCF0B] <= 1'b1;
            ocrb_n <= ocrb_p;
        end
        if(tov_int_rst)
        begin
            TIFR[`TOV0] <= 1'b0;
        end
        if(ocra_int_rst)
        begin
            TIFR[`OCF0A] <= 1'b0;
        end
        if(ocrb_int_rst)
        begin
            TIFR[`OCF0B] <= 1'b0;
        end
        // Sample one IO core clock once every prescaller positive edge clock.
        clk_int_del <= clk_int; // Shift prescaller clock to a delay register every IO core positive edge clock to detect prescaller positive edges.
        if(((~clk_int_del & clk_int) || TCCRB[`CS02:`CS00] == 3'b001) && TCCRB[`CS02:`CS00] != 3'b000) // if prescaller clock = IO core clock disable prescaller positive edge detector.
        begin
            if(up_count & ~halt)
                TCNT <= TCNT + 8'd1;
            else
            if(~up_count & ~halt)
                TCNT <= TCNT - 8'd1;
            // OCRA
            if(updt_ocr_on_top ? (TCNT == 8'hff):(TCNT == OCRA_int))
                OCRA_int <= OCRA;
            if(TCNT == OCRA_int)
            begin
                case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                    3'h2: oca <= ~oca;
                    default:
                    begin
                        case(OCRA_int)
                            8'h00:  oca <= 1'b0;
                            8'hFF:  oca <= 1'b1;
                            default:
                            begin
                                if(up_count)
                                begin
                                    case(TCCRA[`COM0A1:`COM0A0])
                                        2'h1: oca <= ~oca;
                                        2'h2: oca <= 1'b0;
                                        2'h3: oca <= 1'b1;
                                    endcase
                                end
                                else
                                begin
                                    case(TCCRA[`COM0A1:`COM0A0])
                                        2'h1: oca <= ~oca;
                                        2'h2: oca <= 1'b1;
                                        2'h3: oca <= 1'b0;
                                    endcase
                                end
                            end
                        endcase
                    end
                endcase
                if(TIMSK[`OCIE0A] == 1'b1)
                begin
                    if(ocra_p == ocra_n && clk_active == 1'b1)
                        ocra_p <= ~ocra_p;
                    else
                    begin
                        ocra_p <= 1'b0;
                        ocra_n <= 1'b0;
                    end
                end
            end
            // !OCRA
            if(USE_OCRB == "TRUE")
            begin
                // OCRB
                if(updt_ocr_on_top ? (TCNT == 8'hff):(TCNT == OCRB_int))
                    OCRB_int <= OCRB;
                if(TCNT == OCRB_int)
                begin
                    case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                        3'h2: ocb <= ~ocb;
                        default:
                        begin
                            case(OCRB_int)
                                8'h00:  ocb <= 1'b0;
                                8'hFF:  ocb <= 1'b1;
                                default:
                                begin
                                    if(up_count)
                                    begin
                                        case(TCCRA[`COM0B1:`COM0B0])
                                            2'h1: ocb <= ~ocb;
                                            2'h2: ocb <= 1'b0;
                                            2'h3: ocb <= 1'b1;
                                        endcase
                                    end
                                    else
                                    begin
                                        case(TCCRA[`COM0B1:`COM0B0])
                                            2'h1: ocb <= ~ocb;
                                            2'h2: ocb <= 1'b1;
                                            2'h3: ocb <= 1'b0;
                                        endcase
                                    end
                                end
                            endcase
                        end
                    endcase
                    if(TIMSK[`OCIE0B] == 1'b1)
                    begin
                        if(ocrb_p == ocrb_n && clk_active == 1'b1)
                            ocrb_p <= ~ocrb_p;
                    end
                    else
                    begin
                        ocrb_p <= 1'b0;
                        ocrb_n <= 1'b0;
                    end
                end
            end // USE_OCRB != "TRUE"
            // TCNT overflow logick.
            if(&{TCNT == t_ovf_value, ~halt})
            begin
                if(TIMSK[`TOIE0] == 1'b1)
                begin
                    if(tov_p == tov_n && clk_active == 1'b1)
                        tov_p <= ~tov_p;
                end
                else
                begin
                    tov_p <= 1'b0;
                    tov_n <= 1'b0;
                end
            end
            if(&{TCNT == top_value, ~halt})
            begin
                case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                    3'h1, 3'h5:
                    begin
                        up_count <= 1'b0;
                        TCNT <= TCNT - 8'd1;
                    end
                    default: TCNT <= 8'h00;
                endcase
            end
            else if(&{TCNT == 8'h00, ~halt})
            begin
                case({TCCRB[`WGM02], TCCRA[`WGM01:`WGM00]})
                    3'h1, 3'h5:
                    begin
                        up_count <= 1'b1;
                        TCNT <= TCNT + 8'd1;
                    end
                endcase
            end
        end
        // Write registers
        if(wr_dat)
        begin
            case(addr_dat)
                GTCCR_ADDR: GTCCR <= bus_dat_in;
                TCCRA_ADDR: TCCRA <= bus_dat_in;
                TCCRB_ADDR: TCCRB <= bus_dat_in;
                TCNT_ADDR: TCNT <= bus_dat_in;
                OCRA_ADDR: OCRA <= bus_dat_in;
                OCRB_ADDR: OCRB <= bus_dat_in;
                TIFR_ADDR: TIFR <= TIFR & ~bus_dat_in;
            endcase
        end
        if(wr_dat & addr_dat == TIMSK_ADDR)
            TIMSK <= bus_dat_in;
    end
end

assign tov_int = TIFR[`TOV0];
assign ocra_int = TIFR[`OCF0A];
assign ocrb_int = TIFR[`OCF0B];

assign oca_io_connect = (TCCRA[`COM0A1:`COM0A0] == 2'b00) ? 1'b0 : (TCCRA[`COM0A1:`COM0A0] == 2'b01 ? ((TCCRA[`WGM01:`WGM00] == 2'd1 || TCCRA[`WGM01:`WGM00] == 2'd3) ? TCCRB[`WGM02] : 1'b1) : 1'b1);
assign ocb_io_connect = USE_OCRB == "TRUE" ? ((TCCRA[`COM0B1:`COM0B0] == 2'b00) ? 1'b0 : (TCCRA[`COM0B1:`COM0B0] == 2'b01 ? ((TCCRA[`WGM01:`WGM00] == 2'd1 || TCCRA[`WGM01:`WGM00] == 2'd3) ? TCCRB[`WGM02] : 1'b1) : 1'b1)) : 1'b0;

endmodule
