/*
 * This IP is the ATMEGA 16bit TIMER implementation.
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

//`define TIFR0 ('h00)
`define TOV0 0
`define OCF0A 1
`define OCF0B 2
`define OCF0C 3

//`define GTCCR ('h00)
`define PSRSYNC 0
`define PSRASY  1
`define TSM     7

//`define TCCR0A ('h00)
`define WGM00   0
`define WGM01   1
`define COM0C0  2
`define COM0C1  3
`define COM0B0  4
`define COM0B1  5
`define COM0A0  6
`define COM0A1  7

//`define TCCR0B ('h00)
`define CS00    0
`define CS01    1
`define CS02    2
`define WGM02   3
`define WGM03   4

//`define TCNT0 ('h00)
//`define OCR0A ('h00)
//`define OCR0B ('h00)
//`define OCR0C ('h00)

//`define TIMSK0 ('h00)
`undef TOIE0
`undef OCIE0A
`undef OCIE0B
`undef OCIE0C
`define TOIE0   0
`define OCIE0A  1
`define OCIE0B  2
`define OCIE0C  3

module atmega_tim_16bit # (
    parameter USE_OCRB = "TRUE",
    parameter USE_OCRC = "TRUE",
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter GTCCR_ADDR = 'h23,
    parameter TCCRA_ADDR = 'h80,
    parameter TCCRB_ADDR = 'h81,
    parameter TCCRC_ADDR = 'h82,
    parameter TCNTL_ADDR = 'h84,
    parameter TCNTH_ADDR = 'h85,
    parameter ICRL_ADDR = 'h86,
    parameter ICRH_ADDR = 'h87,
    parameter OCRAL_ADDR = 'h88,
    parameter OCRAH_ADDR = 'h89,
    parameter OCRBL_ADDR = 'h8A,
    parameter OCRBH_ADDR = 'h8B,
    parameter OCRCL_ADDR = 'h8C,
    parameter OCRCH_ADDR = 'h8D,
    parameter TIMSK_ADDR = 'h6F,
    parameter TIFR_ADDR = 'h16
)(
    input rst,
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
    output ocrc_int,
    input ocrc_int_rst,
    output reg oca,
    output reg ocb,
    output reg occ,
    output oca_io_connect,
    output ocb_io_connect,
    output occ_io_connect
    );

reg [7:0]GTCCR;
reg [7:0]TCCRA;
reg [7:0]TCCRB;
reg [7:0]TCCRC;
reg [15:0]TCNT;
reg [15:0]ICR;
reg [15:0]OCRA;
reg [15:0]OCRB;
reg [15:0]OCRC;
reg [7:0]TIFR;
reg [7:0]TIMSK;
reg [7:0]TMP_REG_wr;
reg [BUS_ADDR_DATA_LEN-1:0]TMP_REG_addr;

reg tov_p;
reg tov_n;
reg ocra_p;
reg ocra_n;
reg ocrb_p;
reg ocrb_n;
reg ocrc_p;
reg ocrc_n;

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
reg updt_ocr_on_bottom;
always @ *
begin
    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
        4'd0, 4'd4, 4'd12: // Imediate.
        begin
            updt_ocr_on_top = 1'b0;
            updt_ocr_on_bottom = 1'b0;
        end
        4'd8, 4'd9: // On bottom.
        begin
            updt_ocr_on_top = 1'b0;
            updt_ocr_on_bottom = 1'b1;
        end
        default: // On TOP.
        begin
            updt_ocr_on_top = 1'b1;
            updt_ocr_on_bottom = 1'b0;
        end
    endcase
end

reg [15:0]top_value;
always @ *
begin
    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
        4'd0: top_value = 16'hFFFF;
        4'd1, 4'd5: top_value = 16'h00FF;
        4'd2, 4'd6: top_value = 16'h01FF;
        4'd3, 4'd7: top_value = 16'h03FF;
        4'd8, 4'd10, 4'd12, 4'd14: top_value = ICR;
        default: top_value = OCRA;
    endcase
end

reg [15:0]t_ovf_value;
always @ *
begin
    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
        4'd5, 4'd6, 4'd7, 4'd14, 4'd15: t_ovf_value = top_value;
        4'd0, 4'd4, 4'd12: t_ovf_value = 16'hFFFF;
        default: t_ovf_value = 16'h0000;
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
                TCCRA_ADDR:
                begin
                    bus_dat_out = TCCRA;
                end
                TCCRB_ADDR:
                begin
                    bus_dat_out = TCCRB;
                end
                TCCRC_ADDR:
                begin
                    bus_dat_out = TCCRC;
                end
                TCNTL_ADDR:
                begin
                    bus_dat_out = TCNT[7:0];
                end
                TCNTH_ADDR:
                begin
                    bus_dat_out = TCNT[15:8];
                end
                ICRL_ADDR:
                begin
                    bus_dat_out = ICR[7:0];
                end
                ICRH_ADDR:
                begin
                    bus_dat_out = ICR[15:8];
                end
                OCRAL_ADDR:
                begin
                    bus_dat_out = OCRA[7:0];
                end
                OCRAH_ADDR:
                begin
                    bus_dat_out = OCRA[15:8];
                end
                OCRBL_ADDR:
                begin
                    if(USE_OCRB == "TRUE")
                        bus_dat_out = OCRB[7:0];
                end
                OCRBH_ADDR:
                begin
                    if(USE_OCRB == "TRUE")
                        bus_dat_out = OCRB[15:8];
                end
                OCRCL_ADDR:
                begin
                    if(USE_OCRC == "TRUE")
                        bus_dat_out = OCRC[7:0];
                end
                OCRCH_ADDR:
                begin
                    if(USE_OCRC == "TRUE")
                        bus_dat_out = OCRC[15:8];
                end
                TIFR_ADDR:
                begin
                    bus_dat_out = TIFR;
                end
                TIMSK_ADDR:
                begin
                    bus_dat_out = TIMSK;
                end
            endcase
        end
    end
end

/* Set "oc" pin on specified conditions*/
always @ (posedge clk)
begin
    if(rst)
    begin
        TMP_REG_wr <= 8'h00;
        TMP_REG_addr <= 8'h00;
        GTCCR <= 8'h00;
        TCCRA <= 8'h00;
        TCCRB <= 8'h00;
        TCCRC <= 8'h00;
        TCNT <= 16'h0000;
        ICR <= 16'h0000;
        OCRA <= 16'h0000;
        OCRB <= 16'h0000;
        OCRC <= 16'h0000;
        TIFR <= 8'h00;
        TIMSK <= 8'h00;
        tov_p <= 1'b0;
        tov_n <= 1'b0;
        ocra_p <= 1'b0;
        ocra_n <= 1'b0;
        ocrb_p <= 1'b0;
        ocrb_n <= 1'b0;
        ocrc_p <= 1'b0;
        ocrc_n <= 1'b0;
        oca <= 1'b0;
        ocb <= 1'b0;
        occ <= 1'b0;
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
        if(ocrc_p ^ ocrc_n)
        begin
            TIFR[`OCF0C] <= 1'b1;
            ocrc_n <= ocrc_p;
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
        if(ocrc_int_rst)
        begin
            TIFR[`OCF0C] <= 1'b0;
        end
        // Sample one IO core clock once every prescaller positive edge clock.
        clk_int_del <= clk_int; // Shift prescaller clock to a delay register every IO core positive edge clock to detect prescaller positive edges.
        if((~clk_int_del & clk_int) || TCCRB[`CS02:`CS00] == 3'b001) // if prescaller clock = IO core clock disable prescaller positive edge detector.
        begin
            if(up_count && (TCNT < top_value))
            begin
                TCNT <= TCNT + 1'b1;
            end
            else if(!up_count && (TCNT > 16'h0000))
            begin
                TCNT <= TCNT - 1'b1;
            end
            else if(TCNT > top_value)
            begin
                TCNT <= top_value;
            end
            // OCRA
            if(TCNT == OCRA)
            begin
                case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                    4'd4, 4'd12: oca <= ~oca;
                    default:
                    begin
                        case(OCRA)
                            16'h0000: oca <= 1'b0;
                            16'hFFFF: oca <= 1'b1;
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
                    begin
                        ocra_p <= ~ocra_p;
                    end
                end
                else
                begin
                    ocra_p <= 1'b0;
                    ocra_n <= 1'b0;
                end
            end
            // !OCRA
            if(USE_OCRB == "TRUE")
            begin
                // OCRB
                if(TCNT == OCRB)
                begin
                    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                        4'd4, 4'd12: ocb <= ~ocb;
                        default:
                        begin
                            case(OCRB)
                                16'h0000: ocb <= 1'b0;
                                16'hFFFF: ocb <= 1'b1;
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
                        begin
                            ocrb_p <= ~ocrb_p;
                        end
                    end
                    else
                    begin
                        ocrb_p <= 1'b0;
                        ocrb_n <= 1'b0;
                    end
                end
            end // USE_OCRB != "TRUE"
            if(USE_OCRC == "TRUE")
            begin
                // OCRC
                if(TCNT == OCRC)
                begin
                    case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                        4'd4, 4'd12: occ <= ~occ;
                        default:
                        begin
                            case(OCRC)
                                16'h0000: occ <= 1'b0;
                                16'hFFFF: occ <= 1'b1;
                                default:
                                begin
                                    if(up_count)
                                    begin
                                        case(TCCRA[`COM0C1:`COM0C0])
                                            2'h1: occ <= ~occ;
                                            2'h2: occ <= 1'b0;
                                            2'h3: occ <= 1'b1;
                                        endcase
                                    end
                                    else
                                    begin
                                        case(TCCRA[`COM0C1:`COM0C0])
                                            2'h1: occ <= ~occ;
                                            2'h2: occ <= 1'b1;
                                            2'h3: occ <= 1'b0;
                                        endcase
                                    end
                                end
                            endcase
                        end
                    endcase
                    if(TIMSK[`OCIE0C] == 1'b1)
                    begin
                        if(ocrc_p == ocrc_n && clk_active == 1'b1)
                        begin
                            ocrc_p <= ~ocrc_p;
                        end
                    end
                    else
                    begin
                        ocrc_p <= 1'b0;
                        ocrc_n <= 1'b0;
                    end
                end
            end // USE_OCRC != "TRUE"
            // TCNT overflow logick.
            if(TCNT == t_ovf_value)
            begin
                if(TIMSK[`TOIE0] == 1'b1)
                begin
                    if(tov_p == tov_n && clk_active == 1'b1)
                    begin
                        tov_p <= ~tov_p;
                    end
                end
                else
                begin
                    tov_p <= 1'b0;
                    tov_n <= 1'b0;
                end
            end
            if(TCNT == top_value)
            begin
                case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                    4'd1, 4'd2, 4'd3, 4'd8, 4'd9, 4'd10, 4'd11:
                    begin
                        up_count <= 1'b0;
                        TCNT <= TCNT - 1'b1;
                    end
                    default: TCNT <= 16'h0000;
                endcase
            end
            else if(TCNT == 16'h0000)
            begin
                case({TCCRB[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]})
                    4'd1, 4'd2, 4'd3, 4'd8, 4'd9, 4'd10, 4'd11:
                    begin
                        if (!up_count)
                        begin
                            up_count <= 1'b1;
                            TCNT <= TCNT + 1'b1;
                        end
                    end
                endcase
            end
        end
        // Write registers
        if(wr_dat)
        begin
            case(addr_dat)
                GTCCR_ADDR:
                begin
                    GTCCR <= bus_dat_in;
                end
                TCCRA_ADDR:
                begin
                    TCCRA <= bus_dat_in;
                end
                TCCRB_ADDR:
                begin
                    TCCRB <= bus_dat_in;
                end
                TCCRC_ADDR:
                begin
                    TCCRC <= bus_dat_in;
                end
/*              TCNTL_ADDR:
                begin
                    TCNT <= (TMP_REG_addr == TCNTH_ADDR) ? {TMP_REG_wr, bus_dat_in} : {8'd0, bus_dat_in};
                end
                TCNTH_ADDR:
                begin
                    TMP_REG_wr <= bus_dat_in;
                    TMP_REG_addr <= TCNTH_ADDR;
                end */
                ICRL_ADDR:
                begin
                    ICR <= (TMP_REG_addr == ICRH_ADDR) ? {TMP_REG_wr, bus_dat_in} : {8'd0, bus_dat_in};
                end
                ICRH_ADDR:
                begin
                    TMP_REG_wr <= bus_dat_in;
                    TMP_REG_addr <= ICRH_ADDR;
                end
                OCRAL_ADDR:
                begin
                    OCRA <= (TMP_REG_addr == OCRAH_ADDR) ? {TMP_REG_wr, bus_dat_in} : {8'd0, bus_dat_in};
                end
                OCRAH_ADDR:
                begin
                    TMP_REG_wr <= bus_dat_in;
                    TMP_REG_addr <= OCRAH_ADDR;
                end
                OCRBL_ADDR:
                begin
                    if(USE_OCRB == "TRUE")
                        OCRB <= (TMP_REG_addr == OCRBH_ADDR) ? {TMP_REG_wr, bus_dat_in} : {8'd0, bus_dat_in};
                end
                OCRBH_ADDR:
                begin
                    if(USE_OCRB == "TRUE")
                        TMP_REG_wr <= bus_dat_in;
                        TMP_REG_addr <= OCRBH_ADDR;
                end
                OCRCL_ADDR:
                begin
                    if(USE_OCRC == "TRUE")
                        OCRC <= (TMP_REG_addr == OCRCH_ADDR) ? {TMP_REG_wr, bus_dat_in} : {8'd0, bus_dat_in};
                end
                OCRCH_ADDR:
                begin
                    if(USE_OCRC == "TRUE")
                        TMP_REG_wr <= bus_dat_in;
                        TMP_REG_addr <= OCRCH_ADDR;
                end
/*              TIFR_ADDR:
                begin
                    TIFR <= TIFR & ~bus_dat_in;
                end */
                TIMSK_ADDR:
                begin
                    TIMSK <= bus_dat_in;
                end
            endcase
        end
    end
end

assign tov_int = TIFR[`TOV0];
assign ocra_int = TIFR[`OCF0A];
assign ocrb_int = TIFR[`OCF0B];
assign ocrc_int = TIFR[`OCF0C];

assign oca_io_connect = (TCCRA[`COM0A1:`COM0A0] == 2'b00) ? 1'b0 : (TCCRA[`COM0A1:`COM0A0] == 2'b01 ? (({TCCRA[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]} == 4'd14 || {TCCRA[`WGM03:`WGM02], TCCRA[`WGM01:`WGM00]} == 4'd15) ? 1'b1 : 1'b0) : 1'b1);
assign ocb_io_connect = (TCCRA[`COM0B1:`COM0B0] == 2'b00 || TCCRA[`COM0B1:`COM0B0] == 2'b01) ? 1'b0 : 1'b1;
assign occ_io_connect = (TCCRA[`COM0C1:`COM0C0] == 2'b00 || TCCRA[`COM0C1:`COM0C0] == 2'b01) ? 1'b0 : 1'b1;

endmodule
