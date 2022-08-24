/*
 * This IP is the ATMEGA EEPROM implementation.
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

module atmega_eep # (
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter EEARH_ADDR = 'h20,
    parameter EEARL_ADDR = 'h21,
    parameter EEDR_ADDR = 'h22,
    parameter EECR_ADDR = 'h23,
    parameter EEP_SIZE = 512
)(
    input rst,
    input clk,

    input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
    input wr_dat,
    input rd_dat,
    input [7:0]bus_dat_in,
    output reg [7:0]bus_dat_out,

    output int_out,
    input int_rst,

    input [16:0]ext_eep_addr,
    input [7:0]ext_eep_data_in,
    input ext_eep_data_wr,
    output [7:0]ext_eep_data_out,
    input ext_eep_data_rd,
    input ext_eep_data_en
    );

// EEPROM_SYS_FLAGS_SHOW_LOGO_LEDS / EEPROM_AUDIO_ON
(* ram_init_file = "EEPROM.mif" *)
reg [7:0]eep[EEP_SIZE-1 : 0];

reg [7:0]EEARH;
reg [7:0]EEARL;
reg [7:0]EEDR_WRITE;
reg [7:0]EEDR_READ;
reg [7:0]EECR;

reg[2:0]eempe_timeout_cnt;
reg eep_wr;
reg [7:0]dat_to_write;
reg [7:0]read_tmp;

reg int_p;
reg int_n;

always @ *
begin
    bus_dat_out = 8'h00;
    if(rd_dat)
    begin
        case(addr_dat)
            EEARH_ADDR: bus_dat_out = EEARH;
            EEARL_ADDR: bus_dat_out = EEARL;
            EEDR_ADDR: bus_dat_out = EEDR_READ;
            EECR_ADDR: bus_dat_out = EECR;
        endcase
    end
end

always @ (posedge clk)
begin
    if(rst)
    begin
        EEARH <= 8'h00;
        EEARL <= 8'h00;
        EEDR_READ <= 8'h00;
        EEDR_WRITE <= 8'h00;
        EECR <= 8'h00;
        //content_modifyed <= 1'b0;
        eempe_timeout_cnt <= 3'h0;
        int_p <= 1'b0;
        int_n <= 1'b0;
        dat_to_write <= 1'b0;
        eep_wr <= 1'b0;
    end
    else
    begin
        eep_wr <= 1'b0;
        if(eempe_timeout_cnt)
        begin
            eempe_timeout_cnt <= eempe_timeout_cnt - 1;
        end
        if(wr_dat)
        begin
            case(addr_dat)
                EEARH_ADDR: EEARH <= bus_dat_in;
                EEARL_ADDR: EEARL <= bus_dat_in;
                EEDR_ADDR: EEDR_WRITE <= bus_dat_in;
                EECR_ADDR:
                begin
                    EECR <= bus_dat_in;
                    if(EECR[2] | bus_dat_in[1])
                    begin
                        eempe_timeout_cnt <= 3'h4;
                    end
                end
            endcase
        end
        if(&EECR[2:1])
        begin
            if(|eempe_timeout_cnt & ({EEARH, EEARL} > 2))
            begin
                case(EECR[5:4])
                    2'h0, 2'h2:
                    begin
                        dat_to_write <= EEDR_WRITE;
                        eep_wr <= 1'b1;
                    end
                    2'h1:
                    begin
                        dat_to_write <= 8'h00;
                        eep_wr <= 1'b1;
                    end
                endcase
            end
            EECR[2:1] <= 2'b00;
            if(int_p == int_n)
            begin
                int_p <= ~int_p;
            end
        end
        if(EECR[0])
        begin
            EEDR_READ <= read_tmp;
            EECR[0] <= 1'b0;
        end
        if(int_rst)
        begin
            int_n <= int_p;
        end
        //if(eep_wr)
        //begin
            //content_modifyed <= content_modifyed | 1'b1;
        //end
    end
end

always @ (posedge clk)
begin
    if(eep_wr & ({EEARH, EEARL} > 2))
    begin
        eep[ext_eep_data_en ? ext_eep_addr : {EEARH, EEARL}] <= ext_eep_data_en ? ext_eep_data_in : dat_to_write;
    end
    read_tmp <= eep[ext_eep_data_en ? ext_eep_addr : {EEARH, EEARL}];
end

assign ext_eep_data_out = (ext_eep_data_rd & ext_eep_data_en) ? read_tmp : 8'h00;
assign int_out = EECR[3] ? (int_p ^ int_n) : 1'b0;

endmodule
