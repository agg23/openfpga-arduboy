/*
 * This IP is the ATMEGA UART implementation.
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

// UCSRA
`define RXC     7
`define TXC     6
`define UDRE    5
`define FE      4
`define DOR     3
`define UPE     2
`define U2X     1
`define MPCM    0
// UCSRB
`define RXCIE   7
`define TXCIE   6
`define UDREIE  5
`define RXEN    4
`define TXEN    3
`define UCSZ2   2
`define RXB8    1
`define TXB8    0
// UCSRC
`define UMSEL1  7
`define UMSEL0  6
`define UPM1    5
`define UPM0    4
`define USBS    3
`define UCSZ1   2
`define UCSZ0   1
`define UCPOL   0
// UCSRD
`define CTSEN   1
`define RTSEN   0

module atmega_uart # (
    parameter BUS_ADDR_DATA_LEN = 8,
    parameter UDR_ADDR = 'hc1,
    parameter UCSRA_ADDR = 'hc8,
    parameter UCSRB_ADDR = 'hc9,
    parameter UCSRC_ADDR = 'hca,
    parameter UCSRD_ADDR = 'h00,
    parameter UBRRL_ADDR = 'hcc,
    parameter UBRRH_ADDR = 'hcd,
    parameter USE_TX = "TRUE",
    parameter USE_RX = "TRUE"
    )(
    input rst,
    input clk,
    input [BUS_ADDR_DATA_LEN-1:0]addr_dat,
    input wr_dat,
    input rd_dat,
    input [7:0]bus_dat_in,
    output reg [7:0]bus_dat_out,
    output rxc_int,
    input rxc_int_rst,
    output txc_int,
    input txc_int_rst,
    output udre_int,
    input udre_int_rst,

    input rx,
    output reg tx,
    output tx_connect
    );

reg [7:0]UDR_rx;
reg [7:0]UDR_tx;
reg [7:0]UCSRA;
reg [7:0]UCSRB;
reg [7:0]UCSRC;
reg [7:0]UCSRD;
reg [7:0]UBRRL;
reg [3:0]UBRRH;

reg udre_p, udre_n;
reg udre_int_p, udre_int_n;
reg txc_p, txc_n;

wire [15:0]tx_prescaller_value_int = ({UBRRH, UBRRL} + 1) << (UCSRA[`U2X] ? 3 : 4);
wire [13:0]rx_prescaller_value_int = ({UBRRH, UBRRL} + 1) << (UCSRA[`U2X] ? 0 : 1);
reg [3:0]bit_per_word_int;
wire stop_bits_int = UCSRC[`USBS];
always @ *
begin
    case({UCSRB[`UCSZ2], UCSRC[`UCSZ1:`UCSZ0]})
    3'd0: bit_per_word_int = 4'd5;
    3'd1: bit_per_word_int = 4'd6;
    3'd2: bit_per_word_int = 4'd7;
    3'd3: bit_per_word_int = 4'd8;
    3'd7: bit_per_word_int = 4'd9;
    default: bit_per_word_int = 4'd8;
    endcase
end

reg [15:0]tx_prescaller_cnt;
reg [1:0]tx_state;
reg [8:0]tx_shift_reg;
reg [3:0]tx_bit_cnt;
reg send_p;
reg send_n;
reg udre_old;

always @ (posedge clk)
begin
    if(USE_TX == "TRUE")
    begin
        if(rst)
        begin
            UDR_tx <= 8'h00;
            UCSRA[3:0] <= 4'h0;
            UCSRB[7:2] <= 6'h00;
            UCSRB[0] <= 1'b0;
            UCSRC <= 8'h00;
            UCSRD <= 8'h00;
            UBRRL <= 8'h00;
            UBRRH <= 8'h00;
            tx_prescaller_cnt <= 16'h0000;
            tx_state <= 2'h0;
            tx <= 1'b1;
            send_p <= 1'b0;
            send_n <= 1'b0;
            udre_p <= 1'b0;
            udre_n <= 1'b0;
            txc_p <= 1'b0;
            txc_n <= 1'b0;
            tx_bit_cnt <= 4'h0;
            tx_shift_reg <= 9'h000;
            udre_old <= 1'b1;
        end
        else
        begin
            udre_old <= UCSRA[`UDRE];
            if(~UCSRB[`TXEN])
            begin
                tx_prescaller_cnt <= 16'h0000;
                tx_state <= 2'h0;
                tx <= 1'b1;
                send_p <= 1'b0;
                send_n <= 1'b0;
                udre_p <= 1'b0;
                udre_n <= 1'b0;
                tx_bit_cnt <= 4'h0;
            end
            else
            begin
                if(tx_prescaller_cnt)
                begin
                    tx_prescaller_cnt <= tx_prescaller_cnt - 1;
                end
                else
                begin
                    tx_prescaller_cnt <= tx_prescaller_value_int;
                    if(tx_state == 2'h1)
                    begin
                        if(tx_bit_cnt)
                        begin
                            tx <= tx_shift_reg[0];
                        end
                        else
                        begin
                            if(~stop_bits_int)
                            begin
                                tx_state <= 2'h3;
                            end
                            else
                            begin
                                tx_state <= 2'h2;
                            end
                            tx <= 1'b1;
                        end
                        tx_shift_reg <= {1'b0, tx_shift_reg[8:1]};
                        tx_bit_cnt <= tx_bit_cnt - 1;
                    end
                    else if(tx_state == 2'h2)
                    begin
                        tx_state <= 2'h3;
                    end
                    else if(tx_state == 2'h3)
                    begin
                        tx_state <= 2'h0;
                        txc_p <= ~txc_n;
                    end
                end
                if(udre_p ^ udre_n & send_p == send_n)
                begin
                    send_p <= ~send_n;
                    udre_n <= udre_p;
                end
                if((send_p ^ send_n) & tx_state == 2'h0)
                begin
                    send_n <= send_p;
                    tx_prescaller_cnt <= tx_prescaller_value_int;
                    tx_shift_reg <= UDR_tx;
                    tx_shift_reg[8] <= UCSRB[`TXB8];
                    tx <= 1'b0;
                    tx_bit_cnt <= bit_per_word_int;
                    tx_state <= 2'h1;
                end
            end
        end
        if(~UCSRB[`UDREIE])
        begin
            udre_p <= 1'b0;
            udre_n <= 1'b0;
        end
        if(~UCSRB[`TXCIE])
        begin
            txc_p <= 1'b0;
            txc_n <= 1'b0;
        end
        if(wr_dat)
        begin
            case(addr_dat)
                UDR_ADDR:
                begin
                    if(UDR_ADDR >= 'h40 && UDR_ADDR != 'h0)
                    begin
                        UDR_tx <= bus_dat_in;
                        udre_p <= ~udre_n;
                    end
                end
                UCSRA_ADDR:
                begin
                    if(UCSRA_ADDR >= 'h40 && UCSRA_ADDR != 'h0)
                    begin
                        txc_n <= txc_p;
                        UCSRA[3:0] <= bus_dat_in[3:0];
                    end
                end
                UCSRB_ADDR:
                begin
                    if(UCSRB_ADDR >= 'h40 && UCSRB_ADDR != 'h0)
                    begin
                        UCSRB[7:2] <= bus_dat_in[7:2];
                        UCSRB[0] <= bus_dat_in[0];
                    end
                end
                UCSRC_ADDR:
                begin
                    if(UCSRC_ADDR >= 'h40 && UCSRC_ADDR != 'h0)
                        UCSRC <= bus_dat_in;
                end
                UCSRD_ADDR:
                begin
                    if(UCSRD_ADDR >= 'h40 && UCSRD_ADDR != 'h0)
                        UCSRD <= bus_dat_in;
                end
                UBRRL_ADDR:
                begin
                    if(UBRRL_ADDR >= 'h40 && UBRRL_ADDR != 'h0)
                        UBRRL <= bus_dat_in;
                end
                UBRRH_ADDR:
                begin
                    if(UBRRH_ADDR >= 'h40 && UBRRH_ADDR != 'h0)
                        UBRRH <= bus_dat_in;
                end
            endcase
        end
        if(udre_int_rst)
        begin
            udre_int_n <= udre_int_p;
        end
        if(txc_int_rst)
        begin
            txc_n <= txc_p;
        end
        if(~udre_old & UCSRA[`UDRE])
            udre_int_p <= ~udre_int_n;
    end
end

always @ * UCSRA[`UDRE] = udre_p == udre_n & send_p == send_n;
always @ * UCSRA[`TXC] = txc_p ^ txc_n;
assign udre_int = UCSRB[`UDREIE] ? udre_int_p ^ udre_int_n : 1'b0;
assign txc_int = UCSRB[`TXCIE] ? UCSRA[`TXC] : 1'b0;
assign tx_connect = UCSRB[`TXEN];

reg [13:0]rx_prescaller_cnt;
reg [1:0]rx_state;
reg [8:0]rx_shift_reg;
reg [2:0]rx_pin_state_0_cnt;
reg [2:0]rx_pin_state_1_cnt;
reg [3:0]rx_sample_cnt;
reg [3:0]rx_bit_cnt;
wire rx_pin_state = (rx_pin_state_0_cnt <= rx_pin_state_1_cnt) & ((rx_pin_state_0_cnt == 3'b100) | (rx_pin_state_1_cnt == 3'b100));
reg rxc_p;
reg rxc_n;

always @ (posedge clk)
begin
    if(USE_RX == "TRUE")
    begin
        if(rst)
        begin
            rx_prescaller_cnt <= 16'h0000;
            rx_state <= 2'h0;
            rx_pin_state_0_cnt <= 2'h0;
            rx_pin_state_1_cnt <= 2'h0;
            rx_bit_cnt <= 4'h0;
            rx_sample_cnt <= 4'h0;
            rx_shift_reg <= 'h0;
            UCSRA[`FE] <= 1'b0;
            rxc_p <= 1'b0;
            rxc_n <= 1'b0;
            UDR_rx <= 8'h00;
            UCSRB[`RXB8] <= 1'b0;
        end
        else
        begin
            if(~UCSRB[`RXEN])
            begin
                rx_prescaller_cnt <= 16'h0000;
                rx_state <= 2'h0;
                rx_pin_state_0_cnt <= 2'h0;
                rx_pin_state_1_cnt <= 2'h0;
                rx_bit_cnt <= 4'h0;
                rx_sample_cnt <= 4'h0;
                rx_shift_reg <= 'h0;
                UCSRA[`FE] <= 1'b0;
                rxc_p <= 1'b0;
                rxc_n <= 1'b0;
            end
            else
            begin
                if(rx_prescaller_cnt)
                begin
                    rx_prescaller_cnt <= rx_prescaller_cnt - 1;
                end
                else
                begin
                    rx_prescaller_cnt <= rx_prescaller_value_int - 1;
                    if(rx_sample_cnt < 4'd8)
                    begin
                        rx_sample_cnt <= rx_sample_cnt + 1;
                    end
                    else
                    begin
                        rx_sample_cnt <= 4'h0;
                    end
                    if(rx)
                    begin
                        if(rx_pin_state_1_cnt != 3'b100)
                        begin
                            rx_pin_state_1_cnt <= rx_pin_state_1_cnt + 1;
                        end
                        if(rx_pin_state_0_cnt)
                        begin
                            rx_pin_state_0_cnt <= rx_pin_state_0_cnt - 1;
                        end
                    end
                    else
                    begin
                        if(rx_pin_state_0_cnt != 3'b100)
                        begin
                            rx_pin_state_0_cnt <= rx_pin_state_0_cnt + 1;
                        end
                        if(rx_pin_state_1_cnt)
                        begin
                            rx_pin_state_1_cnt <= rx_pin_state_1_cnt - 1;
                        end
                    end
                    case(rx_state)
                        2'h0:
                        begin
                            if(~rx)
                            begin // Check for low level on RX pin.
                                rx_pin_state_0_cnt <= 3'h0;
                                rx_pin_state_1_cnt <= 3'h0;
                                rx_sample_cnt <= 4'h2;
                                rx_state <= 2'h1;
                            end
                        end
                        2'h1:
                        begin
                            if(rx_sample_cnt == 4'h5)
                            begin
                                if(~rx_pin_state) // Check the START condition thru filter.
                                begin // Start confirmed
                                    rx_bit_cnt <= 4'h0;
                                    rx_shift_reg <= 'h0;
                                    rx_state <= 2'h2;
                                end
                                else
                                begin // False start, go back to IDLE
                                    rx_pin_state_0_cnt <= 3'h0;
                                    rx_pin_state_1_cnt <= 3'h0;
                                    rx_state <= 2'h0;
                                end
                            end
                        end
                        2'h2:
                        begin
                            if(rx_sample_cnt == 4'h0)
                            begin
                                rx_pin_state_0_cnt <= 3'h0;
                                rx_pin_state_1_cnt <= 3'h0;
                            end
                            if(rx_sample_cnt == 4'h5)
                                begin
                                if(rx_bit_cnt < bit_per_word_int)
                                begin
                                    rx_shift_reg[rx_bit_cnt] <= rx_pin_state;
                                    rx_bit_cnt <= rx_bit_cnt + 1;
                                end
                                else
                                begin
                                    if(rx_pin_state)
                                    begin
                                        UCSRA[`FE] <= 1'b0;
                                    end
                                    else
                                    begin
                                        UCSRA[`FE] <= 1'b1;
                                    end
                                    if(~stop_bits_int)
                                    begin
                                        UDR_rx <= rx_shift_reg;
                                        UCSRB[`RXB8] <= rx_shift_reg[8];
                                        rxc_p <= ~rxc_n;
                                        rx_pin_state_0_cnt <= 3'h0;
                                        rx_pin_state_1_cnt <= 3'h0;
                                        rx_prescaller_cnt <= 0;
                                        rx_sample_cnt <= 4'd0;
                                        rx_state <= 2'h0;
                                    end
                                    else
                                    begin
                                        rx_state <= 2'h3;
                                    end
                                end
                            end
                        end
                        2'h3:
                        begin
                            if(rx_sample_cnt == 4'h5)
                            begin
                                if(~rx_pin_state)
                                begin
                                    UCSRA[`FE] <= 1'b1;
                                end
                                UDR_rx <= rx_shift_reg;
                                UCSRB[`RXB8] <= rx_shift_reg[8];
                                rxc_p <= ~rxc_n;
                                rx_pin_state_0_cnt <= 3'h0;
                                rx_pin_state_1_cnt <= 3'h0;
                                rx_prescaller_cnt <= 0;
                                rx_sample_cnt <= 4'd0;
                                rx_state <= 2'h0;
                            end
                        end
                    endcase
                end
            end
        end
        if(~UCSRB[`RXCIE])
        begin
            rxc_p <= 1'b0;
            rxc_n <= 1'b0;
        end
        if(rd_dat)
        begin
            case(addr_dat)
                UDR_ADDR:
                begin
                    if(UDR_ADDR >= 'h40)
                        rxc_n <= rxc_p;
                end
            endcase
        end
        if(rxc_int_rst)
        begin
            rxc_n <= rxc_p;
        end
    end
end

always @ * UCSRA[`RXC] = rxc_p ^ rxc_n;
assign rxc_int = UCSRB[`RXCIE] ? UCSRA[`RXC] : 1'b0;


always @ *
begin
    if(rst)
    begin
        bus_dat_out = 8'b0;
    end
    else
    begin
        bus_dat_out = 8'b0;
        if(rd_dat)
        begin
            case(addr_dat)
                UDR_ADDR:
                begin
                    if(UDR_ADDR >= 'h40 && UDR_ADDR != 'h0)
                        bus_dat_out = UDR_rx;
                end
                UCSRA_ADDR:
                begin
                    if(UCSRA_ADDR >= 'h40 && UCSRA_ADDR != 'h0)
                        bus_dat_out = UCSRA;
                end
                UCSRB_ADDR:
                begin
                    if(UCSRB_ADDR >= 'h40 && UCSRB_ADDR != 'h0)
                        bus_dat_out = UCSRB;
                end
                UCSRC_ADDR:
                begin
                    if(UCSRC_ADDR >= 'h40 && UCSRC_ADDR != 'h0)
                        bus_dat_out = UCSRC;
                end
                UCSRD_ADDR:
                begin
                    if(UCSRD_ADDR >= 'h40 && UCSRD_ADDR != 'h0)
                        bus_dat_out = UCSRD;
                end
                UBRRL_ADDR:
                begin
                    if(UBRRL_ADDR >= 'h40 && UBRRL_ADDR != 'h0)
                        bus_dat_out = UBRRL;
                end
                UBRRH_ADDR:
                begin
                    if(UBRRH_ADDR >= 'h40 && UBRRH_ADDR != 'h0)
                        bus_dat_out = UBRRH;
                end
            endcase
        end
    end
end
endmodule
