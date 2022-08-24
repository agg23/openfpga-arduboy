/*
 * This IP is the MEGA/XMEGA ATMEGA32U4 implementation.
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

`include "mega-def.v"

//`define USE_PLL
`define USE_TIMER_0
`define USE_TIMER_1
`define USE_TIMER_3
`define USE_TIMER_4
`define USE_SPI_1
`define USE_UART_1
`define USE_EEPROM

/* ATMEGA32U4 is a "MEGA_ENHANCED_128K" family */
`define CORE_TYPE               `MEGA_ENHANCED_128K
`define ROM_ADDR_WIDTH          14
`define BUS_ADDR_DATA_LEN       12
`define RAM_ADDR_WIDTH          12
`define EEP_ADDR_WIDTH          10
`define RESERVED_RAM_FOR_IO     'h100

`define VECTOR_INT_TABLE_SIZE   42
`define WATCHDOG_CNT_WIDTH      0//27

/* TIMMERS PRESCALLERS MODULE */
module tim_013_prescaller (
    input rst,
    input clk,
    output clk8,
    output clk64,
    output clk256,
    output clk1024
);
reg [9:0]cnt;

always @ (posedge clk)
begin
    if(rst)
    begin
        cnt <= 10'h000;
    end
    else
    begin
        cnt <= cnt + 10'd1;
    end
end

assign clk8 = cnt[2];
assign clk64 = cnt[5];
assign clk256 = cnt[7];
assign clk1024 = cnt[9];

endmodule
/* !TIMMERS PRESCALLERS MODULE */

module atmega32u4 # (
    parameter REGS_REGISTERED = "FALSE",
    parameter USE_HALT = "FALSE",
    parameter USE_PIO_B = "TRUE",
    parameter USE_PIO_C = "TRUE",
    parameter USE_PIO_D = "TRUE",
    parameter USE_PIO_E = "TRUE",
    parameter USE_PIO_F = "TRUE",
    parameter USE_PLL = "TRUE",
    parameter USE_PLL_HI_FREQ = "FALSE",
    parameter USE_TIMER_0 = "TRUE",
    parameter USE_TIMER_1 = "TRUE",
    parameter USE_TIMER_3 = "TRUE",
    parameter USE_TIMER_4 = "TRUE",
    parameter USE_SPI_1 = "TRUE",
    parameter USE_UART_1 = "TRUE",
    parameter USE_EEPROM = "TRUE"
)(
    input rst,
    input clk,
    input clk_pll,
    output [`ROM_ADDR_WIDTH-1:0] pgm_addr,
    input [15:0] pgm_data,
    input [5:0] buttons,
    input [7:0] joystick_analog,
    input status,
    output [2:0] RGB,
    output Buzzer1, Buzzer2, DC, spi_scl, spi_mosi,
    input uart_rx,
    output uart_tx
    );

wire core_clk = clk;
wire wdt_rst;

/* CORE WIRES */
wire [`BUS_ADDR_DATA_LEN-1:0]data_addr;
wire [7:0]core_data_out;
wire data_write;
reg  [7:0]core_data_in;
wire data_read;
/* !CORE WIRES */

/* IO WIRES */
wire [7:0]pb_in;
wire [7:0]pc_in;
wire [7:0]pd_in;
wire [7:0]pe_in;
wire [7:0]pf_in;
wire [7:0]pb_out;
wire [7:0]pc_out;
wire [7:0]pd_out;
wire [7:0]pe_out;
wire [7:0]pf_out;
/* !IO WIRES */

/* IO PIN FUNCTION CHANGE REQUEST */
wire tim0_oca_io_connect;
wire tim0_ocb_io_connect;
wire tim1_oca_io_connect;
wire tim1_ocb_io_connect;
wire tim1_occ_io_connect;
wire tim3_oca_io_connect;
wire tim3_ocb_io_connect;
wire tim3_occ_io_connect;
wire tim4_ocap_io_connect;
wire tim4_ocan_io_connect;
wire tim4_ocbp_io_connect;
wire tim4_ocbn_io_connect;
wire tim4_occp_io_connect;
wire tim4_occn_io_connect;
wire tim4_ocdp_io_connect;
wire tim4_ocdn_io_connect;
wire uart_tx_io_connect;
wire spi_io_connect;
wire io_conn_slave;
/* !IO PIN FUNCTION CHANGE REQUEST */
wire pll_enabled;
/* IO ALTERNATIVE FUNCTION */
wire tim0_oca;
wire tim0_ocb;
wire tim1_oca;
wire tim1_ocb;
wire tim1_occ;
wire tim3_oca;
wire tim3_ocb;
wire tim3_occ;
wire tim4_oca;
wire tim4_ocb;
wire tim4_occ;
wire tim4_ocd;
wire spi_miso;
wire usb_ck_out;
wire tim_ck_out;
/* !IO ALTERNATIVE FUNCTION */

assign pf_in[6] = buttons[0]; // BUTTON RIGHT
assign pf_in[5] = buttons[1]; // BUTTON LEFT
assign pf_in[4] = buttons[2]; // BUTTON DOWN
assign pf_in[7] = buttons[3]; // BUTTON UP
assign pe_in[6] = buttons[4]; // BUTTON A
assign pb_in[4] = buttons[5]; // BUTTON B

// RGB LED is common anode (ie. HIGH = OFF)
assign RGB[2] = tim1_ocb_io_connect ? tim1_ocb : ~(pb_out[6]);
assign RGB[1] = tim0_oca_io_connect ? ~tim0_oca : ~(pb_out[7]);
assign RGB[0] = tim1_oca_io_connect ? tim1_oca : ~(pb_out[5]);

assign Buzzer1 = pc_out[6];
assign Buzzer2 = tim4_ocap_io_connect ? tim4_oca : 1'b0;
assign DC = pd_out[4];

/* Interrupt wires */
wire ram_sel = |data_addr[`BUS_ADDR_DATA_LEN-1:8];
wire int_int0 = 0;
wire int_int1 = 0;
wire int_int2 = 0;
wire int_int3 = 0;
wire int_reserved0 = 0;
wire int_reserved1 = 0;
wire int_int6 = 0;
wire int_reserved3 = 0;
wire int_pcint0 = 0;
wire int_usb_general = 0;
wire int_usb_endpoint = 0;
wire int_wdt = 0;
wire int_reserved4 = 0;
wire int_reserved5 = 0;
wire int_reserved6 = 0;
wire int_timer1_capt = 0;
wire int_timer1_compa;
wire int_timer1_compb;
wire int_timer1_compc;
wire int_timer1_ovf;
wire int_timer0_compa;
wire int_timer0_compb;
wire int_timer0_ovf;
wire int_spi_stc;
wire int_usart1_rx;
wire int_usart1_udre;
wire int_usart1_tx;
wire int_analog_comp = 0;
wire int_adc = 0;
wire int_ee_ready;
wire int_timer3_capt = 0;
wire int_timer3_compa;
wire int_timer3_compb;
wire int_timer3_compc;
wire int_timer3_ovf;
wire int_twi = 0;
wire int_spm_ready = 0;
wire int_timer4_compa;
wire int_timer4_compb;
wire int_timer4_compd;
wire int_timer4_ovf;
wire int_timer4_fpf = 0;
/* !Interrupt wires */

/* Interrupt reset wires */
wire int_int0_rst;
wire int_int1_rst;
wire int_int2_rst;
wire int_int3_rst;
wire int_reserved0_rst;
wire int_reserved1_rst;
wire int_int6_rst;
wire int_reserved3_rst;
wire int_pcint0_rst;
wire int_usb_general_rst;
wire int_usb_endpoint_rst;
wire int_wdt_rst;
wire int_reserved4_rst;
wire int_reserved5_rst;
wire int_reserved6_rst;
wire int_timer1_capt_rst;
wire int_timer1_compa_rst;
wire int_timer1_compb_rst;
wire int_timer1_compc_rst;
wire int_timer1_ovf_rst;
wire int_timer0_compa_rst;
wire int_timer0_compb_rst;
wire int_timer0_ovf_rst;
wire int_spi_stc_rst;
wire int_usart1_rx_rst;
wire int_usart1_udre_rst;
wire int_usart1_tx_rst;
wire int_analog_comp_rst;
wire int_adc_rst;
wire int_ee_ready_rst;
wire int_timer3_capt_rst;
wire int_timer3_compa_rst;
wire int_timer3_compb_rst;
wire int_timer3_compc_rst;
wire int_timer3_ovf_rst;
wire int_twi_rst;
wire int_spm_ready_rst;
wire int_timer4_compa_rst;
wire int_timer4_compb_rst;
wire int_timer4_compd_rst;
wire int_timer4_ovf_rst;
wire int_timer4_fpf_rst;
/* !Interrupt reset wires */

/* ADC */
wire [7:0]dat_random_d_out;
unstable_counters unstable_counters
(
    .clk(clk),
    .rst(rst),
    .rd(data_read & ~ram_sel),
    .addr(data_addr[7:0]),
    .joystick_analog(joystick_analog),
    .status(status),
    .dat(dat_random_d_out)
);
/* !ADC */

/* PORTB */
wire [7:0]dat_pb_d_out;
generate
if (USE_PIO_B == "TRUE")
begin: PORTB
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h25),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h24),
    .PIN_ADDR('h23),
    .PINMASK(8'b11110000),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b00010000),
    .OUT_ENABLED_MASK(8'b11101111)
)pio_b(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pb_d_out),

    .io_in(pb_in),
    .io_out(pb_out)
    );
end
else
begin
assign dat_pb_d_out = 0;
end
endgenerate
/* !PORTB */

/* PORTC */
wire [7:0]dat_pc_d_out;
generate
if (USE_PIO_C == "TRUE")
begin: PORTC
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h28),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h27),
    .PIN_ADDR('h26),
    .PINMASK(8'b11000000),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b00000000),
    .OUT_ENABLED_MASK(8'b11000000)
)pio_c(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pc_d_out),

    .io_in(pc_in),
    .io_out(pc_out)
    );
end
else
begin
assign dat_pc_d_out = 0;
end
endgenerate
/* !PORTC */

/* PORTD */
wire [7:0]dat_pd_d_out;
generate
if (USE_PIO_D == "TRUE")
begin: PORTD
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h2b),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h2a),
    .PIN_ADDR('h29),
    .PINMASK(8'b11010000),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b00000000),
    .OUT_ENABLED_MASK(8'b11111111)
)pio_d(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pd_d_out),

    .io_in(pd_in),
    .io_out(pd_out)
    );
end
else
begin
assign dat_pd_d_out = 0;
end
endgenerate
/* !PORTD */

/* PORTE */
wire [7:0]dat_pe_d_out;
generate
if (USE_PIO_E == "TRUE")
begin: PORTE
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h2e),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h2d),
    .PIN_ADDR('h2c),
    .PINMASK(8'b01000000),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b01000000),
    .OUT_ENABLED_MASK(8'b00000000)
)pio_e(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(0), //read-only
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pe_d_out),

    .io_in(pe_in),
    .io_out(pe_out)
    );
end
else
begin
assign dat_pe_d_out = 0;
end
endgenerate
/* !PORTE */

/* PORTF */
wire [7:0]dat_pf_d_out;
generate
if (USE_PIO_F == "TRUE")
begin: PORTF
atmega_pio # (
    .BUS_ADDR_DATA_LEN(8),
    .PORT_WIDTH(8),
    .USE_CLEAR_SET("FALSE"),
    .PORT_OUT_ADDR('h31),
    .PORT_CLEAR_ADDR('h00),
    .PORT_SET_ADDR('h01),
    .DDR_ADDR('h30),
    .PIN_ADDR('h2f),
    .PINMASK(8'b11110011),
    .PULLUP_MASK(8'b00000000),
    .PULLDN_MASK(8'b00000000),
    .INVERSE_MASK(8'b11110000),
    .OUT_ENABLED_MASK(8'b00000000)
)pio_f(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(0), //read-only
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pf_d_out),

    .io_in(pf_in),
    .io_out(pf_out)
    );
end
else
begin
assign dat_pf_d_out = 0;
end
endgenerate
/* !PORTF */

/* SPI */
wire [7:0]dat_spi_d_out;
generate
if (USE_SPI_1 == "TRUE")
begin: SPI_DISPLAY
atmega_spi_m # (
    .BUS_ADDR_DATA_LEN(8),
    .SPCR_ADDR('h4c),
    .SPSR_ADDR('h4d),
    .SPDR_ADDR('h4e),
    .DINAMIC_BAUDRATE("FALSE"),
    .BAUDRATE_CNT_LEN(0),
    .BAUDRATE_DIVIDER(0),
    .USE_TX("TRUE"),
    .USE_RX("FALSE")
)spi(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_spi_d_out),
    .int_out(int_spi_stc),
    .int_rst(int_spi_stc_rst),
    .io_connect(spi_io_connect),
    .io_conn_slave(io_conn_slave),

    .scl(spi_scl),
    .miso(spi_miso),
    .mosi(spi_mosi)
    );
end
else
begin
assign dat_spi_d_out = 0;
assign int_spi_stc = 1'b0;
assign spi_io_connect = 1'b0;
end
endgenerate
/* !SPI */

/* UART */
wire [7:0]dat_uart0_d_out;
generate
if (USE_UART_1 == "TRUE")
begin: UART1
atmega_uart # (
    .BUS_ADDR_DATA_LEN(8),
    .UDR_ADDR('hce),
    .UCSRA_ADDR('hc8),
    .UCSRB_ADDR('hc9),
    .UCSRC_ADDR('hca),
    .UBRRL_ADDR('hcc),
    .UBRRH_ADDR('hcd),
    .USE_TX("TRUE"),
    .USE_RX("TRUE")
    )uart(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_uart0_d_out),
    .rxc_int(int_usart1_rx),
    .rxc_int_rst(int_usart1_rx_rst),
    .txc_int(int_usart1_tx),
    .txc_int_rst(int_usart1_tx_rst),
    .udre_int(int_usart1_udre),
    .udre_int_rst(int_usart1_udre_rst),

    .rx(uart_rx),
    .tx(uart_tx),
    .tx_connect(uart_tx_io_connect)
    );
end
else
begin
assign dat_uart0_d_out = 1'b0;
assign int_usart1_rx = 1'b0;
assign int_usart1_tx = 1'b0;
assign int_usart1_udre = 1'b0;
assign uart_tx_io_connect = 1'b0;
end
endgenerate
/* UART */

/* TIMER PRESCALLER */
wire clk8;
wire clk64;
wire clk256;
wire clk1024;
tim_013_prescaller tim_013_prescaller_inst(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024)
);
/* !TIMER PRESCALLER */

/* TIMER 0 */
wire [7:0]dat_tim0_d_out;
generate
if (USE_TIMER_0 == "TRUE")
begin:TIMER0
atmega_tim_8bit # (
    .USE_OCRB("TRUE"),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h43),
    .TCCRA_ADDR('h44),
    .TCCRB_ADDR('h45),
    .TCNT_ADDR('h46),
    .OCRA_ADDR('h47),
    .OCRB_ADDR('h48),
    .TIMSK_ADDR('h6E),
    .TIFR_ADDR('h35)
)tim_0(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim0_d_out),
    .tov_int(int_timer0_ovf),
    .tov_int_rst(int_timer0_ovf_rst),
    .ocra_int(int_timer0_compa),
    .ocra_int_rst(int_timer0_compa_rst),
    .ocrb_int(int_timer0_compb),
    .ocrb_int_rst(int_timer0_compb_rst),
    .oca(tim0_oca),
    .ocb(tim0_ocb),
    .oca_io_connect(tim0_oca_io_connect),
    .ocb_io_connect(tim0_ocb_io_connect)
    );
end
else
begin
assign dat_tim0_d_out = 0;
assign int_timer0_ovf = 1'b0;
assign int_timer0_compa = 1'b0;
assign tim0_oca_io_connect = 1'b0;
assign tim0_ocb_io_connect = 1'b0;
end
endgenerate
/* !TIMER 0 */

/* TIMER 1 */
wire [7:0]dat_tim1_d_out;
generate
if (USE_TIMER_1 == "TRUE")
begin: TIMER1
atmega_tim_16bit # (
    .USE_OCRB("TRUE"),
    .USE_OCRC("TRUE"),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h43),
    .TCCRA_ADDR('h80),
    .TCCRB_ADDR('h81),
    .TCCRC_ADDR('h82),
    .TCNTL_ADDR('h84),
    .TCNTH_ADDR('h85),
    .ICRL_ADDR('h86),
    .ICRH_ADDR('h87),
    .OCRAL_ADDR('h88),
    .OCRAH_ADDR('h89),
    .OCRBL_ADDR('h8A),
    .OCRBH_ADDR('h8B),
    .OCRCL_ADDR('h8C),
    .OCRCH_ADDR('h8D),
    .TIMSK_ADDR('h6F),
    .TIFR_ADDR('h36)
)tim_1(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim1_d_out),
    .tov_int(int_timer1_ovf),
    .tov_int_rst(int_timer1_ovf_rst),
    .ocra_int(int_timer1_compa),
    .ocra_int_rst(int_timer1_compa_rst),
    .ocrb_int(int_timer1_compb),
    .ocrb_int_rst(int_timer1_compb_rst),
    .ocrc_int(int_timer1_compc),
    .ocrc_int_rst(int_timer1_compc_rst),
    .oca(tim1_oca),
    .ocb(tim1_ocb),
    .occ(tim1_occ),
    .oca_io_connect(tim1_oca_io_connect),
    .ocb_io_connect(tim1_ocb_io_connect),
    .occ_io_connect(tim1_occ_io_connect)
    );
end
else
begin
assign dat_tim1_d_out = 0;
end
endgenerate
/* !TIMER 1 */

/* TIMER 3 */
wire [7:0]dat_tim3_d_out;
generate
if (USE_TIMER_3 == "TRUE")
begin: TIMER3
atmega_tim_16bit # (
    .USE_OCRB("FALSE"),
    .USE_OCRC("FALSE"),
    .BUS_ADDR_DATA_LEN(8),
    .GTCCR_ADDR('h43),
    .TCCRA_ADDR('h90),
    .TCCRB_ADDR('h91),
    .TCCRC_ADDR('h92),
    .TCNTL_ADDR('h94),
    .TCNTH_ADDR('h95),
    .ICRL_ADDR('h96),
    .ICRH_ADDR('h97),
    .OCRAL_ADDR('h98),
    .OCRAH_ADDR('h99),
    .OCRBL_ADDR('h9A),
    .OCRBH_ADDR('h9B),
    .OCRCL_ADDR('h9C),
    .OCRCH_ADDR('h9D),
    .TIMSK_ADDR('h71),
    .TIFR_ADDR('h38)
)tim_3(
    .rst(rst),
    .clk(clk),
    .clk8(clk8),
    .clk64(clk64),
    .clk256(clk256),
    .clk1024(clk1024),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim3_d_out),
    .tov_int(int_timer3_ovf),
    .tov_int_rst(int_timer3_ovf_rst),
    .ocra_int(int_timer3_compa),
    .ocra_int_rst(int_timer3_compa_rst),
    .ocrb_int(int_timer3_compb),
    .ocrb_int_rst(int_timer3_compb_rst),
    .ocrc_int(int_timer3_compc),
    .ocrc_int_rst(int_timer3_compc_rst),
    .oca(tim3_oca),
    .ocb(tim3_ocb),
    .occ(tim3_occ),
    .oca_io_connect(tim3_oca_io_connect),
    .ocb_io_connect(tim3_ocb_io_connect),
    .occ_io_connect(tim3_occ_io_connect)
    );
end
else
begin
assign dat_tim3_d_out = 0;
end
endgenerate
/* !TIMER 3 */

/* PLL */
wire [7:0]dat_pll_d_out;
generate
if(USE_PLL == "TRUE")
begin: PLL
atmega_pll # (
    .BUS_ADDR_DATA_LEN(8),
    .PLLCSR_ADDR('h49),
    .PLLFRQ_ADDR('h52),
    .USE_PLL(USE_PLL_HI_FREQ)
)pll(
    .rst(rst),
    .clk(clk),
    .clk_pll(clk_pll),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_pll_d_out),
    .pll_enabled(pll_enabled),

    .usb_ck_out(usb_ck_out),
    .tim_ck_out(tim_ck_out)
    );
end
else
begin
    assign dat_pll_d_out = 0;
end
endgenerate
/* !PLL */

/* TIMER 4 */
wire [7:0]dat_tim4_d_out;
generate
if (USE_TIMER_4 == "TRUE")
begin: TIMER4
atmega_tim_10bit # (
    .USE_OCRA("TRUE"),
    .USE_OCRB("TRUE"),
    .USE_OCRD("TRUE"),
    .BUS_ADDR_DATA_LEN(8),
    .TCCRA_ADDR('hc0),
    .TCCRB_ADDR('hc1),
    .TCCRC_ADDR('hc2),
    .TCCRD_ADDR('hc3),
    .TCCRE_ADDR('hc4),
    .TCNTL_ADDR('hbe),
    .TCH_ADDR('hbf),
    .OCRA_ADDR('hcf),
    .OCRB_ADDR('hd0),
    .OCRC_ADDR('hd1),
    .OCRD_ADDR('hd2),
    .TIMSK_ADDR('h72),
    .TIFR_ADDR('h39)
)tim_4(
    .rst(rst),
    .clk(clk),
    .clk_pll(tim_ck_out),
    .pll_enabled(pll_enabled),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_tim4_d_out),
    .tov_int(int_timer4_ovf),
    .tov_int_rst(int_timer4_ovf_rst),
    .ocra_int(int_timer4_compa),
    .ocra_int_rst(int_timer4_compa_rst),
    .ocrb_int(int_timer4_compb),
    .ocrb_int_rst(int_timer4_compb_rst),
    .ocrc_int(),
    .ocrc_int_rst(),
    .ocrd_int(int_timer4_compd),
    .ocrd_int_rst(int_timer4_compd_rst),
    .oca(tim4_oca),
    .ocb(tim4_ocb),
    .occ(tim4_occ),
    .ocd(tim4_ocd),
    .ocap_io_connect(tim4_ocap_io_connect),
    .ocan_io_connect(tim4_ocan_io_connect),
    .ocbp_io_connect(tim4_ocbp_io_connect),
    .ocbn_io_connect(tim4_ocbn_io_connect),
    .occp_io_connect(tim4_occp_io_connect),
    .occn_io_connect(tim4_occn_io_connect),
    .ocdp_io_connect(tim4_ocdp_io_connect),
    .ocdn_io_connect(tim4_ocdn_io_connect)
    );
end
else
begin
assign dat_tim4_d_out = 0;
end
endgenerate
/* !TIMER 4 */

/* EEPROM */
wire [7:0]dat_eeprom_d_out;
generate
if (USE_EEPROM == "TRUE")
begin: EEPROM
atmega_eep # (
    .BUS_ADDR_DATA_LEN(8),
    .EEARH_ADDR('h42),
    .EEARL_ADDR('h41),
    .EEDR_ADDR('h40),
    .EECR_ADDR('h3F),
    .EEP_SIZE(1024)
)eep(
    .rst(rst),
    .clk(clk),
    .addr_dat(data_addr[7:0]),
    .wr_dat(data_write & ~ram_sel),
    .rd_dat(data_read & ~ram_sel),
    .bus_dat_in(core_data_out),
    .bus_dat_out(dat_eeprom_d_out),
    .int_out(int_ee_ready),
    .int_rst(int_ee_ready_rst)
    /*.ext_eep_addr(0),
    .ext_eep_data_in(0),
    .ext_eep_data_wr(1'b0),
    .ext_eep_data_out(ext_eep_data_out),
    .ext_eep_data_rd(1'b0),
    .ext_eep_data_en(1'b0),
    .content_modifyed(eep_content_modifyed),
    .debug()*/
    );
end
else
begin
assign dat_eeprom_d_out = 0;
end
endgenerate
/* !EEPROM */

/* RAM */
wire [7:0]ram_bus_out;
mega_ram  #(
    .ADDR_BUS_WIDTH(`RAM_ADDR_WIDTH),
    .DATA_BUS_WIDTH(8),
    .RAM_PATH("")
)ram(
    .rst(rst),
    .clk(core_clk),
    .we(data_write & ram_sel),
    .a(data_addr[`RAM_ADDR_WIDTH-1:0]/* - `RESERVED_RAM_FOR_IO*/),
    .d_in(core_data_out),
    .d_out(ram_bus_out)
);
/* !RAM */

/* DATA BUS IN DEMULTIPLEXER */
always @ *
begin
    core_data_in = ram_bus_out;
    if(~ram_sel) begin
        case(data_addr[7:0])
            'h25, 'h24, 'h23: core_data_in = dat_pb_d_out;
            'h28, 'h27, 'h26: core_data_in = dat_pc_d_out;
            'h2b, 'h2a, 'h29: core_data_in = dat_pd_d_out;
            'h2e, 'h2d, 'h2c: core_data_in = dat_pe_d_out;
            'h31, 'h30, 'h2f: core_data_in = dat_pf_d_out;
            'h4c, 'h4d, 'h4e: core_data_in = dat_spi_d_out;
            'h44, 'h45, 'h46,
            'h47, 'h48, 'h6E,
            'h35:             core_data_in = dat_tim0_d_out;
            'h6F, 'h36:       core_data_in = dat_tim1_d_out;
            'h71, 'h38:       core_data_in = dat_tim3_d_out;
            'hc0, 'hc1, 'hc2,
            'hc3, 'hc4, 'hbe,
            'hbf, 'hcf, 'hd0,
            'hd1, 'hd2, 'h72,
            'h39:             core_data_in = dat_tim4_d_out;
            'h49, 'h52:       core_data_in = dat_pll_d_out;
            'h42, 'h41, 'h40,
            'h3F:             core_data_in = dat_eeprom_d_out;
            'hce, 'hc8, 'hc9,
            'hca, 'hcc, 'hcd: core_data_in = dat_uart0_d_out;
            'h78, 'h79:       core_data_in = dat_random_d_out;
        endcase
        case(data_addr[7:4])
            'h8:              core_data_in = dat_tim1_d_out;
            'h9:              core_data_in = dat_tim3_d_out;
        endcase
    end
end
/* !DATA BUS IN DEMULTIPLEXER */

/* ATMEGA CORE */
mega # (
    .CORE_TYPE(`CORE_TYPE),
    .ROM_ADDR_WIDTH(`ROM_ADDR_WIDTH),
    .RAM_ADDR_WIDTH(`BUS_ADDR_DATA_LEN),
    .WATCHDOG_CNT_WIDTH(`WATCHDOG_CNT_WIDTH),/* If is 0 the watchdog is disabled */
    .VECTOR_INT_TABLE_SIZE(`VECTOR_INT_TABLE_SIZE),/* If is 0 the interrupt module is disabled */
    .USE_HALT(USE_HALT),
    .REGS_REGISTERED(REGS_REGISTERED)
    )atmega32u4_inst(
    .rst(rst),
    .sys_rst_out(wdt_rst),
    // Core clock.
    .clk(core_clk),
    // Watchdog clock input that can be different from the core clock.
    .clk_wdt(core_clk),
    // Used to halt the core.
    .halt(0),
    .halt_ack(),
    // FLASH space data interface.
    .pgm_addr(pgm_addr),
    .pgm_data(pgm_data),
    // RAM space data interface.
    .data_addr(data_addr),
    .data_out(core_data_out),
    .data_write(data_write),
    .data_in(core_data_in),
    .data_read(data_read),
    // Interrupt lines from all IO's.
    .int_sig({
    int_timer4_fpf, int_timer4_ovf, int_timer4_compd, int_timer4_compb, int_timer4_compa,
    int_spm_ready,
    int_twi,
    int_timer3_ovf, int_timer3_compc, int_timer3_compb, int_timer3_compa, int_timer3_capt,
    int_ee_ready,
    int_adc,
    int_analog_comp,
    int_usart1_tx, int_usart1_udre, int_usart1_rx,
    int_spi_stc,
    int_timer0_ovf, int_timer0_compb, int_timer0_compa,
    int_timer1_ovf, int_timer1_compc, int_timer1_compb, int_timer1_compa, int_timer1_capt,
    int_reserved6, int_reserved5, int_reserved4,
    int_wdt,
    int_usb_endpoint, int_usb_general,
    int_pcint0,
    int_reserved3,
    int_int6,
    int_reserved1, int_reserved0,
    int_int3, int_int2, int_int1, int_int0}
    ),
    // Interrupt reset lines going to all IO's.
    .int_rst({
    int_timer4_fpf_rst, int_timer4_ovf_rst, int_timer4_compd_rst, int_timer4_compb_rst, int_timer4_compa_rst,
    int_spm_ready_rst,
    int_twi_rst,
    int_timer3_ovf_rst, int_timer3_compc_rst, int_timer3_compb_rst, int_timer3_compa_rst, int_timer3_capt_rst,
    int_ee_ready_rst,
    int_adc_rst,
    int_analog_comp_rst,
    int_usart1_tx_rst, int_usart1_udre_rst, int_usart1_rx_rst,
    int_spi_stc_rst,
    int_timer0_ovf_rst, int_timer0_compb_rst, int_timer0_compa_rst,
    int_timer1_ovf_rst, int_timer1_compc_rst, int_timer1_compb_rst, int_timer1_compa_rst, int_timer1_capt_rst,
    int_reserved6_rst, int_reserved5_rst, int_reserved4_rst,
    int_wdt_rst,
    int_usb_endpoint_rst, int_usb_general_rst,
    int_pcint0_rst,
    int_reserved3_rst,
    int_int6_rst,
    int_reserved1_rst, int_reserved0_rst,
    int_int3_rst, int_int2_rst, int_int1_rst, int_int0_rst}
    )
);
/* !ATMEGA CORE */

endmodule
