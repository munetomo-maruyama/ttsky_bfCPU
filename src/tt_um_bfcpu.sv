/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_bfcpu (
    input  logic [7:0] ui_in,    // Dedicated inputs
    output logic [7:0] uo_out,   // Dedicated outputs
    input  logic [7:0] uio_in,   // IOs: Input path
    output logic [7:0] uio_out,  // IOs: Output path
    output logic [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  logic       ena,      // always 1 when the design is powered, so you can ignore it
    input  logic       clk,      // clock
    input  logic       rst_n     // reset_n - low to reset
);
//
logic       qspi_cs_n;
logic       qspi_sck;
logic [3:0] qspi_sio_o;
logic [3:0] qspi_sio_i;
logic       qspi_cs_e;
logic       qspi_sck_e;
logic [3:0] qspi_sio_e;
logic       uart_txd;
logic       uart_rxd;
//
TOP U_TOP
(
    .CLK   (clk),
    .RES_N (rst_n),
    //
    //
    .QSPI_CS_N  (qspi_cs_n),
    .QSPI_CS_E  (qspi_cs_e),
    .QSPI_SCK   (qspi_sck),
    .QSPI_SCK_E (qspi_sck_e),
    .QSPI_SIO_O (qspi_sio_o),
    .QSPI_SIO_E (qspi_sio_e),
    .QSPI_SIO_I (qspi_sio_i),
    //
    .UART_TXD      (uart_txd),
    .UART_RXD      (uart_rxd)
);

// All output pins must be assigned. If not used, assign to 0.
assign uo_out[0] = uart_txd;
assign uo_out[1] = 1'b0;
assign uo_out[2] = 1'b0;
assign uo_out[3] = 1'b0;
assign uo_out[4] = 1'b0;
assign uo_out[5] = 1'b0;
assign uo_out[6] = 1'b0;
assign uo_out[7] = 1'b0;
//
assign uio_out[0] = 1'b0;
assign uio_out[1] = 1'b0;
assign uio_out[2] = qspi_sio_o[2];
assign uio_out[3] = qspi_sio_o[3];
assign uio_out[4] = qspi_cs_n;
assign uio_out[5] = qspi_sck;
assign uio_out[6] = qspi_sio_o[0];
assign uio_out[7] = qspi_sio_o[1];
//
assign uio_oe[0] = 1'b0;
assign uio_oe[1] = 1'b0;
assign uio_oe[2] = qspi_sio_e[2];
assign uio_oe[3] = qspi_sio_e[3];
assign uio_oe[4] = qspi_cs_e;
assign uio_oe[5] = qspi_sck_e;
assign uio_oe[6] = qspi_sio_e[0];
assign uio_oe[7] = qspi_sio_e[1];
//
assign uart_rxd = ui_in[7];
assign qspi_sio_i[0] = uio_in[6];
assign qspi_sio_i[1] = uio_in[7];
assign qspi_sio_i[2] = uio_in[2];
assign qspi_sio_i[3] = uio_in[3];

// List all unused inputs to prevent warnings
wire _unused;
assign _unused = &{1'b0, ui_in[6:0], uio_in[5:4], uio_in[1:0], ena};

endmodule
