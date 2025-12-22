//===========================================================
// bfCPU Project
//-----------------------------------------------------------
// File Name   : uart.sv
// Description : UART Module Top
//-----------------------------------------------------------
// History :
// Rev.01 2025.11.21 M.Maruyama First Release
//-----------------------------------------------------------
// Copyright (C) 2025-2026 M.Maruyama
//===========================================================

//-------------------
// Module Top
//-------------------
module UART
(
    input  logic CLK,
    input  logic RES,
    //
    input  logic        IO_REQ,
    input  logic        IO_WRITE,
    input  logic [ 7:0] IO_WDATA,
    output logic [ 7:0] IO_RDATA,
    output logic        IO_RDY,
    //
    output logic UART_TXD,
    input  logic UART_RXD
);

//---------------------
// Instance of UART IP
//---------------------
logic       we;
logic       re;
logic [7:0] din;
logic [7:0] dout;
logic       sio_ce, sio_ce_x4;
logic [7:0] div0, div1;
logic       tx_full, rx_empty;
logic       rts;
logic _unused;
assign _unused = rts;
//
// Baud Rate = (fCLK/4) / ((div0+2)*(div1))
//     CLK=10MHz, 115200bps
//     115200*4=460800
//     10MHz/460800Hz=22=11*2
//     div0=9 (11-2) , div1=2
`ifdef SIMULATION
// 312500bps (3200ns)
assign div0 = 8'h02;
assign div1 = 8'h02;
`else
// 115200bps
assign div0 = 8'h09;
assign div1 = 8'h02;
`endif
//
sasc_top U_SASC_TOP
(
    .clk       (CLK),
    .rst       (~RES),
    .rxd_i     (UART_RXD),
    .txd_o     (UART_TXD),
    .cts_i     (1'b0),
    .rts_o     (rts),
    .sio_ce    (sio_ce),
    .sio_ce_x4 (sio_ce_x4),
    .din_i     (din),
    .dout_o    (dout),
    .re_i      (re),
    .we_i      (we),
    .full_o    (tx_full),
    .empty_o   (rx_empty)
);
//
sasc_brg BRG
(
    .clk       (CLK),
    .rst       (~RES),
    .div0      (div0),
    .div1      (div1),  
    .sio_ce    (sio_ce),
    .sio_ce_x4 (sio_ce_x4)
);

//------------------
// I/O Data Write
//------------------
logic  waphase;
logic  wdphase;
logic  tx_rdy;
//
assign tx_rdy = ~tx_full;
assign waphase  = IO_REQ & IO_WRITE & IO_RDY;
//
assign we = wdphase & IO_RDY;
//
always_ff @(posedge CLK, posedge RES)
begin
    if (RES)
        din <= 8'h00;
    else if (waphase)
        din <= IO_WDATA;
    else if (IO_RDY)
        din <= 8'h00;
end
//
always_ff @(posedge CLK, posedge RES)
begin
    if (RES)
        wdphase <= 1'b0;
    else if (waphase)
        wdphase <= 1'b1;
    else if (IO_RDY)
        wdphase <= 1'b0;
end

//------------------
// I/O Data Read
//------------------
logic  raphase;
logic  rdphase;
logic  rx_rdy;
//
assign rx_rdy = ~rx_empty;
assign raphase  = IO_REQ & ~IO_WRITE & IO_RDY;
//
always_ff @(posedge CLK, posedge RES)
begin
    if (RES)
        rdphase <= 1'b0;
    else if (raphase)
        rdphase <= 1'b1;
    else if (IO_RDY)
        rdphase <= 1'b0;
end
//
assign re = rdphase & IO_RDY;
assign IO_RDATA = dout;

//-------------------
// I/O Access Ready
//-------------------
assign IO_RDY = (wdphase)? tx_rdy
              : (rdphase)? rx_rdy : 1'b1;

endmodule
//===========================================================
// End of File
//===========================================================
