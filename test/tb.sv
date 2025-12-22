//===========================================================
// bfCPU Project
//-----------------------------------------------------------
// File Name   : tb.v
// Description : Testbench for Tiny Tapeout Chip
//-----------------------------------------------------------
// History :
// Rev.01 2025.11.21 M.Maruyama First Release
//-----------------------------------------------------------
// Copyright (C) 2025-2026 M.Maruyama
//===========================================================

`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/

`ifdef GATESIM
`define TB_UART_BITWIDTH  8681 //ns, 115200bps
`else
`define TB_UART_BITWIDTH  3200 //ns
`endif

//---------------------------------------
// Testbench Top
//---------------------------------------
module tb ();
integer i;

//---------------------
// Dump the signals to a FST file. You can view it with gtkwave or surfer.
//---------------------
`ifdef GATESIM
initial begin
    $dumpfile("tb_gate.vcd");
    $dumpvars(0, tb);
end
`else
initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0, tb);
end
`endif

//---------------------
// Initialize RAM
//---------------------
initial
begin
    $readmemh("./QSPI/multiplication.v", U_M23LC512.MemoryBlock);
end

//-------------------------
// Device Under Test
//-------------------------
logic       ena;      // always 1 when the design is powered, so you can ignore it
logic       clk;      // clock
logic       rst_n;    // reset_n - low to reset
//
logic [7:0] ui_in;    // Dedicated inputs
logic [7:0] uo_out;   // Dedicated outputs
logic [7:0] uio_in;   // IOs: Input path
logic [7:0] uio_out;  // IOs: Output path
logic [7:0] uio_oe;   // IOs: Enable path (active high: 0=input, 1=output)
//
wire        qspi_cs_n;
logic       qspi_cs_n_o;
logic       qspi_cs_e;
wire        qspi_sck;
logic       qspi_sck_o;
logic       qspi_sck_e;
wire  [3:0] qspi_sio;
logic [3:0] qspi_sio_o;
logic [3:0] qspi_sio_e;
logic [3:0] qspi_sio_i;
logic       uart_txd;
logic       uart_rxd;
//
assign qspi_cs_n    = (qspi_cs_e )? qspi_cs_n_o : 1'bz;
assign qspi_sck     = (qspi_sck_e)? qspi_sck_o  : 1'bz;
assign qspi_sio[0]  = (qspi_sio_e[0])? qspi_sio_o[0] : 1'bz;
assign qspi_sio[1]  = (qspi_sio_e[1])? qspi_sio_o[1] : 1'bz;
assign qspi_sio[2]  = (qspi_sio_e[2])? qspi_sio_o[2] : 1'bz;
assign qspi_sio[3]  = (qspi_sio_e[3])? qspi_sio_o[3] : 1'bz;
assign qspi_sio_i[0] = qspi_sio[0];
assign qspi_sio_i[1] = qspi_sio[1];
assign qspi_sio_i[2] = qspi_sio[2];
assign qspi_sio_i[3] = qspi_sio[3];
//
assign qspi_cs_n_o   = uio_out[4];
assign qspi_sck_o    = uio_out[5];
assign qspi_sio_o[0] = uio_out[6];
assign qspi_sio_o[1] = uio_out[7];
assign qspi_sio_o[2] = uio_out[2];
assign qspi_sio_o[3] = uio_out[3];
//
assign qspi_cs_e     = uio_oe[4];
assign qspi_sck_e    = uio_oe[5];
assign qspi_sio_e[0] = uio_oe[6];
assign qspi_sio_e[1] = uio_oe[7];
assign qspi_sio_e[2] = uio_oe[2];
assign qspi_sio_e[3] = uio_oe[3];
//
assign uio_in[6] = qspi_sio_i[0];
assign uio_in[7] = qspi_sio_i[1];
assign uio_in[2] = qspi_sio_i[2];
assign uio_in[3] = qspi_sio_i[3];
//
assign uart_txd = uo_out[0];
assign ui_in[7] = uart_rxd;
//
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif
//
// Chip Top
tt_um_bfcpu U_CHIP
(
    // Include power ports for the Gate Level test:
`ifdef GL_TEST
    .VPWR(VPWR),
    .VGND(VGND),
`endif
    .ui_in  (ui_in),    // Dedicated inputs
    .uo_out (uo_out),   // Dedicated outputs
    .uio_in (uio_in),   // IOs: Input path
    .uio_out(uio_out),  // IOs: Output path
    .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
    .ena    (ena),      // enable - goes high when design is selected
    .clk    (clk),      // clock
    .rst_n  (rst_n)     // not reset
);

//-----------------------------
// SPIRAM (23LC512)
//-----------------------------
pullup (qspi_sio[0]);
pullup (qspi_sio[1]);
pullup (qspi_sio[2]);
pullup (qspi_sio[3]);
//
M23LC512 U_M23LC512
(
    .SI_SIO0      (qspi_sio[0]),
    .SO_SIO1      (qspi_sio[1]),
    .SCK          (qspi_sck),
    .CS_N         (qspi_cs_n),
    .SIO2         (qspi_sio[2]),
    .HOLD_N_SIO3  (qspi_sio[3]),
    .RESET        (~rst_n)
);

//-------------------------
// Support Functions
//-------------------------
//-----------------------------
// UART Generate Receive Data
//-----------------------------
logic detect_in;
logic [7:0] tx_data;
//
always_ff @(posedge clk, negedge rst_n)
begin
    if (~rst_n)
        detect_in <= 1'b0;
    else
        `ifdef GATESIM
         // backslash needs one space after the name 
        detect_in <= U_CHIP.\U_TOP.U_UART.rdphase ;
        `else
        detect_in <= U_CHIP.U_TOP.U_UART.rdphase;
        `endif
end
//
logic [7:0] uart_rxd_data[0:255];
logic [7:0] uart_rxd_seq;
logic       uart_rxd_done;
//
initial
begin
    uart_rxd      = 1'b1;
    uart_rxd_seq  = 8'h00;
    uart_rxd_done = 1'b0;
    //
    for (i = 0; i < 256; i = i + 1) uart_rxd_data[i] = 8'(i);
    //
    @(posedge clk);
    @(posedge rst_n);
    @(posedge clk);
    //
    forever
    begin
        @(posedge detect_in);
        // Message
        $display("===== UART RxD ===== (IN ) at 0x%02x", uart_rxd_data[uart_rxd_seq]);
        #(1);
        // Start RXD
        #(`TB_UART_BITWIDTH);
        uart_rxd = 1'b0;
        uart_rxd_done = 1'b1; // Trigger
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][0];
        uart_rxd_done = 1'b0;
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][1];
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][2];
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][3];
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][4];
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][5];
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][6];
        #(`TB_UART_BITWIDTH);
        uart_rxd = uart_rxd_data[uart_rxd_seq][7];
        #(`TB_UART_BITWIDTH);
        uart_rxd = 1'b1;
        uart_rxd_seq = uart_rxd_seq + 8'h01;
    end
end

//-----------------------------
// UART Detect Traansmit Data
//-----------------------------
logic [7:0] uart_txd_data;
logic       uart_txd_done;
//
initial
begin
    @(posedge clk);
    @(posedge rst_n);
    @(posedge clk);
    //
    forever
    begin
        uart_txd_done = 1'b0;
        @(negedge uart_txd);
        // Start TXD
        uart_txd_data = 8'h00;
        #(`TB_UART_BITWIDTH / 2);
        #(`TB_UART_BITWIDTH);
        uart_txd_data[0] = uart_txd;
        #(`TB_UART_BITWIDTH);
        uart_txd_data[1] = uart_txd;
        #(`TB_UART_BITWIDTH);
        uart_txd_data[2] = uart_txd;
        #(`TB_UART_BITWIDTH);
        uart_txd_data[3] = uart_txd;
        #(`TB_UART_BITWIDTH);
        uart_txd_data[4] = uart_txd;
        #(`TB_UART_BITWIDTH);
        uart_txd_data[5] = uart_txd;
        #(`TB_UART_BITWIDTH);
        uart_txd_data[6] = uart_txd;
        #(`TB_UART_BITWIDTH);
        uart_txd_data[7] = uart_txd;
        uart_txd_done = 1'b1; // Trigger
        #(`TB_UART_BITWIDTH);
        uart_txd_done = 1'b0;
        // Message
        $display("===== UART TxD ===== (OUT) at 0x%02x", uart_txd_data);
    end
end

endmodule
//===========================================================
// End of File
//===========================================================
