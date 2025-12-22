#//===========================================================
#// bfCPU Project
#//-----------------------------------------------------------
#// File Name   : test.py
#// Description : Cocotb of bfCPU
#//-----------------------------------------------------------
#// History :
#// Rev.01 2025.11.21 M.Maruyama First Release
#//-----------------------------------------------------------
#// Copyright (C) 2025-2026 M.Maruyama
#//===========================================================
# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # Set the clock period to 100 ns (10 MHz)
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Assset Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0x80 # bit7=RXD
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    
    # Initialize UART RXD Patterns
    await ClockCycles(dut.clk, 10)
    for i in range(0, 256, 1):
        dut.uart_rxd_data[i].value = (i + 2) % 256

    # Negate Reset
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut._log.info("Simulation of bfCPU")

    # Wait for one clock cycle to see the output values
    await ClockCycles(dut.clk, 10)

    # Wait for TXD
    await dut.uart_txd_done.rising_edge
    assert dut.uart_txd_data.value == 0x06 # 2*3
    #
    await dut.uart_txd_done.rising_edge
    assert dut.uart_txd_data.value == 0x14 # 4*5
    #
    await dut.uart_txd_done.rising_edge
    assert dut.uart_txd_data.value == 0x2a # 6*7

    # Finish
    await ClockCycles(dut.clk, 100)

#//===========================================================
# End of File
#//===========================================================
