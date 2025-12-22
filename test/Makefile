# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES =  \
    tt_um_bfcpu.sv \
    TOP/top.sv     \
    CPU/cpu.sv     \
    CACHE/cache.sv \
    QSPI_SRAM/qspi_sram.sv \
    UART/uart.sv           \
    UART/sasc/trunk/rtl/verilog/sasc_top.v   \
    UART/sasc/trunk/rtl/verilog/sasc_fifo4.v \
    UART/sasc/trunk/rtl/verilog/sasc_brg.v
SYS_DIR=$(PWD)/../test/QSPI
SYSTEM_SOURCES = 23LC512.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))
VERILOG_SOURCES += $(addprefix $(SYS_DIR)/,$(SYSTEM_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DUSE_POWER_PINS
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUNIT_DELAY=\#1
COMPILE_ARGS    += -DSIMULATION
COMPILE_ARGS    += -DGATESIM
VERILOG_SOURCES += $(addprefix $(SYS_DIR)/,$(SYSTEM_SOURCES))
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)
# Add Compile Options
COMPILE_ARGS 		+= -g2012
COMPILE_ARGS 		+= -DSIMULATION

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.sv
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
