# Cocotb Makefile for HiL Flight Controller Simulation
#
# Usage:
#   make              — Run the full HiL simulation
#   make clean        — Remove generated files
#
# Prerequisites:
#   - Icarus Verilog (iverilog, vvp)
#   - Python 3 with cocotb installed (pip install cocotb)

# Simulator
SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# RTL source files
VERILOG_SOURCES = \
    $(PWD)/../rtl/gain_regs.v \
    $(PWD)/../rtl/pid_controller.v \
    $(PWD)/../rtl/saturation_guard.v \
    $(PWD)/../rtl/mixer.v \
    $(PWD)/../rtl/pwm_gen.v \
    $(PWD)/../rtl/flight_controller_top.v

# Top-level module name (must match the Verilog module name)
TOPLEVEL = flight_controller_top

# Python test module name (without .py extension)
MODULE = tb_flight_controller

# Compilation flags for Icarus Verilog
COMPILE_ARGS += -g2012
COMPILE_ARGS += -Pflight_controller_top.WATCHDOG_TIMEOUT=500

# Include the cocotb Makefile infrastructure
include $(shell cocotb-config --makefiles)/Makefile.sim
