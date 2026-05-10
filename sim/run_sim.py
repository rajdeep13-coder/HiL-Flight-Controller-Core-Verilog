"""
run_sim.py — Cocotb 2.0 runner script for HiL Flight Controller simulation.
Uses cocotb_tools.runner API (no Makefile/make needed).

Usage:
    python run_sim.py
"""

import os
import sys

# Add iverilog to PATH on Windows if not already there
if sys.platform == "win32":
    iverilog_bin = r"C:\iverilog\bin"
    if iverilog_bin not in os.environ.get("PATH", ""):
        os.environ["PATH"] = iverilog_bin + os.pathsep + os.environ.get("PATH", "")

from cocotb_tools.runner import get_runner


def main():
    # Paths
    proj_dir = os.path.dirname(os.path.abspath(__file__))
    rtl_dir = os.path.join(os.path.dirname(proj_dir), "rtl")
    
    # RTL sources
    sources = [
        os.path.join(rtl_dir, "gain_regs.v"),
        os.path.join(rtl_dir, "pid_controller.v"),
        os.path.join(rtl_dir, "saturation_guard.v"),
        os.path.join(rtl_dir, "mixer.v"),
        os.path.join(rtl_dir, "pwm_gen.v"),
        os.path.join(rtl_dir, "flight_controller_top.v"),
    ]
    
    # Verify all sources exist
    for src in sources:
        if not os.path.exists(src):
            print(f"ERROR: Source file not found: {src}")
            sys.exit(1)
    
    # Get the Icarus Verilog runner
    runner = get_runner("icarus")
    
    # Build the design
    print("=" * 60)
    print("  Building RTL design with Icarus Verilog...")
    print("=" * 60)
    runner.build(
        sources=sources,
        hdl_toplevel="flight_controller_top",
        build_args=["-g2012"],
        build_dir=os.path.join(proj_dir, "sim_build"),
    )
    
    # Run the test
    print("=" * 60)
    print("  Running HiL simulation...")
    print("=" * 60)
    runner.test(
        hdl_toplevel="flight_controller_top",
        test_module="tb_flight_controller",
        build_dir=os.path.join(proj_dir, "sim_build"),
    )
    
    print("=" * 60)
    print("  Simulation complete!")
    print("  CSV log: sim/hil_flight_log.csv")
    print("=" * 60)


if __name__ == "__main__":
    main()
