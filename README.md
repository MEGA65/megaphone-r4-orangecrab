# OrangeCrab code for MEGAphone R4

The MEGAphone R4 has an Orange Crab as "utility FPGA", which largely controls
the power system for the MEGAphone. The idea is that the main FPGA (which sucks
more power) can stay off when the phone is idle, and the utility FPGA can notice
if the user presses the power button, or if one of the cellular modems (or other
communications modules) triggers an event that needs the main FPGA, e.g., an 
incoming phone call or SMS message.

So all it needs to do is to buffer serial comms from the communications bays, and
monitor the buttons and control the power enable lines.  Well, it's not _quite_ 
that simple, but its a pretty good approximation.

# Based on:

# OrangeCrab example projects
This repository contains example code to be run on the OrangeCrab.

---

## RISCV examples
These examples make use of the Vexriscv CPU created inside the FPGA by the bootloader. The RISCV firmware is copied across into the FLASH by the bootloader. If the bootloader determines that it has not loaded new gateware, then the CPU will simply adjust it's program counter to start executing the newly loaded programs.

* __riscv.blink__ - The most basic example. Blink a LED with RISCV firmware
* __riscv.button__ - Read button input and toggle LED colour 

## Verilog examples
These examples use Yosys + NextPnR, to synthesis (or compile) verilog into a bitstream. A nice term for this is gateware. Since it is analogous to firmware, but describes how the FPGA needs to be configured. 

This gateware can be loaded onto the OrangeCrab using its DFU bootloader.

* __verilog.blink__ - The most basic verilog example. Blink a LED with gateware

