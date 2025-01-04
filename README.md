<pre>README: Z80 CPU Emulator Project


  
Overview
This project implements a CPU emulator for the Z80 8-bit processor. The emulator interprets and executes a subset of Z80 instructions, simulating the CPU's behavior and managing its state, including registers, flags, memory, and program counter (PC). The emulator is built in C++ with modularity and expandability in mind.



Project Structure
z80.h
Defines CPU registers, flags, and function declarations.

z80.cpp
Implements the functionality of the CPU registers and utility functions for manipulating the CPU's state.

z80_execute.cpp
Contains the opcode implementation for Z80 instructions, including ALU operations, branching, memory interactions, and bitwise operations.

project-main.c
A simplified testing framework for loading programs and running them on the emulator.


  
Features
Instruction Handling
Supports a variety of instructions including data transfer, arithmetic, logical, and branching.
  
Flag Updates
Handles updates to CPU flags (e.g., Zero, Sign, Carry, Parity/Overflow, Half-Carry, Subtract) based on operations performed.

Memory Management
Implements reading and writing to memory locations through helper functions (z80_mem_read, z80_mem_write).

Control Flow
Supports conditional and unconditional jumps, calls, and returns.

Registers
Includes implementations for 8-bit registers (A, B, C, D, E, H, L) and 16-bit register pairs (BC, DE, HL).

Interrupt Handling
Implements instructions related to interrupts (EI, DI, IM), although no actual interrupt emulation is performed.

Testing and Debugging
Provides a testing setup to validate instruction implementation and CPU behavior.



Usage
To build and run the project:

1. Build the Emulator
run 'make' on the terminal.

2. Load a Program
Prepare a Z80 binary file (program.bin) using tools like printf or a Z80 assembler.
run './z80_emulator program.bin' on the terminal.

The emulator will load the program, execute it, and print the CPU state upon encountering a HALT instruction.


  
Example
Simple Halt Example
halt

Command:
printf '\x76' > program.bin


  
Makefile
The provided Makefile automates compilation:

Build Command: make
Clean Command: make clean


  
Known Limitations
Unimplemented Instructions
Certain Z80 instructions (e.g., IN, OUT, block transfer instructions) are not supported and are treated as NOPs.

Interrupts
The emulator recognizes EI, DI, and IM but does not emulate hardware interrupts.


  
References
http://www.zilog.com/docs/z80/z80cpu_um.pdf
http://z80.info/z80code.txt

</pre>
