// z80_execute.cpp
#include "z80.h"
#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iomanip>
using namespace std;

extern "C"
{
    void z80_mem_write(uint16_t addr, uint8_t value);
    uint8_t z80_mem_read(uint16_t addr);
    void z80_mem_write16(uint16_t addr, uint16_t value);
    uint16_t z80_mem_read16(uint16_t addr);
    void z80_init(void);
    int z80_execute(int cycles);
    void z80_mem_dump(const char *fn);
}

void Z80::execute(uint8_t opcode)
{
    if (halt != true) {
        switch (opcode)
        {
            // NOP and HALT
            case 0x00:
                cycles += 4;
                break; // NOP
            case 0x76: // HALT
                printState();
                halt = true;
                z80_mem_dump("memory.bin");
                cycles += 4;
                break;

            // LD r, n instructions
            case 0x06:
                loadImmediate(B);
                cycles += 7;
                break; // LD B, n
            case 0x0E:
                loadImmediate(C);
                cycles += 7;
                break; // LD C, n
            case 0x16:
                loadImmediate(D);
                cycles += 7;
                break; // LD D, n
            case 0x1E:
                loadImmediate(E);
                cycles += 7;
                break; // LD E, n
            case 0x26:
                loadImmediate(H);
                cycles += 7;
                break; // LD H, n
            case 0x2E:
                loadImmediate(L);
                cycles += 7;
                break; // LD L, n
            case 0x3E:
                loadImmediate(A);
                cycles += 7;
                break; // LD A, n

            // LD r, r' instructions
            case 0x40:
                LD(B, B);
                cycles += 4;
                break; // LD B, B
            case 0x41:
                LD(B, C);
                cycles += 4;
                break; // LD B, C
            case 0x42:
                LD(D, B);
                cycles += 4;
                break; // LD D, B
            case 0x43:
                LD(B, E);
                cycles += 4;
                break; // LD B, E
            case 0x44:
                LD(B, H);
                cycles += 4;
                break; // LD B, H
            case 0x45:
                LD(B, L);
                cycles += 4;
                break; // LD B, L
            case 0x47:
                LD(B, A);
                cycles += 4;
                break; // LD B, A

            case 0x48:
                LD(C, B);
                cycles += 4;
                break; // LD C, B
            case 0x49:
                LD(C, C);
                cycles += 4;
                break; // LD C, C
            case 0x4A:
                LD(C, D);
                cycles += 4;
                break; // LD C, D
            case 0x4B:
                LD(C, E);
                cycles += 4;
                break; // LD C, E
            case 0x4C:
                LD(C, H);
                cycles += 4;
                break; // LD C, H
            case 0x4D:
                LD(C, L);
                cycles += 4;
                break; // LD C, L
            case 0x4F:
                LD(C, A);
                cycles += 4;
                break; // LD C, A

            case 0x78:
                LD(A, B);
                cycles += 4;
                break; // LD A, B
            case 0x79:
                LD(A, C);
                cycles += 4;
                break; // LD A, C
            case 0x7A:
                LD(A, D);
                cycles += 4;
                break; // LD A, D
            case 0x7B:
                LD(A, E);
                cycles += 4;
                break; // LD A, E
            case 0x7C:
                LD(A, H);
                cycles += 4;
                break; // LD A, H
            case 0x7D:
                LD(A, L);
                cycles += 4;
                break; // LD A, L
            case 0x7F:
                LD(A, A);
                cycles += 4;
                break; // LD A, A

            case 0x50:
                LD(D, B);
                cycles += 4;
                break; // LD D, B
            case 0x51:
                LD(D, C);
                cycles += 4;
                break; // LD D, C
            case 0x52:
                LD(D, D);
                cycles += 4;
                break; // LD D, D
            case 0x53:
                LD(D, E);
                cycles += 4;
                break; // LD D, E
            case 0x54:
                LD(D, H);
                cycles += 4;
                break; // LD D, H
            case 0x55:
                LD(D, L);
                cycles += 4;
                break; // LD D, L

            case 0x58:
                LD(E, B);
                cycles += 4;
                break; // LD E, B
            case 0x59:
                LD(E, C);
                cycles += 4;
                break; // LD E, C
            case 0x5A:
                LD(E, D);
                cycles += 4;
                break; // LD E, D
            case 0x5B:
                LD(E, E);
                cycles += 4;
                break; // LD E, E
            case 0x5C:
                LD(E, H);
                cycles += 4;
                break; // LD E, H
            case 0x5D:
                LD(E, L);
                cycles += 4;
                break; // LD E, L

            case 0x60:
                LD(H, B);
                cycles += 4;
                break; // LD H, B
            case 0x61:
                LD(H, C);
                cycles += 4;
                break; // LD H, C
            case 0x62:
                LD(H, D);
                cycles += 4;
                break; // LD H, D
            case 0x63:
                LD(H, E);
                cycles += 4;
                break; // LD H, E
            case 0x64:
                LD(H, H);
                cycles += 4;
                break; // LD H, H
            case 0x65:
                LD(H, L);
                cycles += 4;
                break; // LD H, L

            case 0x68:
                LD(L, B);
                cycles += 4;
                break; // LD L, B
            case 0x69:
                LD(L, C);
                cycles += 4;
                break; // LD L, C
            case 0x6A:
                LD(L, D);
                cycles += 4;
                break; // LD L, D
            case 0x6B:
                LD(L, E);
                cycles += 4;
                break; // LD L, E
            case 0x6C:
                LD(L, H);
                cycles += 4;
                break; // LD L, H
            case 0x6D:
                LD(L, L);
                cycles += 4;
                break; // LD L, L

            case 0x46: // LD B, (HL)
                LD(B, z80_mem_read((H << 8) | L));
                cycles += 7;
                break;

            case 0x4E: // LD C, (HL)
                LD(C, z80_mem_read((H << 8) | L));
                cycles += 7;
                break;

            case 0x56: // LD D, (HL)
                LD(D, z80_mem_read((H << 8) | L));
                cycles += 7;
                break;

            case 0x5E: // LD E, (HL)
                LD(E, z80_mem_read((H << 8) | L));
                cycles += 7;
                break;

            case 0x66: // LD H, (HL)
                LD(H, z80_mem_read((H << 8) | L));
                cycles += 7;
                break;

            case 0x6E: // LD L, (HL)
                LD(L, z80_mem_read((H << 8) | L));
                cycles += 7;
                break;

            case 0x7E: // LD A, (HL)
                LD(A, z80_mem_read((H << 8) | L));
                cycles += 7;
                break;

            case 0x36: // LD (HL), n
            {
                uint8_t value = fetch();            // Fetch the immediate value (n)
                uint16_t hl_address = (H << 8) | L; // Compute the memory address pointed by HL
                z80_mem_write(hl_address, value);   // Write the value to memory
                cycles += 10;                       // Add 10 cycles for this instruction
                break;
            }

            case 0x02: // LD (BC), A
            {
                uint16_t bc_address = (B << 8) | C; // Combine B and C to form the address
                z80_mem_write(bc_address, A);       // Write the value in A to the memory location
                cycles += 7;                        // Add 7 cycles for this instruction
                break;
            }

            case 0x12: // LD (DE), A
            {
                uint16_t de_address = (D << 8) | E; // Combine D and E to form the address
                z80_mem_write(de_address, A);       // Write the value in A to the memory location
                cycles += 7;                        // Add 7 cycles for this instruction
                break;
            }

            case 0x0A: // LD A, (BC)
            {
                uint16_t bc_address = (B << 8) | C; // Combine B and C to form the address
                A = z80_mem_read(bc_address);       // Load the value from memory into A
                cycles += 7;                        // Add 7 cycles for this instruction
                break;
            }

            case 0x1A: // LD A, (DE)
            {
                uint16_t de_address = (D << 8) | E; // Combine D and E to form the 16-bit address
                A = z80_mem_read(de_address);       // Load the value from memory into A
                cycles += 7;                        // Add 7 cycles for this instruction
                break;
            }

            case 0x32: // LD (nn), A
            {
                uint8_t low = fetch();                // Fetch the low byte of the 16-bit address
                uint8_t high = fetch();               // Fetch the high byte of the 16-bit address
                uint16_t address = (high << 8) | low; // Combine the bytes to form the full 16-bit address
                z80_mem_write(address, A);            // Write the value of A to the computed memory location
                cycles += 13;                         // This instruction takes 13 T-states
                break;
            }

            case 0x01: // LD BC, nn
            {
                uint8_t low = fetch();  // Fetch the low byte
                uint8_t high = fetch(); // Fetch the high byte
                B = high;               // Set the high byte of BC
                C = low;                // Set the low byte of BC
                cycles += 10;           // Instruction takes 10 T-states
                break;
            }
            case 0x11: // LD DE, nn
            {
                uint8_t low = fetch();
                uint8_t high = fetch();
                D = high;
                E = low;
                cycles += 10;
                break;
            }
            case 0x21: // LD HL, nn
            {
                uint8_t low = fetch();
                uint8_t high = fetch();
                H = high;
                L = low;
                cycles += 10;
                break;
            }
            case 0x31: // LD SP, nn
            {
                uint8_t low = fetch();
                uint8_t high = fetch();
                SP = (high << 8) | low; // Combine the two bytes into a 16-bit value
                cycles += 10;
                break;
            }

            case 0x22: // LD (nn), HL
            {
                uint16_t address = fetch() | (fetch() << 8); // Fetch 16-bit memory address (nn)
                z80_mem_write(address, L);                   // Write low byte (L) to memory
                z80_mem_write(address + 1, H);               // Write high byte (H) to memory
                cycles += 20;                                // Instruction takes 20 cycles
                break;
            }

            case 0xF9: // LD SP, HL
            {
                SP = (H << 8) | L; // Combine H and L to form the 16-bit value
                cycles += 6;       // Instruction takes 6 cycles
                break;
            }

            case 0x86: // ADD A, (HL)
            {
                uint8_t value = z80_mem_read((H << 8) | L); // Read the memory at address HL
                uint16_t result = A + value;                // Perform the addition

                // Update the flags
                clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Clear all flags first
                if ((result & 0xFF) == 0)
                    setFlag(FLAG_Z); // Zero flag
                if (result & 0x80)
                    setFlag(FLAG_S); // Sign flag
                if (((A & 0xF) + (value & 0xF)) & 0x10)
                    setFlag(FLAG_H); // Half-carry flag
                if (((A ^ value ^ result) & 0x80) == 0x80)
                    setFlag(FLAG_PV); // Overflow flag
                if (result > 0xFF)
                    setFlag(FLAG_C); // Carry flag

                A = result & 0xFF; // Store the lower 8 bits back into A
                cycles += 7;       // Add cycles for the instruction
                break;
            }

            case 0xC6: // ADD A, n
            {
                uint8_t value = fetch(); // Fetch the immediate 8-bit value
                A = add_flag(A, value);  // Perform addition and update flags
                cycles += 7;             // This instruction takes 7 cycles
                break;
            }

            case 0x80: // ADD A, B
                A = add_flag(A, B);
                cycles += 4;
                break;

            case 0x81: // ADD A, C
                A = add_flag(A, C);
                cycles += 4;
                break;

            case 0x82: // ADD A, D
                A = add_flag(A, D);
                cycles += 4;
                break;

            case 0x83: // ADD A, E
                A = add_flag(A, E);
                cycles += 4;
                break;

            case 0x84: // ADD A, H
                A = add_flag(A, H);
                cycles += 4;
                break;

            case 0x85: // ADD A, L
                A = add_flag(A, L);
                cycles += 4;
                break;

            case 0x87: // ADD A, A
                A = add_flag(A, A);
                cycles += 4;
                break;

            case 0x09: // ADD HL, BC
                setHL(add_flag16(HL(), BC()));
                cycles += 11;
                break;

            case 0x19: // ADD HL, DE
                setHL(add_flag16(HL(), DE()));
                cycles += 11;
                break;

            case 0x29: // ADD HL, HL
                setHL(add_flag16(HL(), HL()));
                cycles += 11;
                break;

            case 0x39: // ADD HL, SP
                setHL(add_flag16(HL(), SP));
                cycles += 11;
                break;

            case 0x37:             // SCF (Set Carry Flag)
                setFlag(FLAG_C);   // Set the Carry flag
                clearFlag(FLAG_H); // Reset the Half-Carry flag
                clearFlag(FLAG_N); // Reset the Subtract flag
                // Note: Other flags remain unaffected
                cycles += 4; // 4 T-states
                break;

            case 0x88: // ADC A, B
                A = adc_flag(A, B);
                cycles += 4;
                break;

            case 0x89: // ADC A, C
                A = adc_flag(A, C);
                cycles += 4;
                break;

            case 0x8A: // ADC A, D
                A = adc_flag(A, D);
                cycles += 4;
                break;

            case 0x8B: // ADC A, E
                A = adc_flag(A, E);
                cycles += 4;
                break;

            case 0x8C: // ADC A, H
                A = adc_flag(A, H);
                cycles += 4;
                break;

            case 0x8D: // ADC A, L
                A = adc_flag(A, L);
                cycles += 4;
                break;

            case 0x8E: // ADC A, (HL)
                A = adc_flag(A, z80_mem_read(HL()));
                cycles += 7; // 7 T-states for this operation
                break;

            case 0x8F: // ADC A, A
                A = adc_flag(A, A);
                cycles += 4;
                break;

            case 0xCE: // ADC A, n
                A = adc_flag(A, fetch());
                cycles += 7;
                break;

            case 0x90: // SUB B
                A = sub_flag(A, B);
                cycles += 4;
                break;

            case 0x91: // SUB C
                A = sub_flag(A, C);
                cycles += 4;
                break;

            case 0x92: // SUB D
                A = sub_flag(A, D);
                cycles += 4;
                break;

            case 0x93: // SUB E
                A = sub_flag(A, E);
                cycles += 4;
                break;

            case 0x94: // SUB H
                A = sub_flag(A, H);
                cycles += 4;
                break;

            case 0x95: // SUB L
                A = sub_flag(A, L);
                cycles += 4;
                break;

            case 0x96: // SUB (HL)
                A = sub_flag(A, z80_mem_read(HL()));
                cycles += 7; // Memory access adds more cycles
                break;

            case 0x97: // SUB A
                A = sub_flag(A, A);
                cycles += 4;
                break;

            case 0xD6: // SUB n
                A = sub_flag(A, fetch());
                cycles += 7; // Immediate value adds more cycles
                break;

            case 0x98: // SBC A, B
                A = sbc_flag(A, B);
                cycles += 4;
                break;

            case 0x99: // SBC A, C
                A = sbc_flag(A, C);
                cycles += 4;
                break;

            case 0x9A: // SBC A, D
                A = sbc_flag(A, D);
                cycles += 4;
                break;

            case 0x9B: // SBC A, E
                A = sbc_flag(A, E);
                cycles += 4;
                break;

            case 0x9C: // SBC A, H
                A = sbc_flag(A, H);
                cycles += 4;
                break;

            case 0x9D: // SBC A, L
                A = sbc_flag(A, L);
                cycles += 4;
                break;

            case 0x9E: // SBC A, (HL)
                A = sbc_flag(A, z80_mem_read(HL()));
                cycles += 7;
                break;

            case 0x9F: // SBC A, A
                A = sbc_flag(A, A);
                cycles += 4;
                break;

            case 0xB0:
                OR(B);
                cycles += 4;
                break; // OR B
            case 0xB1:
                OR(C);
                cycles += 4;
                break; // OR C
            case 0xB2:
                OR(D);
                cycles += 4;
                break; // OR D
            case 0xB3:
                OR(E);
                cycles += 4;
                break; // OR E
            case 0xB4:
                OR(H);
                cycles += 4;
                break; // OR H
            case 0xB5:
                OR(L);
                cycles += 4;
                break; // OR L
            case 0xB6:
                OR(z80_mem_read(HL()));
                cycles += 7;
                break; // OR (HL)
            case 0xB7:
                OR(A);
                cycles += 4;
                break; // OR A
            case 0xF6:
                OR(fetch());
                cycles += 7;
                break; // OR n

            case 0xC5:                // PUSH BC
                SP--;                 // Decrement SP for the high-order byte
                z80_mem_write(SP, B); // Store the high-order byte of BC (B) at the new SP location
                SP--;                 // Decrement SP for the low-order byte
                z80_mem_write(SP, C); // Store the low-order byte of BC (C) at the new SP location
                cycles += 11;         // Update cycle count (5, 3, 3 T-states)
                break;

            case 0xD5: // PUSH DE
                SP--;
                z80_mem_write(SP, D); // Store the high-order byte of DE (D)
                SP--;
                z80_mem_write(SP, E); // Store the low-order byte of DE (E)
                cycles += 11;
                break;

            case 0xE5: // PUSH HL
                SP--;
                z80_mem_write(SP, H); // Store the high-order byte of HL (H)
                SP--;
                z80_mem_write(SP, L); // Store the low-order byte of HL (L)
                cycles += 11;
                break;

            case 0xF5: // PUSH AF
                SP--;
                z80_mem_write(SP, A); // Store the high-order byte of AF (A)
                SP--;
                z80_mem_write(SP, F); // Store the low-order byte of AF (F)
                cycles += 11;
                break;

            case 0xC1:                  // POP BC
                C = z80_mem_read(SP++); // Load low byte into C
                B = z80_mem_read(SP++); // Load high byte into B
                cycles += 10;           // 10 T-states for POP BC
                break;

            case 0xD1:                  // POP DE
                E = z80_mem_read(SP++); // Load low byte into E
                D = z80_mem_read(SP++); // Load high byte into D
                cycles += 10;           // 10 T-states for POP DE
                break;

            case 0xE1:                  // POP HL
                L = z80_mem_read(SP++); // Load low byte into L
                H = z80_mem_read(SP++); // Load high byte into H
                cycles += 10;           // 10 T-states for POP HL
                break;

            case 0xF1:                  // POP AF
                F = z80_mem_read(SP++); // Load low byte into F (Flags)
                A = z80_mem_read(SP++); // Load high byte into A (Accumulator)
                cycles += 10;           // 10 T-states for POP AF
                break;

            case 0xC9: // RET
            {
                uint8_t low = z80_mem_read(SP); // Fetch the low byte and increment SP
                SP++;
                uint8_t high = z80_mem_read(SP); // Fetch the high byte and increment SP
                SP++;
                PC = (uint16_t)(high << 8) | low; // Combine high and low into a 16-bit address
                cycles += 10;
                break;
            }

            case 0xC0: // RET NZ
            {
                if (!isFlagSet(FLAG_Z))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }
            case 0xC8: // RET Z
            {
                if (isFlagSet(FLAG_Z))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }
            case 0xD0: // RET NC
            {
                if (!isFlagSet(FLAG_C))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }
            case 0xD8: // RET C
            {
                if (isFlagSet(FLAG_C))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }
            case 0xE0: // RET PO
            {
                if (!isFlagSet(FLAG_PV))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }
            case 0xE8: // RET PE
            {
                if (isFlagSet(FLAG_PV))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }
            case 0xF0: // RET P
            {
                if (!isFlagSet(FLAG_S))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }
            case 0xF8: // RET M
            {
                if (isFlagSet(FLAG_S))
                {
                    RET_Implementation();
                    cycles += 11;
                    break;
                }
                cycles += 5;
                break;
            }

            case 0x17:
            {                // RLA opcode
                rla();       // Call the RLA function
                cycles += 4; // RLA takes 4 cycles
                break;
            }

            case 0x1F:
            {                // RRA opcode
                rra();       // Call the RRA function
                cycles += 4; // RLA takes 4 cycles
                break;
            }

            case 0x07: // RLC A
                rlc(A);
                cycles += 4;
                break;

            case 0x0F: // RRC A
                rrc(A);
                cycles += 8;
                break;

            case 0xC7:
            case 0xCF:
            case 0xD7:
            case 0xDF:
            case 0xE7:
            case 0xEF:
            case 0xF7:
            case 0xFF:
                rst(opcode);
                break;

            case 0xA8: xorReg(B); cycles += 4; break; // XOR B
            case 0xA9: xorReg(C); cycles += 4; break; // XOR C
            case 0xAA: xorReg(D); cycles += 4; break; // XOR D
            case 0xAB: xorReg(E); cycles += 4; break; // XOR E
            case 0xAC: xorReg(H); cycles += 4; break; // XOR H
            case 0xAD: xorReg(L); cycles += 4;  break; // XOR L
            case 0xAE: xorReg(z80_mem_read(HL())); cycles += 7; break; // XOR (HL)
            case 0xAF: xorReg(A); cycles += 4; break; // XOR A

            case 0xEE: // XOR n
            {
                uint8_t value = fetch(); // Fetch the immediate value
                xorReg(value); cycles += 7; break;
            }

            case 0xA0: and_r(B); cycles += 4; break;     // AND B
            case 0xA1: and_r(C); cycles += 4; break;     // AND C
            case 0xA2: and_r(D); cycles += 4; break;     // AND D
            case 0xA3: and_r(E); cycles += 4; break;     // AND E
            case 0xA4: and_r(H); cycles += 4; break;     // AND H
            case 0xA5: and_r(L); cycles += 4; break;     // AND L
            case 0xA6: and_hl(); break;     // AND (HL)
            case 0xA7: and_r(A); cycles += 4; break;     // AND A
            case 0xE6: and_n(); break;      // AND n


            case 0xC2: // JP NZ, nn
            case 0xCA: // JP Z, nn
            case 0xD2: // JP NC, nn
            case 0xDA: // JP C, nn
            case 0xE2: // JP PO, nn
            case 0xEA: // JP PE, nn
            case 0xF2: // JP P, nn
            case 0xFA: // JP M, nn
            {
                // Fetch the 16-bit address (nn)
                uint16_t address = fetch16();

                // Determine the condition code (cc) from the opcode
                uint8_t conditionCode = (opcode >> 3) & 0x07;
                bool conditionMet = false;

                // Evaluate the condition
                switch (conditionCode)
                {
                    case 0: conditionMet = !isFlagSet(FLAG_Z); break; // NZ: Non-zero
                    case 1: conditionMet = isFlagSet(FLAG_Z); break;  // Z: Zero
                    case 2: conditionMet = !isFlagSet(FLAG_C); break; // NC: No Carry
                    case 3: conditionMet = isFlagSet(FLAG_C); break;  // C: Carry
                    case 4: conditionMet = !isFlagSet(FLAG_PV); break; // PO: Parity Odd
                    case 5: conditionMet = isFlagSet(FLAG_PV); break;  // PE: Parity Even
                    case 6: conditionMet = !isFlagSet(FLAG_S); break; // P: Sign Positive
                    case 7: conditionMet = isFlagSet(FLAG_S); break;  // M: Sign Negative
                    default:
                    {
                        cerr << "Unimplemented opcode Jump CC, NN:  " << endl;
                        break;
                    }
                }

                // If condition is met, set PC to the fetched address
                if (conditionMet)
                {
                    PC = address;
                    // JP cc, nn takes 10 T-states if the jump is taken
                    cycles += 10;
                }
                else
                {
                    // JP cc, nn takes 10 T-states if the jump is not taken
                    cycles += 10;
                }

                break;
            }


            case 0xC4: // CALL NZ, nn
            case 0xCC: // CALL Z, nn
            case 0xD4: // CALL NC, nn
            case 0xDC: // CALL C, nn
            case 0xE4: // CALL PO, nn
            case 0xEC: // CALL PE, nn
            case 0xF4: // CALL P, nn
            case 0xFC: // CALL M, nn
            {
                // Determine the condition based on opcode
                uint8_t condition = (opcode >> 3) & 0x07;
                uint16_t address = fetch16(); // Fetch the 16-bit target address

                bool shouldCall = false;
                switch (condition)
                {
                    case 0x00: shouldCall = !isFlagSet(FLAG_Z); break; // NZ
                    case 0x01: shouldCall = isFlagSet(FLAG_Z); break;  // Z
                    case 0x02: shouldCall = !isFlagSet(FLAG_C); break; // NC
                    case 0x03: shouldCall = isFlagSet(FLAG_C); break;  // C
                    case 0x04: shouldCall = !isFlagSet(FLAG_PV); break; // PO
                    case 0x05: shouldCall = isFlagSet(FLAG_PV); break;  // PE
                    case 0x06: shouldCall = !isFlagSet(FLAG_S); break;  // P
                    case 0x07: shouldCall = isFlagSet(FLAG_S); break;   // M
                    default:
                    {
                        cerr << "Unimplemented opcode Call CC, NN:  " << endl;
                        break;
                    }
                }

                if (shouldCall)
                {
                    // Push the current PC onto the stack
                    SP -= 1;
                    z80_mem_write(SP, (PC >> 8) & 0xFF); // Push high byte
                    SP -= 1;
                    z80_mem_write(SP, PC & 0xFF);       // Push low byte

                    // Jump to the target address
                    PC = address;

                    // CALL with condition met takes 17 T-states
                    cycles += 17;
                }
                else
                {
                    // If condition is false, just increment PC
                    // CALL with condition false takes 10 T-states
                    cycles += 10;
                }
                break;
            }

            case 0xCD: // CALL nn
                call_nn();
                break;

            case 0x3F: // CCF
                ccf();
                cycles += 4;
                break;

            case 0xB8: cp(B); cycles += 4; break; // CP B
            case 0xB9: cp(C); cycles += 4; break; // CP C
            case 0xBA: cp(D); cycles += 4; break; // CP D
            case 0xBB: cp(E); cycles += 4; break; // CP E
            case 0xBC: cp(H); cycles += 4; break; // CP H
            case 0xBD: cp(L); cycles += 4; break; // CP L
            case 0xBF: cp(A); cycles += 4; break; // CP A
            case 0xBE: // CP (HL)
                cp(z80_mem_read(HL()));
                cycles += 7;
                break;
            case 0xFE: // CP n
                cycles += 7;
                cp(fetch());
                break;

            case 0x2F: // CPL
            {
                // Invert all bits in the accumulator (one's complement)
                A = ~A;

                // Set the H (half-carry) and N (subtract) flags
                setFlag(FLAG_H);
                setFlag(FLAG_N);

                // S, Z, P/V, and C flags are not affected
                // Increment the cycle count
                cycles += 4;

                break;
            }

            case 0x27: // DAA
            {
                uint8_t correction = 0;
                uint8_t initialA = A;
                bool carrySet = false;

                // Adjust lower nibble if required
                if (isFlagSet(FLAG_H) || (A & 0x0F) > 9)
                {
                    correction |= 0x06;
                }

                // Adjust upper nibble if required
                if (isFlagSet(FLAG_C) || A > 0x99)
                {
                    correction |= 0x60;
                    carrySet = true;
                }

                // Apply correction based on the N (subtract) flag
                if (isFlagSet(FLAG_N))
                {
                    A -= correction;
                }
                else
                {
                    A += correction;
                }

                // Update flags
                clearFlag(FLAG_H); // H is undefined after DAA
                if (carrySet)
                {
                    setFlag(FLAG_C); // Set carry if adjustment required overflow
                }
                else
                {
                    clearFlag(FLAG_C);
                }

                if (A == 0)
                {
                    setFlag(FLAG_Z); // Set Z if result is zero
                }
                else
                {
                    clearFlag(FLAG_Z);
                }

                if (A & 0x80)
                {
                    setFlag(FLAG_S); // Set S if result is negative
                }
                else
                {
                    clearFlag(FLAG_S);
                }

                // Update P/V based on parity
                if (checkParity(A))
                {
                    clearFlag(FLAG_PV); // Clear P/V for odd parity
                }
                else
                {
                    setFlag(FLAG_PV); // Set P/V for even parity
                }

                // DAA takes 4 T-states
                cycles += 4;

                break;
            }

            case 0x3D: // DEC A
            {
                zdec(A); // Decrement register A
                cycles += 4;
                break;
            }

            case 0x05: // DEC B
            {
                zdec(B);      // Decrement register B
                cycles += 4; // DEC takes 4 T-states
                break;
            }

            case 0x0D: // DEC C
            {
                zdec(C); // Decrement register C
                cycles += 4;
                break;
            }

            case 0x15: // DEC D
            {
                zdec(D); // Decrement register D
                cycles += 4;
                break;
            }

            case 0x1D: // DEC E
            {
                zdec(E); // Decrement register E
                cycles += 4;
                break;
            }

            case 0x25: // DEC H
            {
                zdec(H); // Decrement register H
                cycles += 4;
                break;
            }

            case 0x2D: // DEC L
            {
                zdec(L); // Decrement register L
                cycles += 4;
                break;
            }

            case 0x35: // DEC (HL)
            {
                // Read the value from memory at HL
                uint8_t value = z80_mem_read(HL());

                // Perform decrement
                zdec(value);

                // Write the updated value back to memory
                z80_mem_write(HL(), value);

                cycles += 11; // DEC (HL) takes 11 T-states
                break;
            }

            case 0x3C: // INC A
            {
                zinc(A); // increment register A
                cycles += 4;
                break;
            }

            case 0x04: // INC B
            {
                zinc(B);     // increment register B
                cycles += 4; // INC takes 4 T-states
                break;
            }

            case 0x0C: // INC C
            {
                zinc(C); // increment register C
                cycles += 4;
                break;
            }

            case 0x14: // INC D
            {
                zinc(D); // increment register D
                cycles += 4;
                break;
            }

            case 0x1C: // INC E
            {
                zinc(E); // increment register E
                cycles += 4;
                break;
            }

            case 0x24: // INC H
            {
                zinc(H); // increment register H
                cycles += 4;
                break;
            }

            case 0x2C: // INC L
            {
                zinc(L); // increment register L
                cycles += 4;
                break;
            }

            case 0x34: // INC (HL)
            {
                // Read the value from memory at HL
                uint8_t value = z80_mem_read(HL());

                // Perform increment
                zinc(value);

                // Write the updated value back to memory
                z80_mem_write(HL(), value);

                cycles += 11; // INC (HL) takes 11 T-states
                break;
            }

            case 0x0B: // DEC BC
            {
                uint16_t value = BC();
                setBC(--value);
                cycles += 6;
                break;
            }

            case 0x1B: // DEC DE
            {
                uint16_t value = DE();
                setDE(--value);
                cycles += 6;
                break;
            }
            case 0x2B: // DEC HL
            {
                uint16_t value = HL();
                setHL(--value);
                cycles += 6;
                break;
            }
            case 0x3B: // DEC SP
            {
                SP--;
                cycles += 6;
                break;
            }

            case 0x03: // INC BC
            {
                uint16_t value = BC();
                setBC(++value);
                cycles += 6;
                break;
            }

            case 0x13: // INC DE
            {
                uint16_t value = DE();
                setDE(++value);
                cycles += 6;
                break;
            }
            case 0x23: // INC HL
            {
                uint16_t value = HL();
                setHL(++value);
                cycles += 6;
                break;
            }
            case 0x33: // INC SP
            {
                SP++;
                cycles += 6;
                break;
            }

            case 0xF3: // DI (Disable Interrupts)
            {
                // Disable maskable interrupts by resetting IFF1 and IFF2
                IFF1 = false;
                IFF2 = false;

                // DI takes 4 T-states
                cycles += 4;

                break;
            }

            case 0xFB: // EI (Enable Interrupts)
            {
                // Enable maskable interrupts by resetting IFF1 and IFF2
                IFF1 = true;
                IFF2 = true;

                // EI takes 4 T-states
                cycles += 4;

                break;
            }

            case 0x10: // DJNZ, e
            {
                // Fetch the displacement value (signed 8-bit)
                int8_t displacement = static_cast<int8_t>(fetch());

                // Decrement the B register
                B--;

                if (B != 0)
                {
                    // If B is not zero, adjust the PC by the displacement
                    PC += displacement;

                    // DJNZ takes 13 T-states if branching
                    cycles += 13;
                }
                else
                {
                    // If B is zero, continue to the next instruction
                    // DJNZ takes 8 T-states if not branching
                    cycles += 8;
                }

                break;
            }

            case 0xE3: // EX (SP), HL
            {
                // Retrieve values from memory at SP and SP+1
                uint8_t lowSP = z80_mem_read(SP);
                uint8_t highSP = z80_mem_read(SP + 1);

                // Store the original HL values
                uint8_t lowHL = HL() & 0xFF;
                uint8_t highHL = (HL() >> 8) & 0xFF;

                // Exchange the values
                z80_mem_write(SP, lowHL);      // Write HL low byte to SP
                z80_mem_write(SP + 1, highHL); // Write HL high byte to SP+1
                setHL((highSP << 8) | lowSP);    // Load SP values into HL

                // Instruction timing: 19 T-states
                cycles += 19;

                break;
            }

            case 0x08: // EX AF, AF'
            {
                // Swap the accumulator and flags with their shadow versions
                std::swap(A, A_);
                std::swap(F, F_);

                // Instruction timing: 4 T-states
                cycles += 4;

                break;
            }

            case 0xEB: // EX DE, HL
            {
                // Temporarily store the value of DE
                uint16_t temp = DE();

                // Swap DE and HL using the provided setters
                setDE(HL());
                setHL(temp);

                // EX DE, HL takes 4 T-states
                cycles += 4;

                break;
            }

            case 0xD9: // EXX
            {
                // Temporarily store the current values of BC, DE, and HL
                uint16_t tempBC = BC();
                uint16_t tempDE = DE();
                uint16_t tempHL = HL();

                // Swap the register pairs with their alternate sets
                setBC(BC_());
                setDE(DE_());
                setHL(HL_());

                // Set the alternate register pairs to the temporarily stored values
                setBC_(tempBC);
                setDE_(tempDE);
                setHL_(tempHL);

                // EXX takes 4 T-states
                cycles += 4;

                break;
            }

            case 0xE9: // JP (HL)
            {
                // Set the program counter (PC) to the value of HL
                PC = HL();

                // JP (HL) takes 4 T-states
                cycles += 4;

                break;
            }

            case 0xC3: // JP nn
            {
                // Fetch the 16-bit address (nn)
                uint16_t address = fetch16();

                // Load the address into the Program Counter (PC)
                PC = address;

                // JP nn takes 10 T-states
                cycles += 10;

                break;
            }

            case 0x30: // JR NC, e
            {
                // Fetch the relative offset
                int8_t offset = static_cast<int8_t>(fetch());

                // Check the condition (NC: No Carry, FLAG_C = 0)
                if (!isFlagSet(FLAG_C))
                {
                    // Add the signed offset to the program counter (PC)
                    PC += offset;

                    // JR with condition met takes 12 T-states
                    cycles += 12;
                }
                else
                {
                    // Condition not met; no jump, just increment PC
                    // JR with condition not met takes 7 T-states
                    cycles += 7;
                }

                break;
            }

            case 0x20: // JR NZ, e
            {
                // Fetch the relative offset
                int8_t offset = static_cast<int8_t>(fetch());

                // Check the condition (NZ: No Zero, FLAG_Z = 0)
                if (!isFlagSet(FLAG_Z))
                {
                    // Add the signed offset to the program counter (PC)
                    PC += offset;

                    // JR with condition met takes 12 T-states
                    cycles += 12;
                }
                else
                {
                    // Condition not met; no jump, just increment PC
                    // JR with condition not met takes 7 T-states
                    cycles += 7;
                }

                break;
            }

            case 0x28: // JR Z, e
            {
                // Fetch the relative offset
                int8_t offset = static_cast<int8_t>(fetch());

                // Check the condition (NZ: Zero, FLAG_Z = 1)
                if (isFlagSet(FLAG_Z))
                {
                    // Add the signed offset to the program counter (PC)
                    PC += offset;

                    // JR with condition met takes 12 T-states
                    cycles += 12;
                }
                else
                {
                    // Condition not met; no jump, just increment PC
                    // JR with condition not met takes 7 T-states
                    cycles += 7;
                }

                break;
            }

            case 0x38: // JR C, e
            {
                // Fetch the relative offset
                int8_t offset = static_cast<int8_t>(fetch());

                // Check the condition (NC: No Carry, FLAG_C = 0)
                if (isFlagSet(FLAG_C))
                {
                    // Add the signed offset to the program counter (PC)
                    PC += offset;

                    // JR with condition met takes 12 T-states
                    cycles += 12;
                }
                else
                {
                    // Condition not met; no jump, just increment PC
                    // JR with condition not met takes 7 T-states
                    cycles += 7;
                }

                break;
            }

            case 0x18: // JR n
            {
                // Fetch the relative offset
                int8_t offset = static_cast<int8_t>(fetch());

                    PC += offset;

                    // JR with condition met takes 12 T-states
                    cycles += 12;


                break;
            }

            case 0xDB: // IN A, (N )
            {
                int8_t offset = static_cast<int8_t>(fetch());
                cycles += 11;
                break;
            }

            case 0xD3: // IN  (N), A
            {
                int8_t offset = static_cast<int8_t>(fetch());
                cycles += 11;
                break;
            }

                // Handle ED-prefixed instructions
            case 0xED:
            {
                uint8_t subOpcode = fetch(); // Fetch the next byte
                switch (subOpcode)
                {
                    case 0x76: // sleep
                    {
                        printState();
                        halt = true;
                        z80_mem_dump("memory.bin");
                        cycles += 4;
                        break;
                    }

                    case 0x00:
                    {
                        printState();
                        break;
                    }

                    case 0x01:
                    {
                        z80_mem_dump("memory.bin");
                        break;
                    }

                    case 0x78:  // IN A, (C)
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x40: // IN B, (C)
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x48: // IN C, (C)
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x50: // IN D, (C)
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x58: // IN E, (C)
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x60: // IN H, (C)
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x68: // IN L, (C)
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x79: // OUT (C), A
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x41: // OUT (C), B
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x49: // OUT (C), C
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x51: // OUT (C), D
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x59: // OUT (C), E
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x61: // OUT (C), H
                    {
                        cycles += 12;
                        break;
                    }

                    case 0x69: // OUT (C), L
                    {
                        cycles += 12;
                        break;
                    }

                    case 0xAA: // IND
                    {
                        cycles += 16;
                        break;
                    }

                    case 0xBA: // INDR
                    {
                        cycles += 16;
                        break;
                    }

                    case 0xA2: // INI
                    {
                        cycles += 16;
                        break;
                    }

                    case 0xB2: // INIR
                    {
                        cycles += 16;
                        break;
                    }

                    case 0xAB: // OUTD
                    {
                        cycles += 16;
                        break;
                    }

                    case 0xA3: // OUTI
                    {
                        cycles += 16;
                        break;
                    }

                    case 0xBB: // OTDR
                    {
                        cycles += 16;
                        break;
                    }

                    case 0xB3: // OTIR
                    {
                        cycles += 16;
                        break;
                    }

                    case 0x47: // LD I, A
                        I = A; // Load A into I register
                        cycles += 9;
                        break;

                    case 0x57: // LD A, I
                        A = I;
                        clearFlag(FLAG_H | FLAG_N); // Reset H and N
                        if (I & 0x80)
                            setFlag(FLAG_S);
                        else
                            clearFlag(FLAG_S); // Set S if bit 7 of I is 1
                        if (I == 0)
                            setFlag(FLAG_Z);
                        else
                            clearFlag(FLAG_Z); // Set Z if I is zero
                        if (IFF2)
                            setFlag(FLAG_PV);
                        else
                            clearFlag(FLAG_PV); // Set P/V based on IFF2
                        cycles += 9;
                        break;

                    case 0x5F:                      // LD A, R
                        A = R;                      // Load R into A register
                        clearFlag(FLAG_H | FLAG_N); // Reset H and N
                        if (R & 0x80)
                            setFlag(FLAG_S);
                        else
                            clearFlag(FLAG_S); // Set S if bit 7 of R is 1
                        if (R == 0)
                            setFlag(FLAG_Z);
                        else
                            clearFlag(FLAG_Z); // Set Z if R is zero
                        if (IFF2)
                            setFlag(FLAG_PV);
                        else
                            clearFlag(FLAG_PV); // Set P/V based on IFF2
                        cycles += 9;
                        break;

                    case 0x4F: // LD R, A
                    {
                        R = A;       // Load the contents of the accumulator into the refresh register
                        cycles += 9; // Add 9 T states for this instruction
                        break;
                    }

                    case 0x43: // LD (nn), BC
                    case 0x53: // LD (nn), DE
                    case 0x63: // LD (nn), HL
                    case 0x73: // LD (nn), SP
                    {
                        uint8_t low = fetch();                // Fetch the low byte of the 16-bit address
                        uint8_t high = fetch();               // Fetch the high byte of the 16-bit address
                        uint16_t address = (high << 8) | low; // Combine to form the 16-bit address

                        // Determine the register pair to use
                        uint16_t value = 0;
                        switch (subOpcode)
                        {
                        case 0x43:
                            value = (B << 8) | C;
                            break; // BC
                        case 0x53:
                            value = (D << 8) | E;
                            break; // DE
                        case 0x63:
                            value = (H << 8) | L;
                            break; // HL
                        case 0x73:
                            value = SP;
                            break; // SP
                        }

                        // Write the register pair to memory
                        z80_mem_write(address, value & 0xFF);     // Write low byte
                        z80_mem_write(address + 1, (value >> 8)); // Write high byte

                        cycles += 20; // This instruction takes 20 T-states
                        break;
                    }
                    case 0x4B: // LD BC, (nn)
                    {
                        uint16_t address = fetch() | (fetch() << 8); // Fetch the 16-bit memory address (nn)
                        uint8_t low = z80_mem_read(address);         // Read low byte
                        uint8_t high = z80_mem_read(address + 1);    // Read high byte
                        C = low;                                     // Load into C (low byte)
                        B = high;                                    // Load into B (high byte)
                        cycles += 20;                                // Timing for this instruction
                        break;
                    }
                    case 0x5B: // LD DE, (nn)
                    {
                        uint16_t address = fetch() | (fetch() << 8); // Fetch the 16-bit memory address (nn)
                        uint8_t low = z80_mem_read(address);         // Read low byte
                        uint8_t high = z80_mem_read(address + 1);    // Read high byte
                        E = low;                                     // Load into E (low byte)
                        D = high;                                    // Load into D (high byte)
                        cycles += 20;                                // Timing for this instruction
                        break;
                    }
                    case 0x6B: // LD HL, (nn)
                    {
                        uint16_t address = fetch() | (fetch() << 8); // Fetch the 16-bit memory address (nn)
                        uint8_t low = z80_mem_read(address);         // Read low byte
                        uint8_t high = z80_mem_read(address + 1);    // Read high byte
                        L = low;                                     // Load into L (low byte)
                        H = high;                                    // Load into H (high byte)
                        cycles += 20;                                // Timing for this instruction
                        break;
                    }
                    case 0x7B: // LD SP, (nn)
                    {
                        uint16_t address = fetch() | (fetch() << 8); // Fetch the 16-bit memory address (nn)
                        uint8_t low = z80_mem_read(address);         // Read low byte
                        uint8_t high = z80_mem_read(address + 1);    // Read high byte
                        SP = (high << 8) | low;                      // Combine high and low bytes to form SP
                        cycles += 20;                                // Timing for this instruction
                        break;
                    }

                    case 0xA8: // LDD (ED-prefixed)
                    {
                        // Read the value from the memory address pointed by HL
                        uint8_t value = z80_mem_read((H << 8) | L);

                        // Write the value to the memory address pointed by DE
                        z80_mem_write((D << 8) | E, value);

                        // Decrement HL, DE, and BC
                        uint16_t hl = (H << 8) | L;
                        uint16_t de = (D << 8) | E;
                        uint16_t bc = (B << 8) | C;

                        hl--; // HL - 1
                        de--; // DE - 1
                        bc--; // BC - 1

                        // Update HL, DE, and BC registers
                        H = (hl >> 8) & 0xFF;
                        L = hl & 0xFF;
                        D = (de >> 8) & 0xFF;
                        E = de & 0xFF;
                        B = (bc >> 8) & 0xFF;
                        C = bc & 0xFF;

                        // Update Flags
                        clearFlag(FLAG_H | FLAG_N); // Reset H and N flags
                        if (bc != 0)
                            setFlag(FLAG_PV); // Set P/V if BC ≠ 0
                        else
                            clearFlag(FLAG_PV); // Reset P/V otherwise

                        // Timing for this instruction
                        cycles += 16;

                        break;
                    }

                    case 0xA0: // LDI (ED A0)
                    {
                        uint8_t value = z80_mem_read((H << 8) | L); // Read value from memory pointed by HL
                        z80_mem_write((D << 8) | E, value);         // Write the value to memory pointed by DE

                        // Increment HL and DE registers
                        uint16_t hl = (H << 8) | L;
                        uint16_t de = (D << 8) | E;
                        hl += 1;
                        de += 1;
                        H = (hl >> 8) & 0xFF;
                        L = hl & 0xFF;
                        D = (de >> 8) & 0xFF;
                        E = de & 0xFF;

                        // Decrement BC
                        uint16_t bc = (B << 8) | C;
                        bc -= 1;
                        B = (bc >> 8) & 0xFF;
                        C = bc & 0xFF;

                        // Set or reset the P/V flag based on BC
                        if (bc != 0)
                        {
                            setFlag(FLAG_PV); // Parity/Overflow set if BC ≠ 0
                        }
                        else
                        {
                            clearFlag(FLAG_PV); // Clear otherwise
                        }

                        clearFlag(FLAG_N); // Reset N flag
                        clearFlag(FLAG_H); // Reset H flag

                        cycles += 16; // Instruction takes 16 T-states
                        break;
                    }

                    case 0xB8: // LDDR
                    {
                        do
                        {
                            // 1. Copy the byte from (HL) to (DE)
                            uint8_t value = z80_mem_read((H << 8) | L);
                            z80_mem_write((D << 8) | E, value);

                            // 2. Decrement HL, DE, and BC
                            uint16_t hl = (H << 8) | L;
                            uint16_t de = (D << 8) | E;
                            uint16_t bc = (B << 8) | C;

                            hl--;
                            de--;
                            bc--;

                            H = (hl >> 8) & 0xFF;
                            L = hl & 0xFF;
                            D = (de >> 8) & 0xFF;
                            E = de & 0xFF;
                            B = (bc >> 8) & 0xFF;
                            C = bc & 0xFF;

                            // 3. Update flags
                            clearFlag(FLAG_H | FLAG_N); // H and N reset
                            if (bc != 0)
                                setFlag(FLAG_PV); // Set P/V if BC is non-zero
                            else
                                clearFlag(FLAG_PV);

                            cycles += 21; // Add cycle count

                        } while ((B << 8 | C) != 0); // Loop until BC == 0

                        break;
                    }

                    case 0xB0: // LDIR (ED B0)
                    {
                        do
                        {
                            uint8_t value = z80_mem_read((H << 8) | L); // Read value from memory pointed by HL
                            z80_mem_write((D << 8) | E, value);         // Write the value to memory pointed by DE

                            // Increment HL and DE registers
                            uint16_t hl = (H << 8) | L;
                            uint16_t de = (D << 8) | E;
                            hl += 1;
                            de += 1;
                            H = (hl >> 8) & 0xFF;
                            L = hl & 0xFF;
                            D = (de >> 8) & 0xFF;
                            E = de & 0xFF;

                            // Decrement BC
                            uint16_t bc = (B << 8) | C;
                            bc -= 1;
                            B = (bc >> 8) & 0xFF;
                            C = bc & 0xFF;

                            // Set or reset the P/V flag based on BC
                            if (bc != 0)
                            {
                                setFlag(FLAG_PV); // Parity/Overflow set if BC ≠ 0
                            }
                            else
                            {
                                clearFlag(FLAG_PV); // Clear otherwise
                            }

                            clearFlag(FLAG_N); // Reset N flag
                            clearFlag(FLAG_H); // Reset H flag

                            // Timing: 21 T-states for BC ≠ 0, 16 T-states for BC = 0
                            cycles += (bc != 0) ? 21 : 16;

                        } while (C != 0 || B != 0); // Repeat until BC = 0

                        break;
                    }
                    case 0x4A: // ADC HL, BC
                        setHL(adc_flag16(HL(), BC()));
                        cycles += 15; // 4, 4, 4, 3 T-states
                        break;

                    case 0x5A: // ADC HL, DE
                        setHL(adc_flag16(HL(), DE()));
                        cycles += 15; // 4, 4, 4, 3 T-states
                        break;

                    case 0x6A: // ADC HL, HL
                        setHL(adc_flag16(HL(), HL()));
                        cycles += 15; // 4, 4, 4, 3 T-states
                        break;

                    case 0x7A: // ADC HL, SP
                        setHL(adc_flag16(HL(), SP));
                        cycles += 15; // 4, 4, 4, 3 T-states
                        break;

                    case 0x42: // SBC HL, BC
                        setHL(sbc_flag16(HL(), BC()));
                        cycles += 15;
                        break;

                    case 0x52: // SBC HL, DE
                        setHL(sbc_flag16(HL(), DE()));
                        cycles += 15;
                        break;

                    case 0x62: // SBC HL, HL
                        setHL(sbc_flag16(HL(), HL()));
                        cycles += 15;
                        break;

                    case 0x72: // SBC HL, SP
                        setHL(sbc_flag16(HL(), SP));
                        cycles += 15;
                        break;

                    case 0x44: // NEG
                        NEG();
                        cycles += 8;
                        break;

                    case 0x45: // RETN
                        RET_Implementation();
                        cycles += 14; // Adjust cycles for RETN
                        break;

                    case 0x4D: // RETI
                        RET_Implementation();
                        cycles += 14; // Adjust cycles for RETI
                        break;

                    case 0x6F:
                    { // RLD opcode
                        rld();        // Call the RLD function
                        cycles += 18; // RLD takes 18 T-states
                        break;
                    }

                    case 0x67:
                    {                 // RRD opcode
                        rrd();        // Call the RRD function
                        cycles += 18; // RLD takes 18 T-states
                        break;
                    }

                    case 0xA9:
                    {
                        cpd();
                        break;
                    }

                    case 0xB9: // CPDR
                    {
                        do
                        {
                            // Read value from memory at HL
                            uint8_t value = z80_mem_read(HL());

                            // Perform subtraction for comparison (A - (HL))
                            uint8_t result = A - value;
                            // Update flags
                                // Set S (sign flag)
                            if (result & 0x80) {
                                setFlag(FLAG_S);
                            } else {
                                clearFlag(FLAG_S);
                            }
                            setFlag(FLAG_N); // N is always set
                            if (result == 0)
                                setFlag(FLAG_Z); // Z is set if result is zero
                            else
                                clearFlag(FLAG_Z);

                            if ((A & 0xF) < (value & 0xF))
                                setFlag(FLAG_H); // H is set if borrow occurs from bit 4
                            else
                                clearFlag(FLAG_H);

                            if ((BC() - 1) & 0xFFFF == 0)
                                setFlag(FLAG_PV); // PV is set if BC decrements to 0
                            else
                                clearFlag(FLAG_PV);                           // Carry flag is not affected

                            // Decrement HL and BC
                            setHL(HL() - 1);
                            setBC(BC() - 1);

                            // CPDR takes 21 T-states for BC != 0 and A != (HL)
                            cycles += 21;

                            if (BC() == 0 || result == 0) break;

                            // Terminate loop if BC == 0 or A == (HL)
                        } while (true);

                        // If the loop terminated naturally, add the remaining cycles
                        cycles += 16;
                        break;
                    }

                    case 0xB1: // CPIR
                    {
                        do {
                            // Read the value from memory at HL
                            uint8_t value = z80_mem_read(HL());

                            // Perform subtraction for comparison (A - (HL))
                            uint8_t result = A - value;

                            // Update flags
                            if (result & 0x80)
                                setFlag(FLAG_S); // Set S (sign flag) if result is negative
                            else
                                clearFlag(FLAG_S);

                            if (result == 0)
                                setFlag(FLAG_Z); // Set Z if result is zero
                            else
                                clearFlag(FLAG_Z);

                            setFlag(FLAG_N); // N is always set

                            if ((A & 0xF) < (value & 0xF))
                                setFlag(FLAG_H); // H is set if borrow occurs from bit 4
                            else
                                clearFlag(FLAG_H);

                            // Decrement BC and update P/V flag
                            setBC(BC() - 1);

                            if (BC() != 0)
                                setFlag(FLAG_PV); // Set P/V if BC is not zero
                            else
                                clearFlag(FLAG_PV);

                            // Increment HL
                            setHL(HL() + 1);

                            // CPIR takes 16 T-states
                            cycles += 21;

                            if (BC() == 0 || result == 0) break;

                        } while (true);

                        // If the loop terminated naturally, add the remaining cycles
                        cycles += 16;
                        break;
                    }

                    case 0xA1: // CPI
                    {
                        // Read the value from memory at HL
                        uint8_t value = z80_mem_read(HL());

                        // Perform subtraction for comparison (A - (HL))
                        uint8_t result = A - value;

                        // Update flags
                        if (result & 0x80)
                            setFlag(FLAG_S); // Set S (sign flag) if result is negative
                        else
                            clearFlag(FLAG_S);

                        if (result == 0)
                            setFlag(FLAG_Z); // Set Z if result is zero
                        else
                            clearFlag(FLAG_Z);

                        setFlag(FLAG_N); // N is always set

                        if ((A & 0xF) < (value & 0xF))
                            setFlag(FLAG_H); // H is set if borrow occurs from bit 4
                        else
                            clearFlag(FLAG_H);

                        // Decrement BC and update P/V flag
                        setBC(BC() - 1);

                        if (BC() != 0)
                            setFlag(FLAG_PV); // Set P/V if BC is not zero
                        else
                            clearFlag(FLAG_PV);

                        // Increment HL
                        setHL(HL() + 1);

                        // CPI takes 16 T-states
                        cycles += 16;

                        break;
                    }

                    case 0x46: // IM 0
                    {
                        // Set the interrupt mode to 0
                        interruptMode = 0;

                        // IM 0 takes 8 T-states
                        cycles += 8;

                        break;
                    }

                    case 0x56: // IM 1
                    {
                        // Set the interrupt mode to 1
                        interruptMode = 1;

                        // IM 1 takes 8 T-states
                        cycles += 8;

                        break;
                    }

                    case 0x5E: // IM 2
                    {
                        // Set the interrupt mode to 1
                        interruptMode = 2;

                        // IM 2 takes 8 T-states
                        cycles += 8;

                        break;
                    }

                    default:
                        cerr << "Unimplemented ED-prefixed opcode: ED " << hex << (int)subOpcode << endl;
                        break;
                }
                break;
            }

            case 0xDD: // Handle DD-prefixed instructions
            {
                uint8_t subOpcode = fetch(); // Fetch the next byte (sub-instruction)
                switch (subOpcode)
                {
                    case 0x36: // LD (IX+d), n
                    {
                        int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
                        uint8_t value = fetch();                            // Fetch immediate value n
                        uint16_t effective_address = IX + displacement;     // Compute effective address
                        z80_mem_write(effective_address, value);            // Write value to memory
                        cycles += 19;                                       // Timing for this instruction
                        break;
                    }

                    case 0x21: // LD IX, nn
                    {
                        uint8_t low = fetch();  // Fetch low byte
                        uint8_t high = fetch(); // Fetch high byte
                        IX = (high << 8) | low; // Combine to form the 16-bit immediate value
                        cycles += 14;           // Timing for this instruction
                        break;
                    }

                    case 0x2A: // LD IX, (nn)
                    {
                        uint16_t address = fetch() | (fetch() << 8);  // Fetch 16-bit address (nn)
                        uint8_t lowByte = z80_mem_read(address);      // Read low byte from memory
                        uint8_t highByte = z80_mem_read(address + 1); // Read high byte from memory
                        IX = (highByte << 8) | lowByte;               // Set IX register with the 16-bit value
                        cycles += 20;                                 // Instruction takes 20 cycles
                        break;
                    }

                    case 0x70: // LD (IX+d), B
                    case 0x71: // LD (IX+d), C
                    case 0x72: // LD (IX+d), D
                    case 0x73: // LD (IX+d), E
                    case 0x74: // LD (IX+d), H
                    case 0x75: // LD (IX+d), L
                    case 0x77: // LD (IX+d), A
                    {
                        uint8_t displacement = fetch();               // Fetch the signed displacement
                        uint16_t address = IX + (int8_t)displacement; // Compute the target memory address
                        uint8_t value = 0;

                        switch (subOpcode)
                        {
                        case 0x70:
                            value = B;
                            break; // B to memory
                        case 0x71:
                            value = C;
                            break; // C to memory
                        case 0x72:
                            value = D;
                            break; // D to memory
                        case 0x73:
                            value = E;
                            break; // E to memory
                        case 0x74:
                            value = H;
                            break; // H to memory
                        case 0x75:
                            value = L;
                            break; // L to memory
                        case 0x77:
                            value = A;
                            break; // A to memory
                        }

                        z80_mem_write(address, value); // Write the register's value to the computed address
                        cycles += 19;                  // Instruction takes 19 cycles
                        break;
                    }

                    case 0x22: // LD (nn), IX
                    {
                        uint16_t address = fetch() | (fetch() << 8);  // Fetch the 16-bit address (nn)
                        z80_mem_write(address, IX & 0xFF);            // Write low byte of IX to memory
                        z80_mem_write(address + 1, (IX >> 8) & 0xFF); // Write high byte of IX to memory
                        cycles += 20;                                 // Timing for this instruction
                        break;
                    }

                    case 0x46: // LD B, (IX+d)
                    case 0x4E: // LD C, (IX+d)
                    case 0x56: // LD D, (IX+d)
                    case 0x5E: // LD E, (IX+d)
                    case 0x66: // LD H, (IX+d)
                    case 0x6E: // LD L, (IX+d)
                    case 0x7E: // LD A, (IX+d)
                    {
                        int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement (d)
                        uint16_t effective_address = IX + displacement;     // Compute effective address
                        uint8_t value = z80_mem_read(effective_address);    // Read the value from memory

                        switch (subOpcode)
                        {
                        case 0x46:
                            B = value;
                            break; // Load into B
                        case 0x4E:
                            C = value;
                            break; // Load into C
                        case 0x56:
                            D = value;
                            break; // Load into D
                        case 0x5E:
                            E = value;
                            break; // Load into E
                        case 0x66:
                            H = value;
                            break; // Load into H
                        case 0x6E:
                            L = value;
                            break; // Load into L
                        case 0x7E:
                            A = value;
                            break; // Load into A
                        }

                        cycles += 19; // Instruction takes 19 cycles
                        break;
                    }

                    case 0xF9: // LD SP, IX (under DD prefix)
                    {
                        SP = IX;      // Set SP to the value of IX
                        cycles += 10; // Instruction takes 10 cycles
                        break;
                    }

                    case 0x86: // ADD A, (IX+d)
                    {
                        int8_t displacement = static_cast<int8_t>(fetch());   // Fetch signed displacement (d)
                        uint16_t effectiveAddress = IX + displacement;        // Calculate effective address
                        uint8_t memoryValue = z80_mem_read(effectiveAddress); // Fetch value from memory

                        A = add_flag(A, memoryValue); // Perform addition and update flags
                        cycles += 19;                 // Instruction timing
                        break;
                    }

                    case 0x09: // ADD IX, BC
                        IX = add_flag16(IX, BC());
                        cycles += 15;
                        break;

                    case 0x19: // ADD IX, DE
                        IX = add_flag16(IX, DE());
                        cycles += 15;
                        break;

                    case 0x29: // ADD IX, IX
                        IX = add_flag16(IX, IX);
                        cycles += 15;
                        break;

                    case 0x39: // ADD IX, SP
                        IX = add_flag16(IX, SP);
                        cycles += 15;
                        break;

                    case 0x8E:
                    { // ADC A, (IX+d)
                        int8_t offset = (int8_t)fetch();
                        A = adc_flag(A, z80_mem_read(IX + offset));
                        cycles += 19; // 4, 4, 3, 5, 3 T-states
                        break;
                    }

                    case 0x96: // SUB (IX+d)
                        A = sub_flag(A, z80_mem_read(IX + (int8_t)fetch()));
                        cycles += 19; // Indexed access adds more cycles
                        break;

                    case 0xB6:
                    {
                        uint8_t d = fetch();
                        OR(z80_mem_read(IX + static_cast<int8_t>(d)));
                        cycles += 19;
                        break; // OR (IX+d)
                    }

                    case 0xE5: // PUSH IX
                    {
                        PUSH_IX(); // Call the function to handle PUSH IX
                        break;
                    }

                    case 0xE1:
                    {                                      // POP IX
                        uint8_t low = z80_mem_read(SP++);  // Read low byte from stack
                        uint8_t high = z80_mem_read(SP++); // Read high byte from stack
                        IX = ((high << 8) | low);          // Combine high and low into IX
                        cycles += 14;                      // 14 T-states for POP IX
                        break;
                    }

                    case 0xAE:
                    {
                        int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
                        uint16_t address = IX + displacement;               // Compute effective address
                        uint8_t value = z80_mem_read(address);              // Read the value from memory

                        xorReg(value);
                        cycles += 19; // Instruction takes 19 cycles
                        break;
                    }

                    case 0xA6:
                    {
                        and_ix_d();
                        break;
                    }

                    case 0xBE:
                    {
                        uint8_t displacement = static_cast<int8_t>(fetch());
                        uint8_t address = IX + displacement;
                        cp(z80_mem_read(address));
                        cycles += 19;
                        break;
                    }

                    case 0x35: // DEC (IX+d)
                    {
                        int8_t displacement = static_cast<int8_t>(fetch()); // Get displacement
                        uint16_t address = IX + displacement;               // Compute effective address

                        // Read value from memory, decrement, and write back
                        uint8_t value = z80_mem_read(address);
                        zdec(value);
                        z80_mem_write(address, value);

                        cycles += 23; // DEC (IX+d) takes 23 T-states
                        break;
                    }

                    case 0x34: // INC (IX+d)
                    {
                        int8_t displacement = static_cast<int8_t>(fetch()); // Get displacement
                        uint16_t address = IX + displacement;               // Compute effective address

                        // Read value from memory, increment, and write back
                        uint8_t value = z80_mem_read(address);
                        zinc(value);
                        z80_mem_write(address, value);

                        cycles += 23; // INC (IX+d) takes 23 T-states
                        break;
                    }

                    case 0x2B: // DEC IX
                    {
                        IX--;
                        cycles += 10;
                        break;
                    }

                    case 0x23: // INC IX
                    {
                        IX++;
                        cycles += 10;
                        break;
                    }

                    case 0xE3: // EX (SP), IX
                    {
                        // Retrieve values from memory at SP and SP+1
                        uint8_t lowSP = z80_mem_read(SP);
                        uint8_t highSP = z80_mem_read(SP + 1);

                        // Store the original IX values
                        uint8_t lowIX = IX & 0xFF;
                        uint8_t highIX = (IX >> 8) & 0xFF;

                        // Exchange the values
                        z80_mem_write(SP, lowIX);      // Write IX low byte to SP
                        z80_mem_write(SP + 1, highIX); // Write IX high byte to SP+1
                        IX = (highSP << 8) | lowSP;  // Load SP values into HL

                        // Instruction timing: 19 T-states
                        cycles += 23;

                        break;
                    }

                    case 0xE9: // JP (IX)
                    {
                        // Set the program counter (PC) to the value of IX
                        PC = IX;

                        // JP (IX) takes 8 T-states (4 for DD prefix + 4 for the E9 opcode)
                        cycles += 8;

                        break;
                    }

                    case 0xCB:
                    {                                                       // CB-prefixed instructions with IX+d
                        int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
                        uint16_t address = IX + displacement;               // Compute effective address
                        uint8_t targetValue = z80_mem_read(address);        // Read the value from memory

                        uint8_t bitInstruction = fetch();                 // Fetch bit-specific instruction
                        uint8_t bitNumber = (bitInstruction >> 3) & 0x07; // Extract the bit number (middle 3 bits)

                        switch (bitInstruction)
                        {
                            case 0x06:
                            { // RLC (IX+n)
                                // printf("Performing RLC (IX+n)\n");
                                uint8_t bit7 = (targetValue & 0x80) >> 7; // Extract bit 7
                                targetValue = (targetValue << 1) | bit7;  // Rotate left circularly
                                z80_mem_write(address, targetValue);      // Write back to memory

                                // Update flags
                                clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);
                                if (targetValue & 0x80)
                                    setFlag(FLAG_S); // Set Sign flag if bit 7 is 1
                                if (targetValue == 0)
                                    setFlag(FLAG_Z); // Set Zero flag if result is 0
                                if (checkParity(targetValue))
                                    setFlag(FLAG_PV); // Set Parity flag
                                if (bit7)
                                    setFlag(FLAG_C); // Set Carry flag based on original bit 7

                                cycles += 23; // RLC (IX+n) takes 23 T-states
                                break;
                            }

                            case 0x16:
                            { // RL (IX+n)
                                // printf("Performing RL (IX+n)\n");
                                rlMem(address); // Perform RL on memory (IX+n)
                                cycles += 23;   // RL (IX+n) takes 23 T-states
                                break;
                            }

                            case 0x1E:
                            {
                                rrMem(address); // Perform RR on memory (IX+n)
                                cycles += 23;   // RR (IX+n) takes 23 T-states
                                break;
                            }

                            case 0x0E: // RRC (IX+d)
                            {
                                rrcMem(address); // Perform RRC on memory (IX+d)
                                cycles += 23;    // RRC (IX+d) takes 23 T-states
                                break;
                            }

                                            // SET b, (IX+d)
                            case 0xC0: case 0xC1: case 0xC2: case 0xC3: 
                            case 0xC4: case 0xC5: case 0xC6: case 0xC7: 
                            case 0xC8: case 0xC9: case 0xCA: case 0xCB: 
                            case 0xCC: case 0xCD: case 0xCE: case 0xCF: 
                            case 0xD0: case 0xD1: case 0xD2: case 0xD3: 
                            case 0xD4: case 0xD5: case 0xD6: case 0xD7: 
                            case 0xD8: case 0xD9: case 0xDA: case 0xDB: 
                            case 0xDC: case 0xDD: case 0xDE: case 0xDF: 
                            case 0xE0: case 0xE1: case 0xE2: case 0xE3: 
                            case 0xE4: case 0xE5: case 0xE6: case 0xE7: 
                            case 0xE8: case 0xE9: case 0xEA: case 0xEB: 
                            case 0xEC: case 0xED: case 0xEE: case 0xEF: 
                            case 0xF0: case 0xF1: case 0xF2: case 0xF3: 
                            case 0xF4: case 0xF5: case 0xF6: case 0xF7: 
                            case 0xF8: case 0xF9: case 0xFA: case 0xFB: 
                            case 0xFC: case 0xFD: case 0xFE: case 0xFF:
                            {
                                targetValue |= (1 << bitNumber);            // Set the specific bit
                                z80_mem_write(address, targetValue);        // Write the updated value back to memory

                                cycles += 23;                               // SET (IX+d) takes 23 T-states
                                break;
                            }

                            case 0x26: // SLA (IX+d)
                                slaMem(address);
                                cycles += 23;
                                break;

                            case 0x2E: // SRA (IX+d)
                                sraMem(address);
                                cycles += 23;
                                break;

                            case 0x3E:
                            { // SRL (IX+d)
                                srl(targetValue);
                                z80_mem_write(address, targetValue);
                                cycles += 23; // SRL (IX+d) takes 23 T-states
                                break;
                            }

                            case 0x36:
                            { // SLL (IX+d)
                                sll(targetValue);
                                z80_mem_write(address, targetValue);
                                cycles += 23; // SLL (IX+d) takes 23 T-states
                                break;
                            }

                            default:
                                // Add RES (IX+d) logic for opcodes in the range 0x80–0xBF
                                if ((bitInstruction & 0xC0) == 0x80)
                                {
                                    // printf("Performing RES (IX+d)\n");
                                    targetValue &= ~(1 << bitNumber);    // Clear the specified bit
                                    z80_mem_write(address, targetValue); // Write back modified value to memory
                                    // printf("Updated value at address 0x%04X: 0x%02X\n", address, targetValue);
                                    cycles += 15; // RES (IX+d) takes 23 T-states
                                }

                                else if ((bitInstruction & 0xC0) == 0x40)
                                {
                                    // Perform BIT operation
                                    uint8_t mask = 1 << bitNumber; // Create mask for the bit to test
                                    if (targetValue & mask)
                                    {
                                        clearFlag(FLAG_Z); // Reset Z flag if the bit is set
                                    }
                                    else
                                    {
                                        setFlag(FLAG_Z); // Set Z flag if the bit is 0
                                    }
                                    setFlag(FLAG_H);   // H flag is always set
                                    clearFlag(FLAG_N); // N flag is always reset
                                    // No changes to the memory content
                                    cycles += 20; // BIT (IX+d) takes 20 T-states
                                }
                                else
                                {
                                    printf("Unimplemented DD+CB opcode for IX+n: 0x%02X\n", bitInstruction);
                                }
                                break;
                        }
                        break;
                    }

                    default:
                        cerr << "Unimplemented DD-prefixed opcode: " << hex << (int)subOpcode << endl;
                        break;
                }
                break;
            }

            case 0xFD: // Handle FD-prefixed instructions
            {
                uint8_t subOpcode = fetch(); // Fetch the next byte
                switch (subOpcode)
                {
                case 0x21: // LD IY, nn
                {
                    uint8_t low = fetch();  // Fetch low byte
                    uint8_t high = fetch(); // Fetch high byte
                    IY = (high << 8) | low; // Combine to form the 16-bit immediate value
                    cycles += 14;           // Timing for this instruction
                    break;
                }

                case 0x2A: // LD IY, (nn)
                {
                    uint16_t address = fetch() | (fetch() << 8);  // Fetch 16-bit address (nn)
                    uint8_t lowByte = z80_mem_read(address);      // Read low byte from memory
                    uint8_t highByte = z80_mem_read(address + 1); // Read high byte from memory
                    IY = (highByte << 8) | lowByte;               // Set IY register with the 16-bit value
                    cycles += 20;                                 // Instruction takes 20 cycles
                    break;
                }

                    // LD (IY+d), n
                case 0x36:
                {
                    int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
                    uint8_t value = fetch();                            // Fetch immediate value n
                    uint16_t effective_address = IY + displacement;     // Compute effective address
                    z80_mem_write(effective_address, value);            // Write value to memory
                    cycles += 19;                                       // Timing for this instruction
                    break;
                }

                // LD (IY+d), r
                case 0x70: // LD (IY+d), B
                case 0x71: // LD (IY+d), C
                case 0x72: // LD (IY+d), D
                case 0x73: // LD (IY+d), E
                case 0x74: // LD (IY+d), H
                case 0x75: // LD (IY+d), L
                case 0x77: // LD (IY+d), A
                {
                    int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
                    uint16_t effective_address = IY + displacement;     // Compute the target memory address
                    uint8_t value = 0;

                    // Determine the source register
                    switch (subOpcode)
                    {
                    case 0x70:
                        value = B;
                        break; // Load B to memory
                    case 0x71:
                        value = C;
                        break; // Load C to memory
                    case 0x72:
                        value = D;
                        break; // Load D to memory
                    case 0x73:
                        value = E;
                        break; // Load E to memory
                    case 0x74:
                        value = H;
                        break; // Load H to memory
                    case 0x75:
                        value = L;
                        break; // Load L to memory
                    case 0x77:
                        value = A;
                        break; // Load A to memory
                    }

                    z80_mem_write(effective_address, value); // Write value to memory
                    cycles += 19;                            // Instruction takes 19 cycles
                    break;
                }
                case 0x22: // LD (nn), IY
                {
                    uint16_t address = fetch() | (fetch() << 8);  // Fetch the 16-bit address (nn)
                    z80_mem_write(address, IY & 0xFF);            // Write low byte of IY to memory
                    z80_mem_write(address + 1, (IY >> 8) & 0xFF); // Write high byte of IY to memory
                    cycles += 20;                                 // Timing for this instruction
                    break;
                }

                // Inside the FD-prefixed instruction handler
                case 0x46: // LD B, (IY+d)
                case 0x4E: // LD C, (IY+d)
                case 0x56: // LD D, (IY+d)
                case 0x5E: // LD E, (IY+d)
                case 0x66: // LD H, (IY+d)
                case 0x6E: // LD L, (IY+d)
                case 0x7E: // LD A, (IY+d)
                {
                    int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement (d)
                    uint16_t effective_address = IY + displacement;     // Compute effective address
                    uint8_t value = z80_mem_read(effective_address);    // Read the value from memory

                    switch (subOpcode)
                    {
                    case 0x46:
                        B = value;
                        break; // Load into B
                    case 0x4E:
                        C = value;
                        break; // Load into C
                    case 0x56:
                        D = value;
                        break; // Load into D
                    case 0x5E:
                        E = value;
                        break; // Load into E
                    case 0x66:
                        H = value;
                        break; // Load into H
                    case 0x6E:
                        L = value;
                        break; // Load into L
                    case 0x7E:
                        A = value;
                        break; // Load into A
                    }

                    cycles += 19; // Instruction takes 19 cycles
                    break;
                }

                case 0xF9: // LD SP, IY (under FD prefix)
                {
                    SP = IY;      // Set SP to the value of IY
                    cycles += 10; // Instruction takes 10 cycles
                    break;
                }

                case 0x86: // ADD A, (IY+d)
                {
                    int8_t displacement = static_cast<int8_t>(fetch());   // Fetch signed displacement (d)
                    uint16_t effectiveAddress = IY + displacement;        // Calculate effective address
                    uint8_t memoryValue = z80_mem_read(effectiveAddress); // Fetch value from memory

                    A = add_flag(A, memoryValue); // Perform addition and update flags
                    cycles += 19;                 // Instruction timing
                    break;
                }

                case 0x09: // ADD IY, BC
                    IY = add_flag16(IY, BC());
                    cycles += 15; // 4, 4, 4, 3 T-states
                    break;

                case 0x19: // ADD IY, DE
                    IY = add_flag16(IY, DE());
                    cycles += 15; // 4, 4, 4, 3 T-states
                    break;

                case 0x29: // ADD IY, IY
                    IY = add_flag16(IY, IY);
                    cycles += 15; // 4, 4, 4, 3 T-states
                    break;

                case 0x39: // ADD IY, SP
                    IY = add_flag16(IY, SP);
                    cycles += 15; // 4, 4, 4, 3 T-states
                    break;

                case 0x8E:
                { // ADC A, (IY+d)
                    int8_t offset = (int8_t)fetch();
                    A = adc_flag(A, z80_mem_read(IY + offset));
                    cycles += 19; // 4, 4, 3, 5, 3 T-states
                    break;
                }

                case 0x96: // SUB (IY+d)
                    A = sub_flag(A, z80_mem_read(IY + (int8_t)fetch()));
                    cycles += 19; // Indexed access adds more cycles
                    break;

                case 0xB6:
                {
                    uint8_t d = fetch();
                    OR(z80_mem_read(IY + static_cast<int8_t>(d)));
                    cycles += 19;
                    break; // OR (IY+d)
                }

                case 0xE5: // PUSH IY
                {
                    PUSH_IY(); // Call the function to handle PUSH IX
                    break;
                }

                case 0xE1:
                {                                      // POP IX
                    uint8_t low = z80_mem_read(SP++);  // Read low byte from stack
                    uint8_t high = z80_mem_read(SP++); // Read high byte from stack
                    IY = ((high << 8) | low);          // Combine high and low into IX
                    cycles += 14;                      // 14 T-states for POP IX
                    break;
                }

                case 0xAE:
                {
                    int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
                    uint16_t address = IY + displacement;               // Compute effective address
                    uint8_t value = z80_mem_read(address);              // Read the value from memory

                    xorReg(value);
                    cycles += 19; // Instruction takes 19 cycles
                    break;
                }

                case 0xA6:
                {
                    and_iy_d();
                    break;
                }

                case 0xBE: // cp
                {
                    uint8_t displacement = static_cast<int8_t>(fetch());
                    uint8_t address = IY + displacement;
                    cp(z80_mem_read(address));
                    cycles += 19;
                    break;
                }

                case 0x35: // DEC (IY+d)
                {
                    int8_t displacement = static_cast<int8_t>(fetch()); // Get displacement
                    uint16_t address = IY + displacement;               // Compute effective address

                    // Read value from memory, decrement, and write back
                    uint8_t value = z80_mem_read(address);
                    zdec(value);
                    z80_mem_write(address, value);

                    cycles += 23; // DEC (IY+d) takes 23 T-states
                    break;
                }

                case 0x34: // INC (IY+d)
                {
                    int8_t displacement = static_cast<int8_t>(fetch()); // Get displacement
                    uint16_t address = IY + displacement;               // Compute effective address

                    // Read value from memory, increment, and write back
                    uint8_t value = z80_mem_read(address);
                    zinc(value);
                    z80_mem_write(address, value);

                    cycles += 23; // INC (IY+d) takes 23 T-states
                    break;
                }

                case 0x2B: // DEC IY
                {
                    IY--;
                    cycles += 10;
                    break;
                }

                case 0x23: // INC IY
                {
                    IY++;
                    cycles += 10;
                    break;
                }

                case 0xE3: // EX (SP), IY
                {
                    // Retrieve values from memory at SP and SP+1
                    uint8_t lowSP = z80_mem_read(SP);
                    uint8_t highSP = z80_mem_read(SP + 1);

                    // Store the original IY values
                    uint8_t lowIY = IY & 0xFF;
                    uint8_t highIY = (IY >> 8) & 0xFF;

                    // Exchange the values
                    z80_mem_write(SP, lowIY);      // Write IY low byte to SP
                    z80_mem_write(SP + 1, highIY); // Write IY high byte to SP+1
                    IY = (highSP << 8) | lowSP;  // Load SP values into HL

                    // Instruction timing: 19 T-states
                    cycles += 23;

                    break;
                }

                case 0xE9: // JP (IY)
                {
                    // Set the program counter (PC) to the value of IY
                    PC = IY;

                    // JP (IY) takes 8 T-states (4 for DD prefix + 4 for the E9 opcode)
                    cycles += 8;

                    break;
                }

                case 0xCB:
                {                                                       // CB-prefixed instructions with IY+d
                    int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
                    uint16_t address = IY + displacement;               // Compute effective address
                    uint8_t targetValue = z80_mem_read(address);        // Read the value from memory

                    uint8_t bitInstruction = fetch();                 // Fetch bit-specific instruction
                    uint8_t bitNumber = (bitInstruction >> 3) & 0x07; // Extract the bit number (middle 3 bits)

                    switch (bitInstruction)
                    {
                        case 0x06:
                        { // RLC (IY+n)
                            // printf("Performing RLC (IY+n)\n");
                            uint8_t bit7 = (targetValue & 0x80) >> 7; // Extract bit 7
                            targetValue = (targetValue << 1) | bit7;  // Rotate left circularly
                            z80_mem_write(address, targetValue);      // Write back to memory

                            // Update flags
                            clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);
                            if (targetValue & 0x80)
                                setFlag(FLAG_S); // Set Sign flag if bit 7 is 1
                            if (targetValue == 0)
                                setFlag(FLAG_Z); // Set Zero flag if result is 0
                            if (checkParity(targetValue))
                                setFlag(FLAG_PV); // Set Parity flag
                            if (bit7)
                                setFlag(FLAG_C); // Set Carry flag based on original bit 7

                            cycles += 23; // RLC (IY+n) takes 23 T-states
                            break;
                        }

                        case 0x16:
                        { // RL (IY+n)
                            // printf("Performing RL (IY+n)\n");
                            rlMem(address); // Perform RL on memory (IY+n)
                            cycles += 23;   // RL (IY+n) takes 23 T-states
                            break;
                        }

                        case 0x1E:
                        {
                            rrMem(address); // Perform RR on memory (IY+n)
                            cycles += 23;   // RR (IY+n) takes 23 T-states
                            break;
                        }

                        case 0x0E: // RRC (IY+d)
                        {
                            rrcMem(address); // Perform RRC on memory (IY+d)
                            cycles += 23;    // RRC (IY+d) takes 23 T-states
                            break;
                        }

                                                        // SET b, (IY+d)
                        case 0xC0: case 0xC1: case 0xC2: case 0xC3: 
                        case 0xC4: case 0xC5: case 0xC6: case 0xC7: 
                        case 0xC8: case 0xC9: case 0xCA: case 0xCB: 
                        case 0xCC: case 0xCD: case 0xCE: case 0xCF: 
                        case 0xD0: case 0xD1: case 0xD2: case 0xD3: 
                        case 0xD4: case 0xD5: case 0xD6: case 0xD7: 
                        case 0xD8: case 0xD9: case 0xDA: case 0xDB: 
                        case 0xDC: case 0xDD: case 0xDE: case 0xDF: 
                        case 0xE0: case 0xE1: case 0xE2: case 0xE3: 
                        case 0xE4: case 0xE5: case 0xE6: case 0xE7: 
                        case 0xE8: case 0xE9: case 0xEA: case 0xEB: 
                        case 0xEC: case 0xED: case 0xEE: case 0xEF: 
                        case 0xF0: case 0xF1: case 0xF2: case 0xF3: 
                        case 0xF4: case 0xF5: case 0xF6: case 0xF7: 
                        case 0xF8: case 0xF9: case 0xFA: case 0xFB: 
                        case 0xFC: case 0xFD: case 0xFE: case 0xFF:
                        {
                            targetValue |= (1 << bitNumber);            // Set the specific bit
                            z80_mem_write(address, targetValue);        // Write the updated value back to memory

                            cycles += 23;                               // SET (IY+d) takes 23 T-states
                            break;
                        }

                        case 0x26: // SLA (IY+d)
                            slaMem(address);
                            cycles += 23;
                            break;

                        case 0x2E: // SRA (IY+d)
                            sraMem(address);
                            cycles += 23;
                            break;

                        case 0x3E:
                        { // SRL (IY+d)
                            srl(targetValue);
                            z80_mem_write(address, targetValue);
                            cycles += 23; // SRL (IY+d) takes 23 T-states
                            break;
                        }

                        case 0x36:
                        { // SLL (IY+d)
                            sll(targetValue);
                            z80_mem_write(address, targetValue);
                            cycles += 23; // SLL (IY+d) takes 23 T-states
                            break;
                        }

                        default:
                            // Add RES (IY+d) logic for opcodes in the range 0x80–0xBF
                            if ((bitInstruction & 0xC0) == 0x80)
                            {
                                // printf("Performing RES (IY+d)\n");
                                targetValue &= ~(1 << bitNumber);    // Clear the specified bit
                                z80_mem_write(address, targetValue); // Write back modified value to memory
                                // printf("Updated value at address 0x%04X: 0x%02X\n", address, targetValue);
                                cycles += 15; // RES (IY+d) takes 23 T-states
                            }

                            else if ((bitInstruction & 0xC0) == 0x40)
                            {
                                // Perform BIT operation
                                uint8_t mask = 1 << bitNumber; // Create mask for the bit to test
                                if (targetValue & mask)
                                {
                                    clearFlag(FLAG_Z); // Reset Z flag if the bit is set
                                }
                                else
                                {
                                    setFlag(FLAG_Z); // Set Z flag if the bit is 0
                                }
                                setFlag(FLAG_H);   // H flag is always set
                                clearFlag(FLAG_N); // N flag is always reset
                                // No changes to the memory content
                                cycles += 20; // BIT (IY+d) takes 20 T-states
                            }

                            else
                            {
                                printf("Unimplemented DD+CB opcode for IY+n: 0x%02X\n", bitInstruction);
                            }
                            break;
                    }
                    break;
                }

                default:
                    cerr << "Unimplemented FD-prefixed opcode: " << hex << (int)subOpcode << endl;
                    break;
                }
                break;
            }

            case 0xCB: // Handle CB-prefixed instructions
            {
                uint8_t subOpcode = fetch(); // Fetch the next byte (sub-opcode)

                if ((subOpcode & 0xC0) == 0x40) // Check if it's a BIT instruction (0b01xxxxx)
                {
                    uint8_t bitNumber = (subOpcode >> 3) & 0x07; // Extract the bit number (bits 3-5)
                    uint8_t regCode = subOpcode & 0x07;    // Extract the register code (bits 0-2)

                    uint8_t &reg = decodeRegister(regCode); // Map regCode to the corresponding CPU register
                    bit(bitNumber, reg);                          // Execute the BIT instruction
                    cycles += 8;
                    break;
                }

                if ((subOpcode & 0xC0) == 0xC0) // Check if it's a SET instruction (0b11xxxxx)
                {
                    uint8_t bit = (subOpcode >> 3) & 0x07; // Bits 3, 4, 5 specify the bit number
                    uint8_t regCode = subOpcode & 0x07;    // Bits 0, 1, 2 specify the register or (HL)

                    // Perform the SET operation
                    switch (regCode)
                    {
                    case 0x00: // B
                        setBit(B, bit);
                        cycles += 8;
                        break;
                    case 0x01: // C
                        setBit(C, bit);
                        cycles += 8;
                        break;
                    case 0x02: // D
                        setBit(D, bit);
                        cycles += 8;
                        break;
                    case 0x03: // E
                        setBit(E, bit);
                        cycles += 8;
                        break;
                    case 0x04: // H
                        setBit(H, bit);
                        cycles += 8;
                        break;
                    case 0x05: // L
                        setBit(L, bit);
                        cycles += 8;
                        break;
                    case 0x06: // (HL)
                    {
                        uint8_t value = z80_mem_read(HL());
                        setBit(value, bit);
                        z80_mem_write(HL(), value);
                        cycles += 15; // Memory operations take longer
                        break;
                    }
                    case 0x07: // A
                        setBit(A, bit);
                        cycles += 8;
                        break;
                    default:
                        printf("Unhandled SET instruction: 0x%02X\n", subOpcode);
                        break;
                    }

                    break; // Exit after handling SET
                }

                switch (subOpcode)
                {
                    // RES bit 0
                    case 0x80:
                        res(0, B);
                        break; // RES 0, B
                    case 0x81:
                        res(0, C);
                        break; // RES 0, C
                    case 0x82:
                        res(0, D);
                        break; // RES 0, D
                    case 0x83:
                        res(0, E);
                        break; // RES 0, E
                    case 0x84:
                        res(0, H);
                        break; // RES 0, H
                    case 0x85:
                        res(0, L);
                        break; // RES 0, L
                    case 0x87:
                        res(0, A);
                        break; // RES 0, A

                    // RES bit 1
                    case 0x88:
                        res(1, B);
                        break; // RES 1, B
                    case 0x89:
                        res(1, C);
                        break; // RES 1, C
                    case 0x8A:
                        res(1, D);
                        break; // RES 1, D
                    case 0x8B:
                        res(1, E);
                        break; // RES 1, E
                    case 0x8C:
                        res(1, H);
                        break; // RES 1, H
                    case 0x8D:
                        res(1, L);
                        break; // RES 1, L
                    case 0x8F:
                        res(1, A);
                        break; // RES 1, A

                    // RES bit 2
                    case 0x90:
                        res(2, B);
                        break; // RES 2, B
                    case 0x91:
                        res(2, C);
                        break; // RES 2, C
                    case 0x92:
                        res(2, D);
                        break; // RES 2, D
                    case 0x93:
                        res(2, E);
                        break; // RES 2, E
                    case 0x94:
                        res(2, H);
                        break; // RES 2, H
                    case 0x95:
                        res(2, L);
                        break; // RES 2, L
                    case 0x97:
                        res(2, A);
                        break; // RES 2, A

                    // RES bit 3
                    case 0x98:
                        res(3, B);
                        break; // RES 3, B
                    case 0x99:
                        res(3, C);
                        break; // RES 3, C
                    case 0x9A:
                        res(3, D);
                        break; // RES 3, D
                    case 0x9B:
                        res(3, E);
                        break; // RES 3, E
                    case 0x9C:
                        res(3, H);
                        break; // RES 3, H
                    case 0x9D:
                        res(3, L);
                        break; // RES 3, L
                    case 0x9F:
                        res(3, A);
                        break; // RES 3, A

                    // RES bit 4
                    case 0xA0:
                        res(4, B);
                        break; // RES 4, B
                    case 0xA1:
                        res(4, C);
                        break; // RES 4, C
                    case 0xA2:
                        res(4, D);
                        break; // RES 4, D
                    case 0xA3:
                        res(4, E);
                        break; // RES 4, E
                    case 0xA4:
                        res(4, H);
                        break; // RES 4, H
                    case 0xA5:
                        res(4, L);
                        break; // RES 4, L
                    case 0xA7:
                        res(4, A);
                        break; // RES 4, A

                    // RES bit 5
                    case 0xA8:
                        res(5, B);
                        break; // RES 5, B
                    case 0xA9:
                        res(5, C);
                        break; // RES 5, C
                    case 0xAA:
                        res(5, D);
                        break; // RES 5, D
                    case 0xAB:
                        res(5, E);
                        break; // RES 5, E
                    case 0xAC:
                        res(5, H);
                        break; // RES 5, H
                    case 0xAD:
                        res(5, L);
                        break; // RES 5, L
                    case 0xAF:
                        res(5, A);
                        break; // RES 5, A

                    // RES bit 6
                    case 0xB0:
                        res(6, B);
                        break; // RES 6, B
                    case 0xB1:
                        res(6, C);
                        break; // RES 6, C
                    case 0xB2:
                        res(6, D);
                        break; // RES 6, D
                    case 0xB3:
                        res(6, E);
                        break; // RES 6, E
                    case 0xB4:
                        res(6, H);
                        break; // RES 6, H
                    case 0xB5:
                        res(6, L);
                        break; // RES 6, L
                    case 0xB7:
                        res(6, A);
                        break; // RES 6, A

                    // RES bit 7
                    case 0xB8:
                        res(7, B);
                        break; // RES 7, B
                    case 0xB9:
                        res(7, C);
                        break; // RES 7, C
                    case 0xBA:
                        res(7, D);
                        break; // RES 7, D
                    case 0xBB:
                        res(7, E);
                        break; // RES 7, E
                    case 0xBC:
                        res(7, H);
                        break; // RES 7, H
                    case 0xBD:
                        res(7, L);
                        break; // RES 7, L
                    case 0xBF:
                        res(7, A);
                        break; // RES 7, A

                    case 0x86: // RES 0, (HL)
                    case 0x8E: // RES 1, (HL)
                    case 0x96: // RES 2, (HL)
                    case 0x9E: // RES 3, (HL)
                    case 0xA6: // RES 4, (HL)
                    case 0xAE: // RES 5, (HL)
                    case 0xB6: // RES 6, (HL)
                    case 0xBE: // RES 7, (HL)
                    {
                        uint8_t bit = (subOpcode >> 3) & 0x07; // Extract bit number (bits 3-5)
                        uint8_t value = z80_mem_read(HL());    // Read value from memory pointed by HL
                        value &= ~(1 << bit);                  // Reset the specified bit
                        z80_mem_write(HL(), value);            // Write the modified value back to memory
                        cycles += 7;                          // RES (HL) takes 15 cycles
                        break;
                    }

                    case 0x10:
                        rl(B);
                        cycles += 8;
                        break; // RL B
                    case 0x11:
                        rl(C);
                        cycles += 8;
                        break; // RL C
                    case 0x12:
                        rl(D);
                        cycles += 8;
                        break; // RL D
                    case 0x13:
                        rl(E);
                        cycles += 8;
                        break; // RL E
                    case 0x14:
                        rl(H);
                        cycles += 8;
                        break; // RL H
                    case 0x15:
                        rl(L);
                        cycles += 8;
                        break; // RL L
                    case 0x16:
                        rlMem(HL());
                        cycles += 15;
                        break; // RL (HL)
                    case 0x17:
                        rl(A);
                        cycles += 8;
                        break; // RL A

                    case 0x18: rr(B); cycles += 8; break; // RR B
                    case 0x19: rr(C); cycles += 8; break; // RR C
                    case 0x1A: rr(D); cycles += 8; break; // RR D
                    case 0x1B: rr(E); cycles += 8; break; // RR E
                    case 0x1C: rr(H); cycles += 8; break; // RR H
                    case 0x1D: rr(L); cycles += 8; break; // RR L
                    case 0x1F: rr(A); cycles += 8; break; // RR A
                    case 0x1E: rrMem(HL()); cycles += 15; break; // RR (HL)



                    case 0x06:
                    {                 // RLC (HL)
                        rlcMemHL();   // Call the RLC (HL) implementation
                        cycles += 15; // RLC (HL) takes 15 T-states
                        break;
                    }

                    // RLC instructions for each register
                    case 0x00: // RLC B
                        rlc(B);
                        cycles += 8; // RLC r takes 8 T-states
                        break;

                    case 0x01: // RLC C
                        rlc(C);
                        cycles += 8;
                        break;

                    case 0x02: // RLC D
                        rlc(D);
                        cycles += 8;
                        break;

                    case 0x03: // RLC E
                        rlc(E);
                        cycles += 8;
                        break;

                    case 0x04: // RLC H
                        rlc(H);
                        cycles += 8;
                        break;

                    case 0x05: // RLC L
                        rlc(L);
                        cycles += 8;
                        break;

                    case 0x07: // RLC A
                        rlc(A);
                        cycles += 8;
                        break;

                        // RRC r instructions
                    case 0x0F: // RRC A
                        rrc(A);
                        cycles += 8;
                        break;

                    case 0x08: // RRC B
                        rrc(B);
                        cycles += 8;
                        break;

                    case 0x09: // RRC C
                        rrc(C);
                        cycles += 8;
                        break;

                    case 0x0A: // RRC D
                        rrc(D);
                        cycles += 8;
                        break;

                    case 0x0B: // RRC E
                        rrc(E);
                        cycles += 8;
                        break;

                    case 0x0C: // RRC H
                        rrc(H);
                        cycles += 8;
                        break;

                    case 0x0D: // RRC L
                        rrc(L);
                        cycles += 8;
                        break;

                    // RRC (HL)
                    case 0x0E:
                    {
                        rrcMem(HL());   // Perform RRC on the memory address in HL
                        cycles += 15; // RRC (HL) takes 15 T-states
                        break;
                    }

                    case 0x20: sla(B); cycles += 8; break; // SLA B
                    case 0x21: sla(C); cycles += 8; break; // SLA C
                    case 0x22: sla(D); cycles += 8; break; // SLA D
                    case 0x23: sla(E); cycles += 8; break; // SLA E
                    case 0x24: sla(H); cycles += 8; break; // SLA H
                    case 0x25: sla(L); cycles += 8; break; // SLA L
                    case 0x26: slaMem(HL()); cycles += 15; break; // SLA (HL)
                    case 0x27: sla(A); cycles += 8; break; // SLA A

                    case 0x2E: // SRA (HL)
                        sraMem(HL());
                        cycles += 15; // SRA (HL) takes 15 T-states
                        break;

                    case 0x2F: // SRA A
                        sra(A);
                        cycles += 8; // SRA A takes 8 T-states
                        break;

                    case 0x28: // SRA B
                        sra(B);
                        cycles += 8;
                        break;

                    case 0x29: // SRA C
                        sra(C);
                        cycles += 8;
                        break;

                    case 0x2A: // SRA D
                        sra(D);
                        cycles += 8;
                        break;

                    case 0x2B: // SRA E
                        sra(E);
                        cycles += 8;
                        break;

                    case 0x2C: // SRA H
                        sra(H);
                        cycles += 8;
                        break;

                    case 0x2D: // SRA L
                        sra(L);
                        cycles += 8;
                        break;

                    // SRL r
                    case 0x38: srl(B); cycles += 8; break;
                    case 0x39: srl(C); cycles += 8; break;
                    case 0x3A: srl(D); cycles += 8; break;
                    case 0x3B: srl(E); cycles += 8; break;
                    case 0x3C: srl(H); cycles += 8; break;
                    case 0x3D: srl(L); cycles += 8; break;
                    case 0x3E: {
                        uint8_t hlValue = z80_mem_read(HL());
                        srl(hlValue);
                        z80_mem_write(HL(), hlValue);
                        cycles += 15;
                        break;
                    }
                    case 0x3F: srl(A); cycles += 8; break;

                    // SLL r
                    case 0x30: sll(B); cycles += 8; break;
                    case 0x31: sll(C); cycles += 8; break;
                    case 0x32: sll(D); cycles += 8; break;
                    case 0x33: sll(E); cycles += 8; break;
                    case 0x34: sll(H); cycles += 8; break;
                    case 0x35: sll(L); cycles += 8; break;
                    case 0x36: {
                        uint8_t hlValue = z80_mem_read(HL());
                        sll(hlValue);
                        z80_mem_write(HL(), hlValue);
                        cycles += 15;
                        break;
                    }
                    case 0x37: sll(A); cycles += 8; break;

                    default:
                        cerr << "Unimplemented CB-prefixed opcode: " << hex << (int)subOpcode << endl;
                        break;
                }
                break;
            }

            default:
                cerr << "Unimplemented opcode at PC: " << hex << PC - 1 << ", Opcode: " << (int)opcode << endl;
        }
    }
}
