// z80.h
#ifndef Z80_H
#define Z80_H

#include <cstdint>
#include <iostream>

// Declaration of the Z80 CPU Emulator
class Z80 {
public:
    // Registers
    uint8_t A, F; // Accumulator and Flags
    uint8_t B, C; // BC Register Pair
    uint8_t D, E; // DE Register Pair
    uint8_t H, L; // HL Register Pair
    uint16_t PC;  // Program Counter
    uint16_t SP;  // Stack Pointer
    int cycles;   // Total cycles executed
    uint8_t I, R; // Interrupt and Refresh Registers
    bool halt;    // HALT state
    uint16_t IX;  // Index Register IX
    uint16_t IY;  // Index Register IY
    uint16_t hiddenMathRegister; // Hidden 16-bit math register

    uint8_t A_, F_; // Shadow accumulator and flags
    uint8_t B_, C_; // Shadow BC
    uint8_t D_, E_; // Shadow DE
    uint8_t H_, L_; // Shadow HL

    uint8_t interruptMode = 0; // Interrupt Mode

    // Flag constants
    const uint8_t FLAG_Z = 0x80;  // Zero Flag
    const uint8_t FLAG_N = 0x40;  // Add/Subtract Flag
    const uint8_t FLAG_H = 0x20;  // Half Carry Flag
    const uint8_t FLAG_C = 0x10;  // Carry Flag
    const uint8_t FLAG_PV = 0x04; // Parity/Overflow Flag
    const uint8_t FLAG_S = 0x02;  // Sign Flag

    bool IFF1 = false; // Interrupt Flip-Flop 1
    bool IFF2 = false; // Interrupt Flip-Flop 2

    // Constructor and methods
    Z80();
    void init();
    uint8_t fetch();
    uint16_t fetch16();
    void execute(uint8_t opcode); // Declaration of the execute function
    int run(int maxCycles);

private:
    void loadImmediate(uint8_t &reg);
    void LD(uint8_t &dest, uint8_t src);
    void printState() const;
    void setFlag(uint8_t flag);
    void clearFlag(uint8_t flag);
    bool isFlagSet(uint8_t flag);
    uint8_t add_flag(uint8_t a, uint8_t value);
    uint16_t add_flag16(uint16_t a, uint16_t value);
    uint8_t adc_flag(uint8_t a, uint8_t value);

    // Add definitions for 16-bit register pairs
    uint16_t BC() const { return (B << 8) | C; } // BC register pair
    uint16_t DE() const { return (D << 8) | E; } // DE register pair
    uint16_t HL() const { return (H << 8) | L; } // HL register pair

    uint16_t BC_() const { return (B_ << 8) | C_; } // BC_ register pair
    uint16_t DE_() const { return (D_ << 8) | E_; } // DE register pair
    uint16_t HL_() const { return (H_ << 8) | L_; } // HL register pair

    void setBC(uint16_t value)
    {
        B = (value >> 8) & 0xFF;
        C = value & 0xFF;
    }
    void setDE(uint16_t value)
    {
        D = (value >> 8) & 0xFF;
        E = value & 0xFF;
    }
    void setHL(uint16_t value)
    {
        H = (value >> 8) & 0xFF;
        L = value & 0xFF;
    }

    void setBC_(uint16_t value)
    {
        B_ = (value >> 8) & 0xFF;
        C_ = value & 0xFF;
    }
    void setDE_(uint16_t value)
    {
        D_ = (value >> 8) & 0xFF;
        E_ = value & 0xFF;
    }
    void setHL_(uint16_t value)
    {
        H_ = (value >> 8) & 0xFF;
        L_ = value & 0xFF;
    }
    uint16_t adc_flag16(uint16_t a, uint16_t value);
    // Flag handler for subtraction
    uint8_t sub_flag(uint8_t a, uint8_t value);
    uint8_t sbc_flag(uint8_t a, uint8_t value);
    uint16_t sbc_flag16(uint16_t a, uint16_t value);
    void NEG();
    void OR(uint8_t value);
    bool checkParity(uint8_t value);
    void PUSH_IX();
    void PUSH_IY();
    void resMem(uint8_t bit, uint16_t addr);
    void res(uint8_t bit, uint8_t &reg);
    void RET_Implementation();
    void rlMem(uint16_t addr);
    void rl(uint8_t &reg);
    void rla();
    void rlcMemHL();
    void rlc(uint8_t &reg);
    void rld();
    void rr(uint8_t &reg);
    void rrMem(uint16_t addr);
    void rra();
    void rrc(uint8_t &reg);
    void rrcMem(uint16_t addr);
    void rrd();
    void rst(uint8_t opcode);
    void setBit(uint8_t &reg, uint8_t bit);
    void sla(uint8_t &reg);
    void slaMem(uint16_t address);
    void sra(uint8_t &reg);
    void sraMem(uint16_t address);
    void srl(uint8_t &reg);
    void xorReg(uint8_t reg);
    void and_r(uint8_t reg);
    void and_n();
    void and_hl();
    void and_ix_d();
    void and_iy_d();
    void bit(uint8_t bit, uint8_t reg);
    uint8_t& decodeRegister(uint8_t regCode);
    void call_nn();
    void ccf();
    void cp(uint8_t value);
    void cpd();
    void zdec(uint8_t &value);
    void zinc(uint8_t &reg);
    void sll(uint8_t &reg);
};

#endif
