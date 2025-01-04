#include <iostream>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include "z80.h"
using namespace std;

// Flag constants
const uint8_t FLAG_Z = 0x80; // Zero Flag
const uint8_t FLAG_N = 0x40; // Add/Subtract flag
const uint8_t FLAG_H = 0x20; // Half Carry Flag
const uint8_t FLAG_C = 0x10; // Carry Flag
const uint8_t FLAG_PV = 0x04; // Parity/Overflow Flag
const uint8_t FLAG_S = 0x02;  // Sign Flag

// Memory functions provided by the instructor
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

// Constructor
Z80::Z80() : A(0), F(0), B(0), C(0), D(0), E(0), H(0), L(0), PC(0x0000), SP(0xFFFF), I(0), R(0), cycles(0), halt(false), IX(0), IY(0) {}

// Initialize CPU
void Z80::init()
{
    A = F = 0;
    B = C = 0;
    D = E = 0;
    H = L = 0;
    IX = 0xFFFF;
    IY = 0xFFFF;
    PC = 0x0000;
    SP = 0x0000;
    I = R = 0;
    cycles = 0;
    halt = false;
    A_ = F_ = 0;
    B_ = C_ = 0;
    D_ = E_ = 0;
    H_ = L_ = 0;
    halt = false;
    hiddenMathRegister = 0;
}

// Fetch the next opcode
uint8_t Z80::fetch()
{
    return z80_mem_read(PC++);
}

uint16_t Z80::fetch16()
{
    uint8_t low = fetch();    // Fetch the low byte
    uint8_t high = fetch();   // Fetch the high byte
    return (high << 8) | low; // Combine high and low into a 16-bit value
}

// Print the CPU state (for debugging or HALT)
void Z80::printState() const
{
    cout << "CPU State:\n";
    cout << "A: " << hex << (int)A << " F: " << int(F) << endl;
    cout << "B: " << int(B) << " C: " << int(C) << endl;
    cout << "D: " << int(D) << " E: " << int(E) << endl;
    cout << "H: " << int(H) << " L: " << int(L) << endl;
    cout << "I: " << int(I) << " R: " << int(R) << endl;
    cout << "A': " << int(A_) << " F': " << int(F_) << endl;
    cout << "B': " << int(B_) << " C': " << int(C_) << endl;
    cout << "D': " << int(D_) << " E': " << int(E_) << endl;
    cout << "IFF1: " << int(IFF1) << " IFF2: " << int(IFF2) << endl;
    cout << "IM: " << int(interruptMode) << endl;
    cout << "Hidden 16-bit math register: " << int(hiddenMathRegister) << endl;
    cout << "IX: " << int(IX) << " IY: " << int(IY) << endl;
    cout << "PC: " << int(PC) << endl;
    cout << " SP: " << int(SP) << endl;

    

    

    // Decode and print flags
    cout << "Flags: ";
    cout << "S=" << ((F & FLAG_S) ? "1" : "0") << " ";
    cout << "Z=" << ((F & FLAG_Z) ? "1" : "0") << " ";
    cout << "H=" << ((F & FLAG_H) ? "1" : "0") << " ";
    cout << "PV=" << ((F & FLAG_PV) ? "1" : "0") << " ";
    cout << "N=" << ((F & FLAG_N) ? "1" : "0") << " ";
    cout << "C=" << ((F & FLAG_C) ? "1" : "0") << "\n";
}

// Run the CPU for a given number of cycles
int Z80::run(int maxCycles)
{
    int executedCycles = 0;
    while (executedCycles < maxCycles && !halt)
    {
        cycles = 0; // Reset cycles before executing each instruction
        uint8_t opcode = fetch();
        execute(opcode);
        executedCycles += cycles;

        if (opcode == 0x76)
        { // HALT
            break;
        }
    }

    return executedCycles;
}

void Z80::loadImmediate(uint8_t &reg)
{
    reg = fetch(); // Use fetch() to load the immediate value following the opcode
}

void Z80::LD(uint8_t &dest, uint8_t src)
{
    dest = src;
}

uint8_t Z80::add_flag(uint8_t a, uint8_t value)
{
    uint16_t result = (uint16_t)a + (uint16_t)value;

    // Clear existing flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);

    // Update flags
    if ((result & 0xFF) == 0)
        setFlag(FLAG_Z); // Zero flag
    if (result & 0x80)
        setFlag(FLAG_S); // Sign flag
    if (((a & 0xF) + (value & 0xF)) & 0x10)
        setFlag(FLAG_H); // Half-carry flag
    if (((a ^ value ^ result) & 0x80) == 0x80)
        setFlag(FLAG_PV); // Overflow flag
    if (result > 0xFF)
        setFlag(FLAG_C); // Carry flag

    return result & 0xFF; // Return the lower 8 bits of the result
}

// Flag handler for subtraction
uint8_t Z80::sub_flag(uint8_t a, uint8_t value)
{
    int16_t result = (int16_t)a - (int16_t)value;

    // Clear all affected flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);

    // Update the flags
    setFlag(FLAG_N); // Subtraction flag is always 
    if ((result & 0xFF) == 0)
        setFlag(FLAG_Z); // Zero flag
    if (result & 0x80)
        setFlag(FLAG_S); // Sign flag
    if ((a & 0xF) < (value & 0xF))
        setFlag(FLAG_H); // Half-borrow flag
    if (((a ^ value) & (a ^ result) & 0x80) != 0)
        setFlag(FLAG_PV); // Overflow flag
    if (result < 0)
        setFlag(FLAG_C); // Borrow (Carry) flag

    return result & 0xFF; // Return the 8-bit result
}

uint16_t Z80::add_flag16(uint16_t a, uint16_t value) {
    uint32_t result = (uint32_t)a + (uint32_t)value;

    // Update the Half-carry flag (Carry from bit 11)
    if (((a & 0x0FFF) + (value & 0x0FFF)) & 0x1000) {
        setFlag(FLAG_H);
    } else {
        clearFlag(FLAG_H);
    }

    // Update the Carry flag (Carry from bit 15)
    if (result > 0xFFFF) {
        setFlag(FLAG_C);
    } else {
        clearFlag(FLAG_C);
    }

    // Reset the N flag (this is always cleared in ADD instructions)
    clearFlag(FLAG_N);

    return result & 0xFFFF; // Return the lower 16 bits
}

uint16_t Z80::adc_flag16(uint16_t a, uint16_t value)
{
    uint32_t carry = isFlagSet(FLAG_C) ? 1 : 0; // Get the carry flag (0 or 1)
    uint32_t result = (uint32_t)a + (uint32_t)value + carry;

    // Clear and update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);

    if ((result & 0xFFFF) == 0)
        setFlag(FLAG_Z); // Zero flag
    if (result & 0x8000)
        setFlag(FLAG_S); // Sign flag
    if (((a & 0xFFF) + (value & 0xFFF) + carry) & 0x1000)
        setFlag(FLAG_H); // Half-carry flag
    if (((a ^ value ^ result) & 0x8000) == 0x8000)
        setFlag(FLAG_PV); // Overflow flag
    if (result > 0xFFFF)
        setFlag(FLAG_C); // Carry flag
    
    // Reset the N flag (addition clears this flag)
    clearFlag(FLAG_N);

    return result & 0xFFFF; // Return the lower 16 bits of the result
}

uint8_t Z80::adc_flag(uint8_t a, uint8_t value)
{
    // Retrieve the current carry flag (0 or 1)
    uint16_t carry = isFlagSet(FLAG_C) ? 1 : 0;

    // Perform the addition with the carry
    uint16_t result = (uint16_t)a + (uint16_t)value + carry;

    // Clear all relevant flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);

    // Update flags
    if ((result & 0xFF) == 0)
        setFlag(FLAG_Z); // Zero flag (result is zero)
    if (result & 0x80)
        setFlag(FLAG_S); // Sign flag (bit 7 of the result is set)
    if (((a & 0xF) + (value & 0xF) + carry) & 0x10)
        setFlag(FLAG_H); // Half-carry flag (carry from bit 3)
    if (((a ^ value ^ result) & 0x80) == 0x80)
        setFlag(FLAG_PV); // Parity/Overflow flag (signed overflow)
    if (result > 0xFF)
        setFlag(FLAG_C); // Carry flag (carry out of bit 7)

    // Reset the N flag (addition clears this flag)
    clearFlag(FLAG_N);

    // Return the lower 8 bits of the result
    return (uint8_t)(result & 0xFF);
}

uint16_t Z80::sbc_flag16(uint16_t a, uint16_t value)
{
    int32_t carry = isFlagSet(FLAG_C) ? 1 : 0;
    int32_t result = (int32_t)a - (int32_t)value - carry;

    // Clear all affected flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);

    // Update flags
    setFlag(FLAG_N); // Subtraction flag is always set
    if ((result & 0xFFFF) == 0)
        setFlag(FLAG_Z); // Zero flag
    if (result & 0x8000)
        setFlag(FLAG_S); // Sign flag
    if ((a & 0xFFF) < ((value & 0xFFF) + carry))
        setFlag(FLAG_H); // Half-borrow flag
    if (((a ^ value) & (a ^ result) & 0x8000) != 0)
        setFlag(FLAG_PV); // Overflow flag
    if (result < 0)
        setFlag(FLAG_C); // Borrow (Carry) flag

    return result & 0xFFFF; // Return the 16-bit result
}

uint8_t Z80::sbc_flag(uint8_t a, uint8_t value)
{
    int16_t carry = isFlagSet(FLAG_C) ? 1 : 0;
    int16_t result = (int16_t)a - (int16_t)value - carry;

    // Clear all affected flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);

    // Update flags
    setFlag(FLAG_N); // Subtraction flag is always set
    if ((result & 0xFF) == 0)
        setFlag(FLAG_Z); // Zero flag
    if (result & 0x80)
        setFlag(FLAG_S); // Sign flag
    if ((a & 0xF) < ((value & 0xF) + carry))
        setFlag(FLAG_H); // Half-borrow flag
    if (((a ^ value) & (a ^ result) & 0x80) != 0)
        setFlag(FLAG_PV); // Overflow flag
    if (result < 0)
        setFlag(FLAG_C); // Borrow (Carry) flag

    return result & 0xFF; // Return the 8-bit result
}

void Z80::NEG()
{
    // Perform the operation: A ← 0 - A
    uint8_t originalA = A;
    uint16_t result = 0 - A;

    // Update the A register with the result
    A = result & 0xFF;

    // Clear all affected flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);

    // Set the N flag because this is a subtraction operation
    setFlag(FLAG_N);

    // Update flags based on the result
    if (A & 0x80) {
        setFlag(FLAG_S); // Sign flag
    }
    if (A == 0) {
        setFlag(FLAG_Z); // Zero flag
    }
    if ((originalA & 0xF) > 0) {
        setFlag(FLAG_H); // Half-borrow flag
    }
    if (originalA == 0x80) {
        setFlag(FLAG_PV); // Overflow flag for 0x80
    }
    if (originalA != 0) {
        setFlag(FLAG_C); // Carry (borrow) flag is set if A was not 0
    }
}

bool Z80::checkParity(uint8_t value)
{
    int parity = 0;
    while (value)
    {
        parity ^= (value & 1);
        value >>= 1;
    }
    return (parity == 0); // Returns true for even parity
}

void Z80::OR(uint8_t value)
{
    uint8_t result = A | value; // Perform logical OR operation
    A = result;                 // Store result in accumulator (A)

    // Clear/reset flags
    clearFlag(FLAG_H | FLAG_N | FLAG_C | FLAG_PV | FLAG_S | FLAG_Z); // H, N, and C are reset

    // Update S, Z, and P/V flags based on the result
    if (result == 0)
        setFlag(FLAG_Z); // Zero flag
    if (result & 0x80)
        setFlag(FLAG_S); // Sign flag (negative if MSB is set)
    if (checkParity(result))
        setFlag(FLAG_PV); // Parity flag

    // The H flag is explicitly cleared (already handled by `clearFlag`).
}

void Z80::PUSH_IX() {
    // Decrement SP and write the high byte of IX to memory
    SP--;
    z80_mem_write(SP, (IX >> 8) & 0xFF); // Write IXH (high byte)

    // Decrement SP again and write the low byte of IX to memory
    SP--;
    z80_mem_write(SP, IX & 0xFF); // Write IXL (low byte)

    // Update cycles for PUSH IX instruction (15 T states)
    cycles += 15;
}

void Z80::PUSH_IY()
{
    // Decrement SP and write the high byte of IX to memory
    SP--;
    z80_mem_write(SP, (IY >> 8) & 0xFF); // Write IXH (high byte)

    // Decrement SP again and write the low byte of IX to memory
    SP--;
    z80_mem_write(SP, IY & 0xFF); // Write IXL (low byte)

    // Update cycles for PUSH IX instruction (15 T states)
    cycles += 15;
}

void Z80::res(uint8_t bit, uint8_t &reg)
{
    reg &= ~(1 << bit); // Reset the specified bit in the register
    cycles += 8;
}

void Z80::resMem(uint8_t bit, uint16_t addr) {
    uint8_t value = z80_mem_read(addr);
    value &= ~(1 << bit);  // Reset the specified bit in memory
    z80_mem_write(addr, value);
}

void Z80::RET_Implementation()
{
    uint8_t low = z80_mem_read(SP);
    SP++;
    uint8_t high = z80_mem_read(SP);
    SP++;
    PC = (high << 8) | low; // Combine the high and low bytes into the new PC value
}

void Z80::rla()
{
    // Extract the current Carry flag
    uint8_t oldCarry = isFlagSet(FLAG_C) ? 1 : 0;

    // Extract bit 7 (new Carry flag)
    uint8_t newCarry = (A & 0x80) >> 7;

    // Rotate the value left through the Carry flag
    A = (A << 1) | oldCarry;

    // Update flags
    clearFlag(FLAG_H | FLAG_N | FLAG_C); // H and N are always reset, clear C to update
    if (newCarry)
    {
        setFlag(FLAG_C); // Set Carry flag if bit 7 was 1
    }

    // Note: S, Z, and P/V flags are NOT affected by this operation
}

void Z80::rra()
{
    // Extract the current Carry flag
    uint8_t oldCarry = isFlagSet(FLAG_C) ? 0x80 : 0x00; // If Carry is set, prepare to rotate into bit 7

    // Extract bit 0 (new Carry flag)
    uint8_t newCarry = A & 0x01; // Save the least significant bit

    // Rotate the value right through Carry
    A = (A >> 1) | oldCarry;

    // Update flags
    clearFlag(FLAG_H | FLAG_N | FLAG_C); // H and N are always reset, clear C to update
    if (newCarry)
    {
        setFlag(FLAG_C); // Set Carry flag if bit 0 was 1
    }

    // Note: S, Z, and P/V flags are NOT affected by this operation
}

void Z80::rr(uint8_t &reg)
{
    // Save current Carry flag
    uint8_t oldCarry = isFlagSet(FLAG_C) ? 0x80 : 0;

    // Extract bit 0 (new Carry flag)
    uint8_t newCarry = reg & 0x01;

    // Rotate the value right through the Carry flag
    reg = (reg >> 1) | oldCarry;

    // Update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);
    if (newCarry)
        setFlag(FLAG_C); // Set Carry flag if bit 0 was 1
    if (reg & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    if (reg == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(reg))
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
}

void Z80::rrMem(uint16_t addr)
{
    // Read value from memory
    uint8_t value = z80_mem_read(addr);

    // Perform RL operation
    rr(value);

    // Write the rotated value back to memory
    z80_mem_write(addr, value);
}

void Z80::rl(uint8_t &reg)
{
    // Save current Carry flag
    uint8_t oldCarry = isFlagSet(FLAG_C) ? 1 : 0;

    // Extract bit 7 (new Carry flag)
    uint8_t newCarry = (reg & 0x80) >> 7;

    // Rotate the value left through the Carry flag
    reg = (reg << 1) | oldCarry;

    // Update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);
    if (newCarry)
        setFlag(FLAG_C); // Set Carry flag if bit 7 was 1
    if (reg & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    if (reg == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(reg))
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
}



void Z80::rlMem(uint16_t addr)
{
    // Read value from memory
    uint8_t value = z80_mem_read(addr);

    // Perform RL operation
    rl(value);

    // Write the rotated value back to memory
    z80_mem_write(addr, value);
}

void Z80::rlcMemHL()
{
    // Read the value from memory at HL
    uint8_t value = z80_mem_read(HL());

    // Extract bit 7 as the new Carry flag
    uint8_t newCarry = (value & 0x80) >> 7;

    // Rotate the value left, with bit 7 moving to bit 0
    value = (value << 1) | newCarry;

    // Write the rotated value back to memory
    z80_mem_write(HL(), value);

    // Update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Clear all affected flags
    if (value & 0x80)
    {
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    }
    if (value == 0)
    {
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    }
    if (checkParity(value))
    {
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
    }
    if (newCarry)
    {
        setFlag(FLAG_C); // Set Carry flag if bit 7 was 1
    }
}

void Z80::rlc(uint8_t &reg)
{
    // Extract bit 7 for Carry
    uint8_t bit7 = (reg & 0x80) >> 7;

    // Perform Rotate Left Circular
    reg = (reg << 1) | bit7;

    // Update Flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Reset relevant flags
    if (reg & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    if (reg == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(reg))
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
    if (bit7)
        setFlag(FLAG_C); // Set Carry flag if bit 7 was 1
}

void Z80::rrc(uint8_t &reg)
{
    // Extract bit 0 for Carry
    uint8_t bit0 = reg & 0x01;

    // Perform Rotate Right Circular
    reg = (reg >> 1) | (bit0 << 7);

    // Update Flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Reset relevant flags
    if (reg & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    if (reg == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(reg))
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
    if (bit0)
        setFlag(FLAG_C); // Set Carry flag if bit 0 was 1
}

void Z80::rrcMem(uint16_t addr)
{
    // Read the value from memory
    uint8_t value = z80_mem_read(addr);

    // Extract bit 0 for Carry
    uint8_t bit0 = value & 0x01;

    // Perform Rotate Right Circular
    value = (value >> 1) | (bit0 << 7);

    // Write the modified value back to memory
    z80_mem_write(addr, value);

    // Update Flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Reset relevant flags
    if (value & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    if (value == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(value))
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
    if (bit0)
        setFlag(FLAG_C); // Set Carry flag if bit 0 was 1
}

void Z80::rld()
{
    // Read the memory content at the address specified by HL
    uint8_t hlValue = z80_mem_read(HL());

    // Extract nibbles
    uint8_t highNibbleHL = (hlValue & 0xF0) >> 4; // High nibble of memory
    uint8_t lowNibbleHL = hlValue & 0x0F;         // Low nibble of memory
    uint8_t lowNibbleA = A & 0x0F;                // Low nibble of Accumulator (A)

    // Perform rotation
    A = (A & 0xF0) | highNibbleHL;             // High nibble of A remains, replace low nibble with high nibble of memory
    hlValue = (lowNibbleHL << 4) | lowNibbleA; // Combine rotated nibbles for memory

    // Write back the modified value to memory
    z80_mem_write(HL(), hlValue);

    // Update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N); // Reset flags
    if (A & 0x80)
        setFlag(FLAG_S); // Set Sign flag if A is negative
    if (A == 0)
        setFlag(FLAG_Z); // Set Zero flag if A is zero
    if (checkParity(A))
        setFlag(FLAG_PV); // Set Parity/Overflow flag if parity is even

    // H is reset, N is reset, and C is not affected
    cycles += 18; // RLD takes 18 T-states
}

void Z80::rrd()
{
    // Read the memory content at the address specified by HL
    uint8_t hlValue = z80_mem_read(HL());

    // Extract nibbles
    uint8_t highNibbleHL = (hlValue & 0xF0) >> 4; // High nibble of memory
    uint8_t lowNibbleHL = hlValue & 0x0F;         // Low nibble of memory
    uint8_t lowNibbleA = A & 0x0F;                // Low nibble of Accumulator (A)

    // Perform rotation
    A = (A & 0xF0) | lowNibbleHL;               // High nibble of A remains, replace low nibble with low nibble of memory
    hlValue = (lowNibbleA << 4) | highNibbleHL; // Combine rotated nibbles for memory

    // Write back the modified value to memory
    z80_mem_write(HL(), hlValue);

    // Update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N); // Reset flags
    if (A & 0x80)
        setFlag(FLAG_S); // Set Sign flag if A is negative
    if (A == 0)
        setFlag(FLAG_Z); // Set Zero flag if A is zero
    if (checkParity(A))
        setFlag(FLAG_PV); // Set Parity/Overflow flag if parity is even

    // H is reset, N is reset, and C is not affected
    cycles += 18; // RRD takes 18 T-states
}

void Z80::rst(uint8_t opcode)
{
    // Extract the restart address from the opcode
    uint16_t rstAddress = opcode & 0x38; // p is encoded in bits 3, 4, and 5 of the opcode

    // Push the current PC onto the stack
    SP -= 1;                             // Decrement stack pointer
    z80_mem_write(SP, (PC >> 8) & 0xFF); // Push high byte of PC
    SP -= 1;
    z80_mem_write(SP, PC & 0xFF); // Push low byte of PC


    // Set the new PC to the restart address
    PC = rstAddress;

    // The RST instruction takes 11 T-states
    cycles += 11;
}

void Z80::sla(uint8_t &reg)
{
    // Extract bit 7 for Carry flag
    uint8_t bit7 = (reg & 0x80) >> 7;

    // Perform arithmetic shift left
    reg <<= 1;

    // Update Flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Reset relevant flags
    if (reg & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    if (reg == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(reg))
        setFlag(FLAG_PV); // Set Parity flag for even parity
    if (bit7)
        setFlag(FLAG_C); // Set Carry flag if original bit 7 was 1
}

void Z80::slaMem(uint16_t address)
{
    // Read the value from memory
    uint8_t value = z80_mem_read(address);

    // Extract bit 7 for Carry flag
    uint8_t bit7 = (value & 0x80) >> 7;

    // Perform arithmetic shift left
    value <<= 1;

    // Write back the modified value to memory
    z80_mem_write(address, value);

    // Update Flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Reset relevant flags
    if (value & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    if (value == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(value))
        setFlag(FLAG_PV); // Set Parity flag for even parity
    if (bit7)
        setFlag(FLAG_C); // Set Carry flag if original bit 7 was 1
}

void Z80::setBit(uint8_t &reg, uint8_t bit)
{
    reg |= (1 << bit); // Set the specified bit in the register
}

void Z80::sra(uint8_t &reg)
{
    // Extract bit 0 for Carry
    uint8_t bit0 = reg & 0x01;

    // Preserve the state of bit 7 (sign bit) and shift right
    uint8_t signBit = reg & 0x80;
    reg = (reg >> 1) | signBit;

    // Update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);
    if (bit0)
    {
        setFlag(FLAG_C); // Set Carry flag if bit 0 was 1
    }
    if (reg & 0x80)
    {
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    }
    if (reg == 0)
    {
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    }
    if (checkParity(reg))
    {
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
    }
}

void Z80::srl(uint8_t &reg)
{
    // Extract bit 0 for Carry
    uint8_t bit0 = reg & 0x01;

    // Perform Logical Shift Right
    reg >>= 1;

    // Update Flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Reset all relevant flags
    if (bit0)
        setFlag(FLAG_C); // Set Carry flag if bit 0 was 1
    if (reg == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(reg))
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
}

void Z80::sll(uint8_t &reg)
{
    // Extract bit 7 for Carry
    uint8_t bit7 = (reg & 0x80) >> 7;

    // Perform Logical Shift Left
    reg = (reg << 1) | 0x01; // Shift left and set bit 0 to 1

    // Update Flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C); // Reset all relevant flags
    if (bit7)
        setFlag(FLAG_C); // Set Carry flag if bit 7 was 1
    if (reg == 0)
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    if (checkParity(reg))
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
    if (reg & 0x80)
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
}

void Z80::sraMem(uint16_t address)
{
    // Read the value from memory
    uint8_t value = z80_mem_read(address);

    // Extract bit 0 for Carry
    uint8_t bit0 = value & 0x01;

    // Preserve the state of bit 7 (sign bit) and shift right
    uint8_t signBit = value & 0x80;
    value = (value >> 1) | signBit;

    // Write the modified value back to memory
    z80_mem_write(address, value);

    // Update flags
    clearFlag(FLAG_S | FLAG_Z | FLAG_H | FLAG_PV | FLAG_N | FLAG_C);
    if (bit0)
    {
        setFlag(FLAG_C); // Set Carry flag if bit 0 was 1
    }
    if (value & 0x80)
    {
        setFlag(FLAG_S); // Set Sign flag if bit 7 of the result is 1
    }
    if (value == 0)
    {
        setFlag(FLAG_Z); // Set Zero flag if result is 0
    }
    if (checkParity(value))
    {
        setFlag(FLAG_PV); // Set Parity/Overflow flag for even parity
    }
}

void Z80::xorReg(uint8_t reg)
{
    // Perform XOR operation
    A ^= reg;

    // Update flags
    clearFlag(FLAG_H | FLAG_N | FLAG_C); // H, N, and C are reset
    if (A & 0x80)
        setFlag(FLAG_S); // Set Sign flag if the result is negative
    else
        clearFlag(FLAG_S);
    if (A == 0)
        setFlag(FLAG_Z); // Set Zero flag if the result is zero
    else
        clearFlag(FLAG_Z);
    if (checkParity(A))
        setFlag(FLAG_PV); // Set Parity/Overflow flag if parity is even
    else
        clearFlag(FLAG_PV);
}

void Z80::and_r(uint8_t reg)
{
    A &= reg; // Perform AND operation

    // Update flags
    clearFlag(FLAG_C | FLAG_N); // Clear C and N flags
    setFlag(FLAG_H);            // H flag is always set
    if (A & 0x80)
        setFlag(FLAG_S); // Set S flag if result is negative
    else
        clearFlag(FLAG_S);

    if (A == 0)
        setFlag(FLAG_Z); // Set Z flag if result is zero
    else
        clearFlag(FLAG_Z);

    if (checkParity(A))
        setFlag(FLAG_PV); // Set P/V flag for even parity
    else
        clearFlag(FLAG_PV);
}

void Z80::and_n()
{
    uint8_t immediate = fetch(); // Fetch immediate value
    and_r(immediate);
    cycles += 7;
}

void Z80::and_hl()
{
    uint8_t value = z80_mem_read(HL()); // Read value from memory at HL
    and_r(value);
    cycles += 7;
}

void Z80::and_ix_d()
{
    int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
    uint16_t address = IX + displacement;               // Compute effective address
    uint8_t value = z80_mem_read(address);              // Read value from memory
    and_r(value);
    cycles += 19;
}

void Z80::and_iy_d()
{
    int8_t displacement = static_cast<int8_t>(fetch()); // Fetch signed displacement
    uint16_t address = IY + displacement;               // Compute effective address
    uint8_t value = z80_mem_read(address);              // Read value from memory
    and_r(value);
    cycles += 19;
}

void Z80::bit(uint8_t bit, uint8_t reg)
{
    // Test the specified bit in the register
    uint8_t mask = 1 << bit;
    bool isBitSet = (reg & mask) != 0;

    // Update the Z flag
    if (isBitSet)
    {
        clearFlag(FLAG_Z); // Reset Z flag if the bit is set
    }
    else
    {
        setFlag(FLAG_Z); // Set Z flag if the bit is 0
    }

    // Set the H flag (always set for BIT instruction)
    setFlag(FLAG_H);

    // Reset the N flag (always reset for BIT instruction)
    clearFlag(FLAG_N);

    // Note: S flag is unknown, and C is not affected
    // The Parity/Overflow (P/V) flag behavior is unknown
    // Implement this based on specific Z80 emulator requirements.
}

uint8_t& Z80::decodeRegister(uint8_t regCode)
{
    switch (regCode)
    {
        case 0x00: return B; // Register B
        case 0x01: return C; // Register C
        case 0x02: return D; // Register D
        case 0x03: return E; // Register E
        case 0x04: return H; // Register H
        case 0x05: return L; // Register L
        case 0x07: return A; // Register A
        case 0x06:           // (HL) refers to the memory address specified by HL
        {
            static uint8_t hlMemoryValue; // Temporary storage
            hlMemoryValue = z80_mem_read(HL());
            return hlMemoryValue;
        }
        default: throw std::runtime_error("Invalid register code");
    }
}

void Z80::call_nn()
{
    // Fetch the 16-bit address (nn) from the next two memory locations
    uint16_t address = fetch16();

    // Push the current PC onto the stack
    SP -= 1;                             // Decrement SP for high byte
    z80_mem_write(SP, (PC >> 8) & 0xFF); // Write the high byte of PC
    SP -= 1;                             // Decrement SP for low byte
    z80_mem_write(SP, PC & 0xFF);        // Write the low byte of PC

    // Set PC to the fetched address
    PC = address;

    // Update cycles
    cycles += 17; // CALL nn takes 17 T-states
    // printState();
}

void Z80::ccf()
{
    // Store the current carry flag
    bool carrySet = isFlagSet(FLAG_C);

    // Invert the carry flag
    if (carrySet)
        clearFlag(FLAG_C);
    else
        setFlag(FLAG_C);

    // Copy the previous carry flag into the H flag
    if (carrySet)
        setFlag(FLAG_H);
    else
        clearFlag(FLAG_H);

    // Reset the N flag
    clearFlag(FLAG_N);

    // Other flags are unaffected (S, Z, and P/V are unchanged)

    // Increment cycles
    cycles += 4; // CCF takes 4 T-states
}

void Z80::cp(uint8_t value) {
    uint8_t result = A - value;

    // Update flags
    setFlag(FLAG_N); // N flag is always set for CP (subtraction)

    // Set S (sign flag)
    if (result & 0x80) {
        setFlag(FLAG_S);
    } else {
        clearFlag(FLAG_S);
    }

    // Set Z (zero flag)
    if (result == 0) {
        setFlag(FLAG_Z);
    } else {
        clearFlag(FLAG_Z);
    }

    // Set H (half-carry flag)
    if (((A & 0x0F) - (value & 0x0F)) < 0) {
        setFlag(FLAG_H);
    } else {
        clearFlag(FLAG_H);
    }

    // Set P/V (overflow flag)
    if (((A ^ value) & 0x80) && ((A ^ result) & 0x80)) {
        setFlag(FLAG_PV);
    } else {
        clearFlag(FLAG_PV);
    }

    // Set C (carry flag)
    if (A < value) {
        setFlag(FLAG_C);
    } else {
        clearFlag(FLAG_C);
    }
}

void Z80::cpd()
{
    // Compare A with the memory at address HL
    uint8_t value = z80_mem_read(HL());
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
        clearFlag(FLAG_PV);

    // S and C flags are not affected

    // Decrement HL and BC
    setHL(HL() - 1);
    setBC(BC() - 1);

    // Consume cycles
    cycles += 16; // CPD takes 16 T-states
}

void Z80::zdec(uint8_t &value)
{
    // Calculate the result
    uint8_t result = value - 1;

    // Update flags
    if (result & 0x80)
        setFlag(FLAG_S); // Set S (sign flag) if the result is negative
    else
        clearFlag(FLAG_S);

    if (result == 0)
        setFlag(FLAG_Z); // Set Z if the result is zero
    else
        clearFlag(FLAG_Z);

    if ((value & 0x0F) == 0)
        setFlag(FLAG_H); // Set H if borrow occurs from bit 4
    else
        clearFlag(FLAG_H);

    if (value == 0x80)
        setFlag(FLAG_PV); // Set PV if the original value was 0x80
    else
        clearFlag(FLAG_PV);

    setFlag(FLAG_N); // N is always set for DEC

    // Write back the result
    value = result;
}

void Z80::zinc(uint8_t &reg)
{
    uint8_t originalValue = reg;
    reg += 1;

    // Update flags
    if (reg == 0)
    {
        setFlag(FLAG_Z); // Z is set if result is zero
    }
    else
    {
        clearFlag(FLAG_Z);
    }

    if (reg & 0x80)
    {
        setFlag(FLAG_S); // S is set if result is negative
    }
    else
    {
        clearFlag(FLAG_S);
    }

    if ((originalValue & 0xF) == 0xF)
    {
        setFlag(FLAG_H); // H is set if there is a carry from bit 3
    }
    else
    {
        clearFlag(FLAG_H);
    }

    if (originalValue == 0x7F)
    {
        setFlag(FLAG_PV); // P/V is set if the original value was 0x7F
    }
    else
    {
        clearFlag(FLAG_PV);
    }

    clearFlag(FLAG_N); // N is always reset for INC
}

// Flag manipulation helpers
void Z80::setFlag(uint8_t flag) { F |= flag; }
void Z80::clearFlag(uint8_t flag) { F &= ~flag; }
bool Z80::isFlagSet(uint8_t flag) { return (F & flag) != 0; }

// Global instance of the emulator
Z80 cpu;

// C-Compatible Functions
extern "C" void z80_init() {
    cpu.init();
}

extern "C" int z80_execute(int cycles) {
    return cpu.run(cycles);
}