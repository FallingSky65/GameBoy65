#include <cstdint>
#include <iostream>

typedef uint8_t u8;
typedef uint16_t u16;

class CPU {
public:
  struct Registers {
    union { u16 af; struct { u8 f, a; }; };
    union { u16 bc; struct { u8 c, b; }; };
    union { u16 de; struct { u8 e, d; }; };
    union { u16 hl; struct { u8 l, h; }; };

    void      zero(bool set) { f &= 0b01111111; if (set) f |= 0b10000000; }
    void  subtract(bool set) { f &= 0b10111111; if (set) f |= 0b01000000; }
    void halfcarry(bool set) { f &= 0b11011111; if (set) f |= 0b00100000; }
    void     carry(bool set) { f &= 0b11101111; if (set) f |= 0b00010000; }

    bool      zero() { return f & 0b10000000; }
    bool  subtract() { return f & 0b01000000; }
    bool halfcarry() { return f & 0b00100000; }
    bool     carry() { return f & 0b00010000; }
  } registers;

  struct memoryBus {
    u8 memory[65536];
  } bus; 

  u16 pc; // program counter

public:
  enum e_InstructionType {
    NOP,
    LD_R16_IMM16,
    LD_atR16mem_a,
    LD_a_at16mem,
    LD_atIMM16_sp,
    INC_R16, DEC_R16,
    ADDHL_R16,
  };
  //enum e_OperandGroup {
  //  NONE = 0, R8, R16, R16STK, R16MEM, COND, B3, TGT3, IMM8, IMM16
  //};
  enum e_r8 {
    B = 0, C = 1, D = 2, E = 3, H = 4, L = 5, AT_HL = 6, A = 7,
  };
  enum e_r16 {
    BC = 0, DE = 1, HL = 2, SP = 3,
                            AF = 3,
                    HLI= 2, HLD= 3,
  };
  enum e_cond { nz = 0, z = 1, nc = 2, c = 3 };
  
  struct Instruction {
    e_InstructionType type;
    //e_OperandGroup ogroup;
    union { e_r8 r8_1, r8; }; e_r8 r8_2;
    e_r16 r16;
    e_cond cond;
  };

private:
  u8 getBits54(u8 byte) {
    return (byte & 0x30) >> 4;
  }

public:
  Instruction parseInstruction(u8 opcode) {
    u8 block = (opcode & 0b11000000) >> 6;
    switch (block) {
    case 0:
      if (opcode == 0) return { .type = NOP };
      switch (opcode & 0xF) {
        case 0b0001: return { .type = LD_R16_IMM16,
          .r16 = (e_r16)getBits54(opcode) }; break;
        case 0b0010: return { .type = LD_atR16mem_a,
          .r16 = (e_r16)getBits54(opcode) }; break;
        case 0b1010: return { .type = LD_a_at16mem,
          .r16 = (e_r16)getBits54(opcode) }; break;
        case 0b1000: return { .type = LD_atIMM16_sp,
          .r16 = (e_r16)getBits54(opcode) }; break;

        case 0b0011: return { .type = INC_R16,
          .r16 = (e_r16)getBits54(opcode) }; break; 
        case 0b1011: return { .type = DEC_R16,
          .r16 = (e_r16)getBits54(opcode) }; break; 
      }
      break;
    default: break;
    }
    return { .type = NOP };
  }

  u16 execute(Instruction instruction) {
    switch (instruction.type) {
    default: break;
    }

    return 0;
  }
  u8 add(u8 val) {
    u8 res = registers.a + val;
    registers.zero(res == 0);
    registers.subtract(false);
    registers.carry(((u16)registers.a + (u16)val) > 0xFF);
    registers.halfcarry(((registers.a & 0xF) + (val & 0xF)) > 0xF);
    return res;
  }
} cpu;

// GB following units:
//  - CPU 
//  - RAM 
//    - 8 KiB work RAM 
//    - 8 KiB video RAM
//  - ROM 
//  - I/O 
//    - screen 160 x 144
//    - sound: 4 channels stereo
//    - gamepad buttons

int main(void) {
  std::cout << "hi" << std::endl;
  
  CPU::Registers registers;
  registers.af = 256 + 2;

  std::cout << "a: " << (int)registers.a << std::endl;
  std::cout << "f: " << (int)registers.f << std::endl;

  return 0;
}
