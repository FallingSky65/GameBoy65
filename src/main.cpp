#include <cstdint>
#include <cstdio>
#include <iostream>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define getBits54(byte) (((byte) & 0x30) >> 4)
#define getBits543(byte) (((byte) & 0x38) >> 3)
#define getBits43(byte) (((byte) & 0x18) >> 3)
#define getBits210(byte) ((byte) & 0x7)

class CPU {
public:
  struct reg {
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
  } reg;

  struct memoryBus {
    u8 memory[65536];
  } bus; 

  u16 pc; // program counter
  u16 sp; // stack pointer

public:
  //  https://gbdev.io/pandocs/CPU_Instruction_Set.html
  enum e_InstructionType {
    ILLEGAL,

    // block 0
    NOP,
    LD_R16_IMM16, LD_atR16mem_A, LD_A_atR16mem, LD_atIMM16_SP,
    INC_R16, DEC_R16, ADDHL_R16,
    INC_R8, DEC_R8,
    LD_R8_IMM8,
    RLCA, RRCA, RLA, RRA, DAA, CPL, SCF, CCF,
    JR_IMM8, JR_COND_IMM8,
    STOP,

    // block 1
    LD_R8_R8, HALT,

    // block 2 
    ADD_A_R8, ADC_A_R8, SUB_A_R8, SBC_A_R8,
    AND_A_R8, XOR_A_R8,  OR_A_R8,  CP_A_R8,
    
    // block 3 
    ADD_A_IMM8, ADC_A_IMM8, SUB_A_IMM8, SBC_A_IMM8,
    AND_A_IMM8, XOR_A_IMM8,  OR_A_IMM8,  CP_A_IMM8,
    RET_COND, RET, RETI,
    JP_COND_IMM16, JP_IMM16, JP_HL,
    CALL_COND_IMM16, CALL_IMM16, RST_TGT3,
    POP_R16STK, PUSH_R16STK,
    LDH_atC_A, LDH_atIMM8_A, LD_atIMM16_A,
    LDH_A_atC, LDH_A_atIMM8, LD_A_atIMM16,
    ADD_SP_IMM8, LD_HL_SPplusIMM8, LD_SP_HL,
    DI, EI,

    PREFIX,
    // prefixed instructions
    RLC_R8, RRC_R8, RL_R8, RR_R8,
    SLA_R8, SRA_R8, SWAP_R8, SRL_R8,
    BIT_B3_R8, RES_B3_R8, SET_B3_R8,
  };
  //enum e_OperandGroup {
  //  NONE = 0, R8, R16, R16STK, R16MEM, COND, B3, TGT3, IMM8, IMM16
  //};
  enum e_r8 : u8 {
    B = 0, C = 1, D = 2, E = 3, H = 4, L = 5, AT_HL = 6, A = 7,
  };
  enum e_r16 : u8 {
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
    union { u8 b3, tgt3; };
    union { struct { union { u8 imm8, imm8low; }; u8 imm8high; }; u16 imm16; };
  };

public:
  Instruction parseInstruction(u8 opcode) {
    u8 block = (opcode & 0b11000000) >> 6;
    if (block == 0) {
      if (opcode == 0x00) return { .type = NOP };
      if (opcode == 0x10) return { .type = STOP };
      if (!(opcode & 0b100)) switch (opcode & 0xF) {
        case 0b0001: return { .type = LD_R16_IMM16,
          .r16 = (e_r16)getBits54(opcode) };
        case 0b0010: return { .type = LD_atR16mem_A,
          .r16 = (e_r16)getBits54(opcode) };
        case 0b1010: return { .type = LD_A_atR16mem,
          .r16 = (e_r16)getBits54(opcode) };
        case 0b1000: return { .type = LD_atIMM16_SP,
          .r16 = (e_r16)getBits54(opcode) };

        case 0b0011: return { .type = INC_R16,
          .r16 = (e_r16)getBits54(opcode) };
        case 0b1011: return { .type = DEC_R16,
          .r16 = (e_r16)getBits54(opcode) };
        case 0b1001: return { .type = ADDHL_R16,
          .r16 = (e_r16)getBits54(opcode) };
        default: break;
      }
      if (opcode == 0x18) return { .type = JR_IMM8 };
      if ((opcode & 0xE7) == 0x20)
        return { .type = JR_COND_IMM8,
          .cond = (e_cond)getBits43(opcode) };
      if ((opcode & 0x7) == 0x7) switch(getBits543(opcode)) {
        case 0x0: return { .type = RLCA };
        case 0x1: return { .type = RRCA };
        case 0x2: return { .type = RLA };
        case 0x3: return { .type = RRA };
        case 0x4: return { .type = DAA };
        case 0x5: return { .type = CPL };
        case 0x6: return { .type = SCF };
        case 0x7: return { .type = CCF };
        default: break;
      }
      switch (opcode & 0x3) {
        case 0x0: return { .type = INC_R8,
          .r8 = (e_r8)getBits543(opcode) };
        case 0x1: return { .type = DEC_R8,
          .r8 = (e_r8)getBits543(opcode) };
        case 0x2: return { .type = LD_R8_IMM8,
          .r8 = (e_r8)getBits543(opcode) };
        default: break;
      }
    } else if (block == 1) {
      if (opcode == 0b01110110) return { .type = HALT, };
      return { .type = LD_R8_R8,
        .r8_1 = (e_r8)getBits543(opcode),
        .r8_2 = (e_r8)getBits210(opcode), };
    } else if (block == 2) {
      Instruction i = { .r8 = (e_r8)getBits210(opcode) };
      switch (getBits543(opcode)) {
        case 0x0: i.type = ADD_A_R8; return i;
        case 0x1: i.type = ADC_A_R8; return i;
        case 0x2: i.type = SUB_A_R8; return i;
        case 0x3: i.type = SBC_A_R8; return i;
        case 0x4: i.type = AND_A_R8; return i;
        case 0x5: i.type = XOR_A_R8; return i;
        case 0x6: i.type =  OR_A_R8; return i;
        case 0x7: i.type =  CP_A_R8; return i;
      }
    } else {
      if (opcode == 0b11001011) return { .type = PREFIX };
      if ((opcode & 0xE0) == 0xE0 && !(opcode & 0x4))
        switch (opcode) {
          case 0b11100010: return { .type = LDH_atC_A };
          case 0b11100000: return { .type = LDH_atIMM8_A };
          case 0b11101010: return { .type = LD_atIMM16_A };
          case 0b11110010: return { .type = LDH_A_atC };
          case 0b11110000: return { .type = LDH_A_atIMM8 };
          case 0b11111010: return { .type = LD_A_atIMM16 };
          
          case 0b11101000: return { .type = ADD_SP_IMM8 };
          case 0b11111000: return { .type = LD_HL_SPplusIMM8 };
          case 0b11111001: return { .type = LD_SP_HL };
          case 0b11110011: return { .type = DI };
          case 0b11111011: return { .type = EI };
        }
      if ((opcode & 0x7) == 0x6) switch (getBits543(opcode)) {
        case 0x0: return { .type = ADD_A_IMM8, };
        case 0x1: return { .type = ADC_A_IMM8, };
        case 0x2: return { .type = SUB_A_IMM8, };
        case 0x3: return { .type = SBC_A_IMM8, };
        case 0x4: return { .type = AND_A_IMM8, };
        case 0x5: return { .type = XOR_A_IMM8, };
        case 0x6: return { .type =  OR_A_IMM8, };
        case 0x7: return { .type =  CP_A_IMM8, };
      }
      if ((opcode & 0xF) == 0b0001) return {
        .type = POP_R16STK, .r16 = (e_r16)getBits54(opcode) };
      if ((opcode & 0xF) == 0b0101) return {
        .type = PUSH_R16STK, .r16 = (e_r16)getBits54(opcode) };
      if ((opcode & 0x7) == 0x7) return {
        .type = RST_TGT3, .tgt3 = (u8)getBits543(opcode) };
      switch (opcode) {
        case 0b11000000:
        case 0b11001000:
        case 0b11010000:
        case 0b11011000: return { .type = RET_COND,
          .cond = (e_cond)getBits43(opcode) };
        case 0b11001001: return { .type = RET };
        case 0b11011001: return { .type = RETI };
        case 0b11000010:
        case 0b11001010:
        case 0b11010010:
        case 0b11011010: return { .type = JP_COND_IMM16,
          .cond = (e_cond)getBits43(opcode) };
        case 0b11000011: return { .type = JP_IMM16 };
        case 0b11101001: return { .type = JP_HL };
        case 0b11000100:
        case 0b11001100:
        case 0b11010100:
        case 0b11011100: return { .type = CALL_COND_IMM16,
          .cond = (e_cond)getBits43(opcode) };
        case 0b11001101: return { .type = CALL_IMM16 };
        default: break;
      }
    }
    return { .type = ILLEGAL };
  }

  Instruction parsePrefixedInstruction(u8 opcode) {
    u8 block = (opcode & 0b11000000) >> 6;
    Instruction i = { .r8 = (e_r8)getBits210(opcode) };
    if (block == 0) switch (getBits543(opcode)) {
      case 0x0: i.type = RLC_R8; return i;
      case 0x1: i.type = RRC_R8; return i;
      case 0x2: i.type = RL_R8; return i;
      case 0x3: i.type = RR_R8; return i;
      case 0x4: i.type = SLA_R8; return i;
      case 0x5: i.type = SRA_R8; return i;
      case 0x6: i.type = SWAP_R8; return i;
      case 0x7: i.type = SRL_R8; return i;
    }
    i.b3 = getBits543(opcode);
    switch (block) {
      case 0x1: i.type = BIT_B3_R8; return i;
      case 0x2: i.type = RES_B3_R8; return i;
      case 0x3: i.type = SET_B3_R8; return i;
    }
    return { .type = ILLEGAL };
  }

private:
  u8 add(u8 val) {
    u8 res = reg.a + val;
    reg.zero(res == 0);
    reg.subtract(false);
    reg.carry(((u16)reg.a + (u16)val) > 0xFF);
    reg.halfcarry(((reg.a & 0xF) + (val & 0xF)) > 0xF);
    return res;
  }

public:
  struct ExecResult {
    u16 cycles, bytes;
  };
  ExecResult execute(Instruction i) {
    bool prevCarry = reg.carry();
    u8 *r8; u16 *r16, *r16stk, *r16mem; int hlid = 0;
    switch(i.r8) {
      case 0: r8 = &reg.b; break;
      case 1: r8 = &reg.c; break;
      case 2: r8 = &reg.d; break;
      case 3: r8 = &reg.e; break;
      case 4: r8 = &reg.h; break;
      case 5: r8 = &reg.l; break;
      case 6: r8 = &bus.memory[reg.hl]; break;
      case 7: r8 = &reg.a; break;
    }
    switch (i.r16) {
      case 0: r16 = &reg.bc; r16stk = &reg.bc; r16mem = &reg.bc; break;
      case 1: r16 = &reg.de; r16stk = &reg.de; r16mem = &reg.de; break;
      case 2: r16 = &reg.hl; r16stk = &reg.hl; r16mem = &reg.hl; hlid = 1; break;
      case 3: r16 = &sp;     r16stk = &reg.af; r16mem = &reg.hl; hlid = -1;break;
    }

    switch (i.type) {
      case NOP: return {1,1};
      case LD_R16_IMM16: (*r16) = i.imm16; return {3,3};
      case LD_atR16mem_A: bus.memory[*r16mem] = reg.a;
        reg.hl += hlid; return {2,1};
      case LD_A_atR16mem: reg.a = bus.memory[*r16mem];
        reg.hl += hlid; return {2,1};
      case LD_atIMM16_SP:
        bus.memory[i.imm16] = sp & 0xFF;
        bus.memory[i.imm16+1] = sp >> 8;
        return {5,3};
      case INC_R16: (*r16)++; return {2,1};
      case DEC_R16: (*r16)--; return {2,1};
      case ADDHL_R16:
        reg.subtract(false);
        reg.halfcarry(((reg.hl & 0xFFF) + ((*r16) & 0xFFF)) > 0xFFF);
        reg.carry(((u32)reg.hl + (u32)(*r16)) > 0xFFFF);
        reg.hl += (*r16);
        return {2,1};
      case INC_R8:
        reg.zero((u8)((*r8) + 1) == 0);
        reg.subtract(false);
        reg.halfcarry(((*r8) & 0xF) == 0xF);
        (*r8)++;
        if (i.r8 == AT_HL) return {3,1};
        return {1,1};
      case DEC_R8:
        reg.zero((u8)((*r8) - 1) == 0);
        reg.subtract(true);
        reg.halfcarry(((*r8) & 0xF) == 0x0);
        (*r8)--;
        if (i.r8 == AT_HL) return {3,1};
        return {1,1};
      case LD_R8_IMM8: (*r8) = i.imm8; return {2,2};
      case RLCA:
        reg.f = 0; // clear rest of flags
        reg.carry((reg.a & 0x80) == 0x80);
        reg.a <<= 1; if (reg.carry()) reg.a |= 1;
        return {1,1};
      case RRCA:
        reg.f = 0; // clear rest of flags
        reg.carry((reg.a & 1) == 1);
        reg.a >>= 1; if (reg.carry()) reg.a |= 0x80;
        return {1,1};
      case RLA: 
        reg.f = 0; // clear rest of flags
        reg.carry((reg.a & 0x80) == 0x80);
        reg.a <<= 1; if (prevCarry) reg.a |= 1;
        return {1,1};
      case RRA: 
        reg.f = 0; // clear rest of flags
        reg.carry((reg.a & 1) == 1);
        reg.a >>= 1; if (prevCarry) reg.a |= 0x80;
        return {1,1};
      case STOP: status = STOPPED; return {0,2};
      default: return {0,1};
    }
  }

  void printState() {
    printf("HEX:\t\t\t\tDecimal:\n");
    printf("a: %#X\tf: %#X\taf: %#02X   \ta: %hhu\tf: %hhu\taf: %hu\n", reg.a, reg.f, reg.af, reg.a, reg.f, reg.af);
    printf("b: %#X\tc: %#X\tbc: %#02X   \tb: %hhu\tc: %hhu\tbc: %hu\n", reg.b, reg.c, reg.bc, reg.b, reg.c, reg.bc);
    printf("d: %#X\te: %#X\tde: %#02X   \td: %hhu\te: %hhu\tde: %hu\n", reg.d, reg.e, reg.de, reg.d, reg.e, reg.de);
    printf("h: %#X\tl: %#X\thl: %#02X   \th: %hhu\tl: %hhu\thl: %hu\n", reg.h, reg.l, reg.hl, reg.h, reg.l, reg.hl);
    printf("pc: %#02X\tsp: %#02X\t\t\tpc: %hu\tsp: %hu\n", pc, sp, pc, sp);
    printf("Flags: zero=%d subtract=%d halfcarry=%d carry = %d\n", reg.zero(), reg.subtract(), reg.halfcarry(), reg.carry());
  }

  CPU() {

  }

public:
  enum {
    RUNNING, HALTED, STOPPED
  } status;
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
  cpu.pc = 0;
  cpu.bus.memory[0] = 0b00100001; // ld hl, imm16
  cpu.bus.memory[1] = 0xA4; // imm16 = 420
  cpu.bus.memory[2] = 0x01;
  cpu.bus.memory[3] = 0b00101010; // ld a, [hli]
  cpu.bus.memory[4] = 0b00110010; // ld [hld], a
  cpu.bus.memory[6] = 0b00110001; // ld sp, imm16
  cpu.bus.memory[7] = 0xA4; // imm16 = 420
  cpu.bus.memory[8] = 0x01;
  cpu.bus.memory[9] = 0b00001000; // ld [imm16], sp
  cpu.bus.memory[10] = 0xA4; // imm16 = 420
  cpu.bus.memory[11] = 0x01;
  cpu.bus.memory[12] = 0b00010000; // stop

  cpu.bus.memory[420] = 69;

  cpu.printState();

  int cycles = 0;
  while (cpu.status == CPU::RUNNING) {
    u8 opcode = cpu.bus.memory[cpu.pc];
    CPU::Instruction instruction = cpu.parseInstruction(opcode);
    if (instruction.type == CPU::ILLEGAL) {
      std::cout << "illegal opcode encountered" << std::endl; break;
    }
    if (instruction.type == CPU::PREFIX) {
      opcode = cpu.bus.memory[cpu.pc+1];
      instruction = cpu.parsePrefixedInstruction(opcode);
    }
    instruction.imm8low  = cpu.bus.memory[cpu.pc+1];
    instruction.imm8high = cpu.bus.memory[cpu.pc+2];

    CPU::ExecResult execResult = cpu.execute(instruction);
    cycles += execResult.cycles;
    cpu.pc += execResult.bytes;
  }

  cpu.printState();
  printf("mem[420] = %hhu\n", cpu.bus.memory[420]);
  printf("mem[421] = %hhu\n", cpu.bus.memory[421]);
  return 0;
}
