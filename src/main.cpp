#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define getBits54(byte) (((byte) & 0x30) >> 4)
#define getBits543(byte) (((byte) & 0x38) >> 3)
#define getBits43(byte) (((byte) & 0x18) >> 3)
#define getBits210(byte) ((byte) & 0x7)

#define BOOT_OFF 0xFF50

class BUS {
private:
  u8 memory[65536];  // 0x0000 - 0xFFFF
  
  /* memory map
    0x0000 - 0x00FF   256 bytes     boot ROM
    
    0x0000 - 0x3FFF   16384 bytes   game ROM bank 0
    0x4000 - 0x7FFF   16384 bytes   game ROM bank N
    0x8000 - 0x97FF   6144 bytes    tile RAM
    0x9800 - 0x9FFF   2048 bytes    background map 
    0xA000 - 0xBFFF   8192 bytes    cartridge RAM 
    0xC000 - 0xDFFF   8192 bytes    working RAM
    0xE000 - 0xFDFF   7680 bytes    echo RAM
    0xFE00 - 0xFE9F   160 bytes     OAM (Object Attribute bus)
    0xFEA0 - 0xFEFF   96 bytes      unused
    0xFF00 - 0xFF7F   128 bytes     I/O registers
    0xFF80 - 0xFFFE   127 bytes     high RAM
    0xFFFF            1 byte        interrupt enabled register
  */

public:
  u8 operator [](u16 addr) const {
    return memory[addr];
  }

  u8 & operator [](u16 addr) {
    return memory[addr];
  }

public:
  void init() {

  }
};

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
  
    u16 pc; // program counter
    u16 sp; // stack pointer
    
    // TODO: implement interrupts, ime_next
    bool ime, ime_next; // interrupt enable flag
  } reg;

  BUS bus;

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
  struct Flags {
    bool zero, subtract, halfcarry, carry;
  };

  Flags wrappingAdd(u8 &a, u8 b) {
    u8 halfres = (a & 0xF) + (b & 0xF);
    u16 result = (u16)a + (u16)b;
    a = (u8)(result & 0xFF);
    return {(a==0),false,
      (halfres > 0xF), (result > 0xFF)};
  }
  
  Flags wrappingAdc(u8 &a, u8 b) {
    u8 halfres = (a & 0xF) + (b & 0xF) + (reg.carry() ? 1 : 0);
    u16 result = (u16)a + (u16)b + (reg.carry() ? 1 : 0);
    a = (u8)(result & 0xFF);
    return {(a==0),false,
      (halfres > 0xF), (result > 0xFF)};
  }
  
  Flags wrappingSub(u8 &a, u8 b) {
    s8 halfres = (s8)(a & 0xF) - (s8)(b & 0xF);
    s16 result = (s16)a - (s16)b;
    a = (result < 0) ? (u8)(result + 0x100) : (u8)(result);
    return {(a==0),true,
      (halfres < 0), (result < 0)};
  }
  
  Flags wrappingSbc(u8 &a, u8 b) {
    s8 halfres = (s8)(a & 0xF) - (s8)(b & 0xF) - (reg.carry() ? 1 : 0);
    s16 result = (s16)a - (s16)b - (reg.carry() ? 1 : 0);
    a = (result < 0) ? (u8)(result + 0x100) : (u8)(result);
    return {(a==0),true,
      (halfres < 0), (result < 0)};
  }
  
  Flags wrappingAdd(u16 &a, u16 b) {
    u16 halfres = (a & 0xFFF) + (b & 0xFFF);
    u32 result = (u32)a + (u32)b;
    a = (u16)(result & 0xFFFF);
    return {(a==0),false,
      (halfres > 0xFFF), (result > 0xFFFF)};
  }
  
  Flags wrappingSub(u16 &a, u16 b) {
    s16 halfres = (s16)(a & 0xFFF) - (s16)(b & 0xFFF);
    s32 result = (s32)a - (s32)b;
    a = (result < 0) ? (u16)(result + 0x10000) : (u16)(result);
    return {(a==0),true,
      (halfres < 0), (result < 0)};
  }

  void setFlags(Flags flags) {
    reg.zero(flags.zero);
    reg.subtract(flags.subtract);
    reg.halfcarry(flags.halfcarry);
    reg.carry(flags.halfcarry);
  }
  
  void setFlagsZNH(Flags flags) {
    reg.zero(flags.zero);
    reg.subtract(flags.subtract);
    reg.halfcarry(flags.halfcarry);
  }
  
  void setFlagsNHC(Flags flags) {
    reg.subtract(flags.subtract);
    reg.halfcarry(flags.halfcarry);
    reg.carry(flags.halfcarry);
  }

public:
  struct ExecResult {
    u16 cycles, bytes;
  };
  ExecResult execute(Instruction i) {
    bool prevCarry = reg.carry();
    u8 *r8, *r8_1, *r8_2;      
    switch(i.r8) {
      case 0: r8 = &reg.b; break;
      case 1: r8 = &reg.c; break;
      case 2: r8 = &reg.d; break;
      case 3: r8 = &reg.e; break;
      case 4: r8 = &reg.h; break;
      case 5: r8 = &reg.l; break;
      case 6: r8 = &bus[reg.hl]; break;
      case 7: r8 = &reg.a; break;
    } r8_1 = r8;
    switch (i.r8_2) {
      case 0: r8_2 = &reg.b; break;
      case 1: r8_2 = &reg.c; break;
      case 2: r8_2 = &reg.d; break;
      case 3: r8_2 = &reg.e; break;
      case 4: r8_2 = &reg.h; break;
      case 5: r8_2 = &reg.l; break;
      case 6: r8_2 = &bus[reg.hl]; break;
      case 7: r8_2 = &reg.a; break;
    }
    u16 *r16, *r16stk, *r16mem; int hlid = 0;
    switch (i.r16) {
      case 0: r16 = &reg.bc; r16stk = &reg.bc; r16mem = &reg.bc; break;
      case 1: r16 = &reg.de; r16stk = &reg.de; r16mem = &reg.de; break;
      case 2: r16 = &reg.hl; r16stk = &reg.hl; r16mem = &reg.hl; hlid = 1; break;
      case 3: r16 = &reg.sp; r16stk = &reg.af; r16mem = &reg.hl; hlid = -1;break;
    }
    bool cond;
    switch (i.cond) {
      case 0: cond = !reg.zero(); break;
      case 1: cond = reg.zero(); break;
      case 2: cond = !reg.carry(); break;
      case 3: cond = reg.carry(); break;
    }

    switch (i.type) {
      // block 0
      case NOP: return {1,1};
      case LD_R16_IMM16: (*r16) = i.imm16; return {3,3};
      case LD_atR16mem_A: bus[*r16mem] = reg.a;
        reg.hl += hlid; return {2,1};
      case LD_A_atR16mem: reg.a = bus[*r16mem];
        reg.hl += hlid; return {2,1};
      case LD_atIMM16_SP:
        bus[i.imm16] = reg.sp & 0xFF;
        bus[i.imm16+1] = reg.sp >> 8;
        return {5,3};
      case INC_R16: (*r16)++; return {2,1};
      case DEC_R16: (*r16)--; return {2,1};
      case ADDHL_R16:
        setFlagsNHC(wrappingAdd(reg.hl, (*r16)));
        return {2,1};
      case INC_R8:
        setFlagsZNH(wrappingAdd(*r8, 1));
        if (i.r8 == AT_HL) return {3,1};
        return {1,1};
      case DEC_R8:
        setFlagsZNH(wrappingSub(*r8, 1));
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
      case DAA: { // genuinely cancer
        u8 adj = 0;
        if ((!reg.subtract() && (reg.a & 0xF) > 0x09) ||
            reg.halfcarry()) adj |= 0x06;
        if ((!reg.subtract() && reg.a > 0x99) ||
            reg.carry()) adj |= 0x60;
        reg.zero(reg.a == 0);
        reg.halfcarry(false);
        if (!reg.subtract()) { reg.carry(
            wrappingAdd(reg.a, adj).carry); }
        else { reg.carry(
            wrappingSub(reg.a, adj).carry); }
        return {1,1}; }
      case CPL:
        reg.a = ~reg.a;
        reg.subtract(true);
        reg.halfcarry(true);
        return {1,1};
      case SCF:
        reg.subtract(false);
        reg.halfcarry(false);
        reg.carry(true);
        return {1,1};
      case CCF:
        reg.subtract(false);
        reg.halfcarry(false);
        reg.carry(!reg.carry());
        return {1,1};
      case JR_IMM8: // instruction 2 bytes, but jp so return 0
        reg.pc += (s8)i.imm8; return {3,0};
      case JR_COND_IMM8:
        if (cond) { reg.pc += (s8)i.imm8; return {3,0}; }
        return {2,2};
      case STOP: status = STOPPED; return {0,2};
      // block 1
      case LD_R8_R8: (*r8_1) = (*r8_2); return {1,1};
      case HALT: status = HALTED; return {4,1};
      // block 2
      case ADD_A_R8: setFlags(wrappingAdd(reg.a, *r8));
        return {1,1};
      case ADC_A_R8: setFlags(wrappingAdc(reg.a, *r8));
        return {1,1};
      case SUB_A_R8: setFlags(wrappingSub(reg.a, *r8));
        return {1,1};
      case SBC_A_R8: setFlags(wrappingSbc(reg.a, *r8));
        return {1,1};
      case AND_A_R8: reg.a &= *r8; reg.f = 0x20;
        reg.zero(reg.a == 0); return {1,1};
      case XOR_A_R8: reg.a ^= *r8; reg.f = 0;
        reg.zero(reg.a == 0); return {1,1};
      case OR_A_R8: reg.a |= *r8; reg.f = 0;
        reg.zero(reg.a == 0); return {1,1};
      case CP_A_R8: { u8 temp = reg.a;
        setFlags(wrappingSub(temp, *r8));
        return {1,1}; }
      // block 3
      case ADD_A_IMM8: setFlags(wrappingAdd(reg.a, i.imm8));
        return {1,1};
      case ADC_A_IMM8: setFlags(wrappingAdc(reg.a, i.imm8));
        return {1,1};
      case SUB_A_IMM8: setFlags(wrappingSub(reg.a, i.imm8));
        return {1,1};
      case SBC_A_IMM8: setFlags(wrappingSbc(reg.a, i.imm8));
        return {1,1};
      case AND_A_IMM8: reg.a &= i.imm8; reg.f = 0x20;
        reg.zero(reg.a == 0); return {1,1};
      case XOR_A_IMM8: reg.a ^= i.imm8; reg.f = 0;
        reg.zero(reg.a == 0); return {1,1};
      case OR_A_IMM8: reg.a |= i.imm8; reg.f = 0;
        reg.zero(reg.a == 0); return {1,1};
      case CP_A_IMM8: { u8 temp = reg.a;
        setFlags(wrappingSub(temp, i.imm8));
        return {1,1}; }
      case RET_COND:
        if (cond) {
          u8 lsb = bus[reg.sp++];
          u8 msb = bus[reg.sp++];
          reg.pc = ((u16)msb << 8) | (u16)lsb;
          return {5,0};
        } return {2,1};
      case RET: {
        u8 lsb = bus[reg.sp++];
        u8 msb = bus[reg.sp++];
        reg.pc = ((u16)msb << 8) | (u16)lsb;
        return {4,0}; }
      case RETI: {
        u8 lsb = bus[reg.sp++];
        u8 msb = bus[reg.sp++];
        reg.pc = ((u16)msb << 8) | (u16)lsb;
        reg.ime = true;
        return {4,0}; }
      case JP_COND_IMM16:
        if (cond) { reg.pc = i.imm16; return {4,0};}
        return {3,3};
      case JP_IMM16: reg.pc = i.imm16; return {4,0};
      case JP_HL: reg.pc = reg.hl; return {1,0};
      case CALL_COND_IMM16: if (cond) {
          bus[--reg.sp] = (u8)(reg.pc >> 8);
          bus[--reg.sp] = (u8)(reg.pc & 0xFF);
          reg.pc = i.imm16; return {6,0};
        } return {3,3};
      case CALL_IMM16:
        bus[--reg.sp] = (u8)(reg.pc >> 8);
        bus[--reg.sp] = (u8)(reg.pc & 0xFF);
        reg.pc = i.imm16; return {6,0};
      case RST_TGT3:
        bus[--reg.sp] = (u8)(reg.pc >> 8);
        bus[--reg.sp] = (u8)(reg.pc & 0xFF);
        reg.pc = i.tgt3; return {4,0};
      case POP_R16STK: {
        u8 lsb = bus[reg.sp++];
        u8 msb = bus[reg.sp++];
        (*r16stk) = ((u16)msb << 8) | (u16)lsb;
        return {3,1}; }
      case PUSH_R16STK:
        bus[--reg.sp] = (u8)((*r16stk) >> 8);
        bus[--reg.sp] = (u8)((*r16stk) & 0xFF);
        return {4,1};
      case LDH_atC_A:
        bus[(u16)reg.c | 0xFF00] = reg.a;
        return {2,1};
      case LDH_atIMM8_A:
        bus[(u16)i.imm8 | 0xFF00] = reg.a;
        return {3,2};
      case LD_atIMM16_A:
        bus[i.imm16] = reg.a;
        return {4,3};
      case LDH_A_atC:
        reg.a = bus[(u16)reg.c | 0xFF00];
        return {2,1};
      case LDH_A_atIMM8:
        reg.a = bus[(u16)i.imm8 | 0xFF00];
        return {3,2};
      case LD_A_atIMM16:
        reg.a = bus[i.imm16];
        return {4,3};
      case ADD_SP_IMM8: {
        s16 quarterres = (s16)(reg.sp & 0xF) + (s16)(s8)i.imm8;
        s32 halfres = (s32)(reg.sp & 0xFF) + (s32)(s8)i.imm8;
        reg.zero(false); reg.subtract(false);
        reg.halfcarry(quarterres < 0 || quarterres > 0xF);
        reg.carry(halfres < 0 || halfres > 0xFF);
        reg.sp = (u16)((s32)reg.sp + (s32)(s8)i.imm8);
        return {4,2}; }
      case LD_HL_SPplusIMM8: {
        s16 quarterres = (s16)(reg.sp & 0xF) + (s16)(s8)i.imm8;
        s32 halfres = (s32)(reg.sp & 0xFF) + (s32)(s8)i.imm8;
        reg.zero(false); reg.subtract(false);
        reg.halfcarry(quarterres < 0 || quarterres > 0xF);
        reg.carry(halfres < 0 || halfres > 0xFF);
        reg.hl = (u16)((s32)reg.sp + (s32)(s8)i.imm8);
        return {3,2}; }
      case LD_SP_HL: reg.sp = reg.hl; return {2,1};
      case DI: reg.ime = 0; return {1,1};
      case EI: reg.ime_next = 1; return {1,1};
      case RLC_R8:
        reg.f = 0; // clear rest of flags
        reg.carry(((*r8) & 0x80) == 0x80);
        (*r8) <<= 1; if (reg.carry()) (*r8) |= 1;
        reg.zero((*r8) == 0);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case RRC_R8:
        reg.f = 0; // clear rest of flags
        reg.carry(((*r8) & 1) == 1);
        (*r8) >>= 1; if (reg.carry()) (*r8) |= 0x80;
        reg.zero((*r8) == 0);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case RL_R8:
        reg.f = 0; // clear rest of flags
        reg.carry(((*r8) & 0x80) == 0x80);
        (*r8) <<= 1; if (prevCarry) (*r8) |= 1;
        reg.zero((*r8) == 0);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case RR_R8:
        reg.f = 0; // clear rest of flags
        reg.carry(((*r8) & 1) == 1);
        (*r8) >>= 1; if (prevCarry) (*r8) |= 0x80;
        reg.zero((*r8) == 0);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case SLA_R8:
        reg.f = 0; // clear rest of flags
        reg.carry(((*r8) & 0x80) == 0x80);
        (*r8) <<= 1;
        reg.zero((*r8) == 0);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case SRA_R8:
        reg.f = 0; // clear rest of flags
        reg.carry(((*r8) & 1) == 1);
        (*r8) >>= 1; if (i.r8 == AT_HL) (*r8) |= ((*r8) & 0x40) << 1;
        reg.zero((*r8) == 0);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case SWAP_R8:
        (*r8) = ((*r8) >> 4) | ((*r8) << 4);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case SRL_R8:
        reg.f = 0; // clear rest of flags
        reg.carry(((*r8) & 1) == 1);
        (*r8) >>= 1;
        reg.zero((*r8) == 0);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case BIT_B3_R8:
        reg.zero(((*r8) & (1 << i.b3)) == 0);
        reg.subtract(false);
        reg.halfcarry(true);
        if (i.r8 == AT_HL) return {3,2};
        return {2,2};
      case RES_B3_R8:
        (*r8) &= ~(1 << i.b3);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      case SET_B3_R8:
        (*r8) |= (1 << i.b3);
        if (i.r8 == AT_HL) return {4,2};
        return {2,2};
      default: break;
    }
    printf("unable to execute instruction\n");
    return {0,1};
  }

  void printState() {
    printf("HEX:\t\t\t\tDecimal:\n");
    printf("a: %#X\tf: %#X\taf: %#02X   \ta: %hhu\tf: %hhu\taf: %hu\n", reg.a, reg.f, reg.af, reg.a, reg.f, reg.af);
    printf("b: %#X\tc: %#X\tbc: %#02X   \tb: %hhu\tc: %hhu\tbc: %hu\n", reg.b, reg.c, reg.bc, reg.b, reg.c, reg.bc);
    printf("d: %#X\te: %#X\tde: %#02X   \td: %hhu\te: %hhu\tde: %hu\n", reg.d, reg.e, reg.de, reg.d, reg.e, reg.de);
    printf("h: %#X\tl: %#X\thl: %#02X   \th: %hhu\tl: %hhu\thl: %hu\n", reg.h, reg.l, reg.hl, reg.h, reg.l, reg.hl);
    printf("pc: %#02X\tsp: %#02X\t\t\tpc: %hu\tsp: %hu\n", reg.pc, reg.sp, reg.pc, reg.sp);
    printf("Flags: zero=%d subtract=%d halfcarry=%d carry = %d\n", reg.zero(), reg.subtract(), reg.halfcarry(), reg.carry());
  }

  void init() {
    bus.init();
    reg.pc = 0x0000;
  }

public:
  enum {
    BOOTING, RUNNING, HALTED, STOPPED
  } status = BOOTING;
};

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
  CPU cpu;
  cpu.init();

  cpu.printState();

  int cycles = 0;
  while (cpu.status == CPU::RUNNING || cpu.status == CPU::BOOTING) {
    u8 opcode = cpu.bus[cpu.reg.pc];
    CPU::Instruction instruction = cpu.parseInstruction(opcode);
    if (instruction.type == CPU::ILLEGAL) {
      std::cout << "illegal opcode encountered" << std::endl; break;
    }
    if (instruction.type == CPU::PREFIX) {
      opcode = cpu.bus[cpu.reg.pc+1];
      instruction = cpu.parsePrefixedInstruction(opcode);
    }
    instruction.imm8low  = cpu.bus[cpu.reg.pc+1];
    instruction.imm8high = cpu.bus[cpu.reg.pc+2];

    CPU::ExecResult execResult = cpu.execute(instruction);
    cycles += execResult.cycles;
    cpu.reg.pc += execResult.bytes;
  }

  cpu.printState();
  printf("mem[420] = %hhu\n", cpu.bus[420]);
  printf("mem[421] = %hhu\n", cpu.bus[421]);
  return 0;
}
