#include "instr.h"
#include "instr_encoding.h"
#include "trap.h"
#include "util.h"

#include <cassert>
#include <stdexcept>

#define MATCH_AND_RETURN_INSTR2(instr, result)                                 \
  if (UNLIKELY((data() & (instr##_MASK)) != (instr##_ENCODING)))               \
    return UNDEF;                                                              \
  return result;

#define MATCH_AND_RETURN_INSTR(instr) MATCH_AND_RETURN_INSTR2(instr, instr)

std::array<const char *, Compressed::NUMBER_OF_INSTRUCTIONS>
    Compressed::mappingStr = {
        // quadrant zero
        "C.Illegal",
        "C.Reserved",
        "C.ADDI4SPN",
        "C.FLD",
        "C.LQ", // RV128
        "C.LW",
        "C.FLW",
        "C.LD",

        "C.FSD",
        "C.SQ", // RV128
        "C.SW",
        "C.FSW",
        "C.SD",

        // quadrant one
        "C.NOP",
        "C.ADDI",
        "C.JAL",
        "C.ADDIW",
        "C.LI",
        "C.ADDI16SP",
        "C.LUI",
        "C.SRLI",
        "C.SRAI",
        "C.ANDI",
        "C.SUB",
        "C.XOR",
        "C.OR",
        "C.AND",
        "C.SUBW",
        "C.ADDW",
        "C.J",
        "C.BEQZ",
        "C.BNEZ",

        // quadrant two
        "C.SLLI",
        "C.FLDSP",
        "C.LQSP", // RV128
        "C.LWSP",
        "C.FLWSP",
        "C.LDSP",
        "C.JR",
        "C.MV",
        "C.EBREAK",
        "C.JALR",
        "C.ADD",
        "C.FSDSP",
        "C.SQSP", // RV128
        "C.SWSP",
        "C.FSWSP",
        "C.SDSP",
};

/*
Python snippet to generate the "mappingStr":

for e in [e.strip().replace(",", "") for e in s.strip().split("\n")]:
    if "//" in e or len(e) == 0:
        print(e)
    else:
        print('"{}",'.format(e))
 */
std::array<const char *, Opcode::NUMBER_OF_INSTRUCTIONS> Opcode::mappingStr = {
    "ZERO-INVALID",

    // RV32I base instruction set
    "LUI",
    "AUIPC",
    "JAL",
    "JALR",
    "BEQ",
    "BNE",
    "BLT",
    "BGE",
    "BLTU",
    "BGEU",
    "LB",
    "LH",
    "LW",
    "LBU",
    "LHU",
    "SB",
    "SH",
    "SW",
    "ADDI",
    "SLTI",
    "SLTIU",
    "XORI",
    "ORI",
    "ANDI",
    "SLLI",
    "SRLI",
    "SRAI",
    "ADD",
    "SUB",
    "SLL",
    "SLT",
    "SLTU",
    "XOR",
    "SRL",
    "SRA",
    "OR",
    "AND",
    "FENCE",
    "ECALL",
    "EBREAK",

    // Zifencei standard extension
    "FENCE_I",

    // Zicsr standard extension
    "CSRRW",
    "CSRRS",
    "CSRRC",
    "CSRRWI",
    "CSRRSI",
    "CSRRCI",

    // RV32M Standard Extension
    "MUL",
    "MULH",
    "MULHSU",
    "MULHU",
    "DIV",
    "DIVU",
    "REM",
    "REMU",

    // RV32A Standard Extension
    "LR_W",
    "SC_W",
    "AMOSWAP_W",
    "AMOADD_W",
    "AMOXOR_W",
    "AMOAND_W",
    "AMOOR_W",
    "AMOMIN_W",
    "AMOMAX_W",
    "AMOMINU_W",
    "AMOMAXU_W",

    // RV64I base integer set (addition to RV32I)
    "LWU",
    "LD",
    "SD",
    "ADDIW",
    "SLLIW",
    "SRLIW",
    "SRAIW",
    "ADDW",
    "SUBW",
    "SLLW",
    "SRLW",
    "SRAW",

    // RV64M standard extension (addition to RV32M)
    "MULW",
    "DIVW",
    "DIVUW",
    "REMW",
    "REMUW",

    // RV64A standard extension (addition to RV32A)
    "LR_D",
    "SC_D",
    "AMOSWAP_D",
    "AMOADD_D",
    "AMOXOR_D",
    "AMOAND_D",
    "AMOOR_D",
    "AMOMIN_D",
    "AMOMAX_D",
    "AMOMINU_D",
    "AMOMAXU_D",

    // RV32F standard extension
    "FLW",
    "FSW",
    "FMADD_S",
    "FMSUB_S",
    "FNMADD_S",
    "FNMSUB_S",
    "FADD_S",
    "FSUB_S",
    "FMUL_S",
    "FDIV_S",
    "FSQRT_S",
    "FSGNJ_S",
    "FSGNJN_S",
    "FSGNJX_S",
    "FMIN_S",
    "FMAX_S",
    "FCVT_W_S",
    "FCVT_WU_S",
    "FMV_X_W",
    "FEQ_S",
    "FLT_S",
    "FLE_S",
    "FCLASS_S",
    "FCVT_S_W",
    "FCVT_S_WU",
    "FMV_W_X",

    // RV64F standard extension (addition to RV32F)
    "FCVT_L_S",
    "FCVT_LU_S",
    "FCVT_S_L",
    "FCVT_S_LU",

    // RV32D standard extension
    "FLD",
    "FSD",
    "FMADD_D",
    "FMSUB_D",
    "FNMSUB_D",
    "FNMADD_D",
    "FADD_D",
    "FSUB_D",
    "FMUL_D",
    "FDIV_D",
    "FSQRT_D",
    "FSGNJ_D",
    "FSGNJN_D",
    "FSGNJX_D",
    "FMIN_D",
    "FMAX_D",
    "FCVT_S_D",
    "FCVT_D_S",
    "FEQ_D",
    "FLT_D",
    "FLE_D",
    "FCLASS_D",
    "FCVT_W_D",
    "FCVT_WU_D",
    "FCVT_D_W",
    "FCVT_D_WU",

    // RV64D standard extension (addition to RV32D)
    "FCVT_L_D",
    "FCVT_LU_D",
    "FMV_X_D",
    "FCVT_D_L",
    "FCVT_D_LU",
    "FMV_D_X",

    // privileged instructions
    "URET",
    "SRET",
    "MRET",
    "WFI",
    "SFENCE_VMA",
};

Opcode::Type Opcode::getType(Opcode::Mapping mapping) {
  switch (mapping) {
  case SLLI:
  case SRLI:
  case SRAI:
  case ADD:
  case SUB:
  case SLL:
  case SLT:
  case SLTU:
  case XOR:
  case SRL:
  case SRA:
  case OR:
  case AND:
  case MUL:
  case MULH:
  case MULHSU:
  case MULHU:
  case DIV:
  case DIVU:
  case REM:
  case REMU:
  case ADDW:
  case SUBW:
  case SLLW:
  case SRLW:
  case SRAW:
  case MULW:
  case DIVW:
  case DIVUW:
  case REMW:
  case REMUW:
  case LR_W:
  case SC_W:
  case AMOSWAP_W:
  case AMOADD_W:
  case AMOXOR_W:
  case AMOAND_W:
  case AMOOR_W:
  case AMOMIN_W:
  case AMOMAX_W:
  case AMOMINU_W:
  case AMOMAXU_W:
  case LR_D:
  case SC_D:
  case AMOSWAP_D:
  case AMOADD_D:
  case AMOXOR_D:
  case AMOAND_D:
  case AMOOR_D:
  case AMOMIN_D:
  case AMOMAX_D:
  case AMOMINU_D:
  case AMOMAXU_D:
  case FADD_S:
  case FSUB_S:
  case FMUL_S:
  case FDIV_S:
  case FSQRT_S:
  case FSGNJ_S:
  case FSGNJN_S:
  case FSGNJX_S:
  case FMIN_S:
  case FMAX_S:
  case FCVT_W_S:
  case FCVT_WU_S:
  case FMV_X_W:
  case FEQ_S:
  case FLT_S:
  case FLE_S:
  case FCLASS_S:
  case FCVT_S_W:
  case FCVT_S_WU:
  case FMV_W_X:
  case FCVT_L_S:
  case FCVT_LU_S:
  case FCVT_S_L:
  case FCVT_S_LU:
  case FADD_D:
  case FSUB_D:
  case FMUL_D:
  case FDIV_D:
  case FSQRT_D:
  case FSGNJ_D:
  case FSGNJN_D:
  case FSGNJX_D:
  case FMIN_D:
  case FMAX_D:
  case FCVT_S_D:
  case FCVT_D_S:
  case FEQ_D:
  case FLT_D:
  case FLE_D:
  case FCLASS_D:
  case FCVT_W_D:
  case FCVT_WU_D:
  case FCVT_D_W:
  case FCVT_D_WU:
  case FCVT_L_D:
  case FCVT_LU_D:
  case FMV_X_D:
  case FCVT_D_L:
  case FCVT_D_LU:
  case FMV_D_X:
    return Type::R;
  case JALR:
  case LB:
  case LH:
  case LW:
  case LD:
  case LBU:
  case LHU:
  case LWU:
  case ADDI:
  case SLTI:
  case SLTIU:
  case XORI:
  case ORI:
  case ANDI:
  case ADDIW:
  case SLLIW:
  case SRLIW:
  case SRAIW:
  case FLW:
  case FLD:
    return Type::I;
  case SB:
  case SH:
  case SW:
  case SD:
  case FSW:
  case FSD:
    return Type::S;
  case BEQ:
  case BNE:
  case BLT:
  case BGE:
  case BLTU:
  case BGEU:
    return Type::B;
  case LUI:
  case AUIPC:
    return Type::U;
  case JAL:
    return Type::J;
  case FMADD_S:
  case FMSUB_S:
  case FNMSUB_S:
  case FNMADD_S:
  case FMADD_D:
  case FMSUB_D:
  case FNMSUB_D:
  case FNMADD_D:
    return Type::R4;

  default:
    return Type::UNKNOWN;
  }
}

struct InstructionFactory {
  typedef Instruction T;

  static T ADD(unsigned rd, unsigned rs1, unsigned rs2) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) |
             51 | (0 << 12) | (0 << 25));
  }

  static T AND(unsigned rd, unsigned rs1, unsigned rs2) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) |
             51 | (7 << 12) | (0 << 25));
  }

  static T OR(unsigned rd, unsigned rs1, unsigned rs2) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) |
             51 | (6 << 12) | (0 << 25));
  }

  static T XOR(unsigned rd, unsigned rs1, unsigned rs2) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) |
             51 | (4 << 12) | (0 << 25));
  }

  static T SUB(unsigned rd, unsigned rs1, unsigned rs2) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) |
             51 | (0 << 12) | (32 << 25));
  }

  static T LW(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 3 | (2 << 12));
  }

  static T LD(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 3 | (3 << 12));
  }

  static T FLW(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 7 | (2 << 12));
  }

  static T FLD(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 7 | (3 << 12));
  }

  static T SW(unsigned rs1, unsigned rs2, int S_imm) {
    return T((((S_imm & 0b11111) << 7) | ((S_imm & (0b1111111 << 5)) << 20)) |
             ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) | 35 | (2 << 12));
  }

  static T SD(unsigned rs1, unsigned rs2, int S_imm) {
    return T((((S_imm & 0b11111) << 7) | ((S_imm & (0b1111111 << 5)) << 20)) |
             ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) | 35 | (3 << 12));
  }

  static T FSW(unsigned rs1, unsigned rs2, int S_imm) {
    return T((((S_imm & 0b11111) << 7) | ((S_imm & (0b1111111 << 5)) << 20)) |
             ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) | 39 | (2 << 12));
  }

  static T FSD(unsigned rs1, unsigned rs2, int S_imm) {
    return T((((S_imm & 0b11111) << 7) | ((S_imm & (0b1111111 << 5)) << 20)) |
             ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) | 39 | (3 << 12));
  }

  static T LUI(unsigned rd, int U_imm) {
    return T((U_imm & (1048575 << 12)) | ((rd & 0x1f) << 7) | 55);
  }

  static T ADDI(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 19 | (0 << 12));
  }

  static T ANDI(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 19 | (7 << 12));
  }

  static T SRLI(unsigned rd, unsigned rs1, unsigned shamt) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((shamt & 63) << 20) |
             19 | (5 << 12) | (0 << 25));
  }

  static T SRAI(unsigned rd, unsigned rs1, unsigned shamt) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((shamt & 63) << 20) |
             19 | (5 << 12) | (32 << 25));
  }

  static T SLLI(unsigned rd, unsigned rs1, unsigned shamt) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((shamt & 63) << 20) |
             19 | (1 << 12) | (0 << 25));
  }

  static T JAL(unsigned rd, int J_imm) {
    return T(111 | ((rd & 0x1f) << 7) |
             ((J_imm & (0b11111111 << 12)) | ((J_imm & (1 << 11)) << 9) |
              ((J_imm & 0b11111111110) << 20) | ((J_imm & (1 << 20)) << 11)));
  }

  static T JALR(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 103 | (0 << 12));
  }

  static T BEQ(unsigned rs1, unsigned rs2, int B_imm) {
    return T(
        ((((B_imm & 0b11110) << 7) | ((B_imm & (1 << 11)) >> 4)) |
         (((B_imm & (0b111111 << 5)) << 20) | ((B_imm & (1 << 12)) << 19))) |
        ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) | 99 | (0 << 12));
  }

  static T BNE(unsigned rs1, unsigned rs2, int B_imm) {
    return T(
        ((((B_imm & 0b11110) << 7) | ((B_imm & (1 << 11)) >> 4)) |
         (((B_imm & (0b111111 << 5)) << 20) | ((B_imm & (1 << 12)) << 19))) |
        ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) | 99 | (1 << 12));
  }

  static T EBREAK() { return T(1048691); }

  static T ADDIW(unsigned rd, unsigned rs1, int I_imm) {
    return T(((I_imm & 4095) << 20) | ((rd & 0x1f) << 7) |
             ((rs1 & 0x1f) << 15) | 0x1b);
  }

  static T ADDW(unsigned rd, unsigned rs1, unsigned rs2) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) |
             0x3b);
  }

  static T SUBW(unsigned rd, unsigned rs1, unsigned rs2) {
    return T(((rd & 0x1f) << 7) | ((rs1 & 0x1f) << 15) | ((rs2 & 0x1f) << 20) |
             0x3b | 0x40000000);
  }
};

Compressed::Opcode decode_compressed(Instruction &instr, Architecture arch) {
  using namespace Compressed;

  switch (instr.quadrant()) {
  case 0:
    switch (instr.c_opcode()) {
    case 0b000:
      if (instr.c_format() == 0)
        return C_Illegal;
      else
        return C_ADDI4SPN;

    case 0b001:
      return C_FLD;

    case 0b010:
      return C_LW;

    case 0b011:
      if (arch == RV32)
        return C_FLW;
      else
        return C_LD;

    case 0b100:
      return C_Reserved;

    case 0b101:
      return C_FSD;

    case 0b110:
      return C_SW;

    case 0b111:
      if (arch == RV32)
        return C_FSW;
      else
        return C_SD;
    }
    break;

  case 1:
    switch (instr.c_opcode()) {
    case 0b000:
      if (instr.c_format() == 1)
        return C_NOP;
      else
        return C_ADDI;

    case 0b001:
      if (arch == RV32)
        return C_JAL;
      else
        return C_ADDIW;

    case 0b010:
      return C_LI;

    case 0b011:
      if (instr.c_rd() == 2)
        return C_ADDI16SP;
      else
        return C_LUI;

    case 0b100:
      switch (instr.c_f2_high()) {
      case 0b00:
        return C_SRLI;

      case 0b01:
        return C_SRAI;

      case 0b10:
        return C_ANDI;

      case 0b11:
        if (instr.c_b12()) {
          switch (instr.c_f2_low()) {
          case 0b00:
            return C_SUBW;
          case 0b01:
            return C_ADDW;
          }
          return C_Reserved;
        } else {
          switch (instr.c_f2_low()) {
          case 0b00:
            return C_SUB;
          case 0b01:
            return C_XOR;
          case 0b10:
            return C_OR;
          case 0b11:
            return C_AND;
          }
        }
      }
      break;

    case 0b101:
      return C_J;

    case 0b110:
      return C_BEQZ;

    case 0b111:
      return C_BNEZ;
    }
    break;

  case 2:
    switch (instr.c_opcode()) {
    case 0b000:
      return C_SLLI;

    case 0b001:
      return C_FLDSP;

    case 0b010:
      return C_LWSP;

    case 0b011:
      if (arch == RV32)
        return C_FLWSP;
      else
        return C_LDSP;

    case 0b100:
      if (instr.c_b12()) {
        if (instr.c_rs2()) {
          return C_ADD;
        } else {
          if (instr.c_rd()) {
            return C_JALR;
          } else {
            return C_EBREAK;
          }
        }
      } else {
        if (instr.c_rs2()) {
          return C_MV;
        } else {
          return C_JR;
        }
      }

    case 0b101:
      return C_FSDSP;

    case 0b110:
      return C_SWSP;

    case 0b111:
      if (arch == RV32)
        return C_FSWSP;
      else
        return C_SDSP;
    }
    break;

  case 3:
    throw std::runtime_error(
        "compressed instruction expected, but uncompressed found");
  }

  // undefined/unsupported instruction
  return C_Illegal;
}

Opcode::Mapping expand_compressed(Instruction &instr, Compressed::Opcode op,
                                  Architecture arch) {
  using namespace Opcode;
  using namespace Compressed;

  switch (op) {
  case C_Illegal:
    return UNDEF;

  // RV128 currently not supported
  case C_LQ:
  case C_LQSP:
  case C_SQ:
  case C_SQSP:
    return UNDEF;

  case C_Reserved:
    return UNDEF; // reserved instructions should raise an illegal instruction
                  // exception

  case C_NOP:
    instr = InstructionFactory::ADDI(0, 0, 0);
    return ADDI;

  case C_ADD:
    instr = InstructionFactory::ADD(instr.c_rd(), instr.c_rd(), instr.c_rs2());
    return ADD;

  case C_MV:
    instr = InstructionFactory::ADD(instr.c_rd(), 0, instr.c_rs2());
    return ADD;

  case C_AND:
    instr = InstructionFactory::AND(instr.c_rd_small(), instr.c_rd_small(),
                                    instr.c_rs2_small());
    return AND;

  case C_OR:
    instr = InstructionFactory::OR(instr.c_rd_small(), instr.c_rd_small(),
                                   instr.c_rs2_small());
    return OR;

  case C_XOR:
    instr = InstructionFactory::XOR(instr.c_rd_small(), instr.c_rd_small(),
                                    instr.c_rs2_small());
    return XOR;

  case C_SUB:
    instr = InstructionFactory::SUB(instr.c_rd_small(), instr.c_rd_small(),
                                    instr.c_rs2_small());
    return SUB;

  case C_ADDW:
    instr = InstructionFactory::ADDW(instr.c_rd_small(), instr.c_rd_small(),
                                     instr.c_rs2_small());
    return ADDW;

  case C_SUBW:
    instr = InstructionFactory::SUBW(instr.c_rd_small(), instr.c_rd_small(),
                                     instr.c_rs2_small());
    return SUBW;

  case C_LW:
    instr = InstructionFactory::LW(instr.c_rs2_small(), instr.c_rd_small(),
                                   C_LW_UIMM(instr.data()));
    return LW;

  case C_LD:
    instr = InstructionFactory::LD(instr.c_rs2_small(), instr.c_rd_small(),
                                   C_LD_UIMM(instr.data()));
    return LD;

  case C_FLW:
    instr = InstructionFactory::FLW(instr.c_rs2_small(), instr.c_rd_small(),
                                    C_LW_UIMM(instr.data()));
    return FLW;

  case C_FLD:
    instr = InstructionFactory::FLD(instr.c_rs2_small(), instr.c_rd_small(),
                                    C_LD_UIMM(instr.data()));
    return FLD;

  case C_SW:
    instr = InstructionFactory::SW(instr.c_rd_small(), instr.c_rs2_small(),
                                   C_SW_UIMM(instr.data()));
    return SW;

  case C_SD:
    instr = InstructionFactory::SD(instr.c_rd_small(), instr.c_rs2_small(),
                                   C_SD_UIMM(instr.data()));
    return SD;

  case C_FSW:
    instr = InstructionFactory::FSW(instr.c_rd_small(), instr.c_rs2_small(),
                                    C_SW_UIMM(instr.data()));
    return FSW;

  case C_FSD:
    instr = InstructionFactory::FSD(instr.c_rd_small(), instr.c_rs2_small(),
                                    C_SD_UIMM(instr.data()));
    return FSD;

  case C_ADDI4SPN: {
    unsigned n = C_ADDI4SPN_NZUIMM(instr.data());
    if (n == 0)
      return UNDEF;
    instr = InstructionFactory::ADDI(instr.c_rs2_small(), 2, n);
    return ADDI;
  }

  case C_ADDI:
    instr = InstructionFactory::ADDI(instr.c_rd(), instr.c_rd(), instr.c_imm());
    return ADDI;

  case C_JAL:
    instr = InstructionFactory::JAL(1, C_JAL_IMM(instr.data()));
    return JAL;

  case C_ADDIW:
    if (instr.c_rd() == 0)
      return UNDEF; // reserved
    instr = InstructionFactory::ADDI(instr.c_rd(), instr.c_rd(), instr.c_imm());
    return ADDIW;

  case C_LI:
    instr = InstructionFactory::ADDI(instr.c_rd(), 0, instr.c_imm());
    return ADDI;

  case C_ADDI16SP: {
    auto n = C_ADDI16SP_NZIMM(instr.data());
    if (n == 0)
      return UNDEF; // reserved
    instr = InstructionFactory::ADDI(2, 2, n);
    return ADDI;
  }

  case C_LUI: {
    auto n = C_LUI_NZIMM(instr.data());
    if (n == 0)
      return UNDEF; // reserved
    instr = InstructionFactory::LUI(instr.c_rd(), n);
    return LUI;
  }

  case C_SLLI: {
    auto n = instr.c_uimm();
    if (arch == RV32 && n > 31)
      return UNDEF;
    instr = InstructionFactory::SLLI(instr.c_rd(), instr.c_rd(),
                                     arch == RV32 ? n & 31 : n);
    return SLLI;
  }

  case C_SRLI: {
    auto n = instr.c_uimm();
    if (arch == RV32 && n > 31)
      return UNDEF;
    instr = InstructionFactory::SRLI(instr.c_rd_small(), instr.c_rd_small(),
                                     arch == RV32 ? n & 31 : n);
    return SRLI;
  }

  case C_SRAI: {
    auto n = instr.c_uimm();
    if (arch == RV32 && n > 31)
      return UNDEF;
    instr = InstructionFactory::SRAI(instr.c_rd_small(), instr.c_rd_small(),
                                     arch == RV32 ? n & 31 : n);
    return SRAI;
  }

  case C_ANDI:
    instr = InstructionFactory::ANDI(instr.c_rd_small(), instr.c_rd_small(),
                                     instr.c_imm());
    return ANDI;

  case C_J:
    instr = InstructionFactory::JAL(0, C_J_IMM(instr.data()));
    return JAL;

  case C_BEQZ:
    instr = InstructionFactory::BEQ(instr.c_rd_small(), 0,
                                    C_BRANCH_IMM(instr.data()));
    return BEQ;

  case C_BNEZ:
    instr = InstructionFactory::BNE(instr.c_rd_small(), 0,
                                    C_BRANCH_IMM(instr.data()));
    return BNE;

  case C_LWSP:
    if (instr.c_rd() == 0)
      return UNDEF; // reserved
    instr = InstructionFactory::LW(instr.c_rd(), 2, C_LWSP_UIMM(instr.data()));
    return LW;

  case C_LDSP:
    if (instr.c_rd() == 0)
      return UNDEF; // reserved
    instr = InstructionFactory::LD(instr.c_rd(), 2, C_LDSP_UIMM(instr.data()));
    return LD;

  case C_FLWSP:
    instr = InstructionFactory::FLW(instr.c_rd(), 2, C_LWSP_UIMM(instr.data()));
    return FLW;

  case C_FLDSP:
    instr = InstructionFactory::FLD(instr.c_rd(), 2, C_LDSP_UIMM(instr.data()));
    return FLD;

  case C_SWSP:
    instr = InstructionFactory::SW(2, instr.c_rs2(), C_SWSP_UIMM(instr.data()));
    return SW;

  case C_SDSP:
    instr = InstructionFactory::SD(2, instr.c_rs2(), C_SDSP_UIMM(instr.data()));
    return SD;

  case C_FSWSP:
    instr =
        InstructionFactory::FSW(2, instr.c_rs2(), C_SWSP_UIMM(instr.data()));
    return FSW;

  case C_FSDSP:
    instr =
        InstructionFactory::FSD(2, instr.c_rs2(), C_SDSP_UIMM(instr.data()));
    return FSD;

  case C_EBREAK:
    instr = InstructionFactory::EBREAK();
    return EBREAK;

  case C_JR:
    if (instr.c_rd() == 0)
      return UNDEF; // reserved
    instr = InstructionFactory::JALR(0, instr.c_rd(), 0);
    return JALR;

  case C_JALR:
    instr = InstructionFactory::JALR(1, instr.c_rd(), 0);
    return JALR;
  }

  throw std::runtime_error("some compressed instruction not handled");
}

bool is_supported(Compressed::Opcode op) {
  using namespace Compressed;

  switch (op) {
  case C_NOP:
  case C_ADD:
  case C_MV:
  case C_AND:
  case C_OR:
  case C_XOR:
  case C_SUB:
  case C_LW:
  case C_SW:
  case C_ADDI4SPN:
  case C_ADDI:
  case C_JAL:
  case C_LI:
  case C_ADDI16SP:
  case C_LUI:
  case C_SLLI:
  case C_SRLI:
  case C_SRAI:
  case C_ANDI:
  case C_J:
  case C_BEQZ:
  case C_BNEZ:
  case C_JALR:
  case C_JR:
  case C_EBREAK:
  case C_SWSP:
  case C_LWSP:
    return true;
  }

  return false;
}

Opcode::Mapping Instruction::decode_and_expand_compressed(Architecture arch) {
  auto c_op = decode_compressed(arch);
  if (is_supported(c_op))
    return expand_compressed(*this, c_op, arch);
  else
    return Opcode::UNDEF;
}

Compressed::Opcode Instruction::decode_compressed(Architecture arch) {
  return ::decode_compressed(*this, arch);
}

Opcode::Mapping Instruction::decode_normal(Architecture arch) {
  using namespace Opcode;

  Instruction &instr = *this;

  switch (instr.opcode()) {
  case OP_LUI:
    MATCH_AND_RETURN_INSTR(LUI);

  case OP_AUIPC:
    MATCH_AND_RETURN_INSTR(AUIPC);

  case OP_JAL:
    MATCH_AND_RETURN_INSTR(JAL);

  case OP_JALR: {
    MATCH_AND_RETURN_INSTR(JALR);
  }

  case OP_BEQ: {
    switch (instr.funct3()) {
    case F3_BEQ:
      MATCH_AND_RETURN_INSTR(BEQ);
    case F3_BNE:
      MATCH_AND_RETURN_INSTR(BNE);
    case F3_BLT:
      MATCH_AND_RETURN_INSTR(BLT);
    case F3_BGE:
      MATCH_AND_RETURN_INSTR(BGE);
    case F3_BLTU:
      MATCH_AND_RETURN_INSTR(BLTU);
    case F3_BGEU:
      MATCH_AND_RETURN_INSTR(BGEU);
    }
    break;
  }

  case OP_LB: {
    switch (instr.funct3()) {
    case F3_LB:
      MATCH_AND_RETURN_INSTR(LB);
    case F3_LH:
      MATCH_AND_RETURN_INSTR(LH);
    case F3_LW:
      MATCH_AND_RETURN_INSTR(LW);
    case F3_LBU:
      MATCH_AND_RETURN_INSTR(LBU);
    case F3_LHU:
      MATCH_AND_RETURN_INSTR(LHU);
    case F3_LWU:
      MATCH_AND_RETURN_INSTR(LWU);
    case F3_LD:
      MATCH_AND_RETURN_INSTR(LD);
    }
    break;
  }

  case OP_SB: {
    switch (instr.funct3()) {
    case F3_SB:
      MATCH_AND_RETURN_INSTR(SB);
    case F3_SH:
      MATCH_AND_RETURN_INSTR(SH);
    case F3_SW:
      MATCH_AND_RETURN_INSTR(SW);
    case F3_SD:
      MATCH_AND_RETURN_INSTR(SD);
    }
    break;
  }

  case OP_ADDI: {
    switch (instr.funct3()) {
    case F3_ADDI:
      MATCH_AND_RETURN_INSTR(ADDI);
    case F3_SLTI:
      MATCH_AND_RETURN_INSTR(SLTI);
    case F3_SLTIU:
      MATCH_AND_RETURN_INSTR(SLTIU);
    case F3_XORI:
      MATCH_AND_RETURN_INSTR(XORI);
    case F3_ORI:
      MATCH_AND_RETURN_INSTR(ORI);
    case F3_ANDI:
      MATCH_AND_RETURN_INSTR(ANDI);
    case F3_SLLI:
      if (arch == RV32) {
        MATCH_AND_RETURN_INSTR2(SLLI_32, SLLI);
      } else {
        MATCH_AND_RETURN_INSTR(SLLI);
      }
    case F3_SRLI: {
      switch (instr.funct6()) {
      case F6_SRLI:
        if (arch == RV32) {
          MATCH_AND_RETURN_INSTR2(SRLI_32, SRLI);
        } else {
          MATCH_AND_RETURN_INSTR(SRLI);
        }
      case F6_SRAI:
        if (arch == RV32) {
          MATCH_AND_RETURN_INSTR2(SRAI_32, SRAI);
        } else {
          MATCH_AND_RETURN_INSTR(SRAI);
        }
      }
    }
    }
    break;
  }

  case OP_ADDIW: {
    switch (instr.funct3()) {
    case F3_ADDIW:
      MATCH_AND_RETURN_INSTR(ADDIW);
    case F3_SLLIW:
      MATCH_AND_RETURN_INSTR(SLLIW);
    case F3_SRLIW: {
      switch (instr.funct7()) {
      case F7_SRLIW:
        MATCH_AND_RETURN_INSTR(SRLIW);
      case F7_SRAIW:
        MATCH_AND_RETURN_INSTR(SRAIW);
      }
    }
    }
    break;
  }

  case OP_ADD: {
    switch (instr.funct7()) {
    case F7_ADD:
      switch (instr.funct3()) {
      case F3_ADD:
        MATCH_AND_RETURN_INSTR(ADD);
      case F3_SLL:
        MATCH_AND_RETURN_INSTR(SLL);
      case F3_SLT:
        MATCH_AND_RETURN_INSTR(SLT);
      case F3_SLTU:
        MATCH_AND_RETURN_INSTR(SLTU);
      case F3_XOR:
        MATCH_AND_RETURN_INSTR(XOR);
      case F3_SRL:
        MATCH_AND_RETURN_INSTR(SRL);
      case F3_OR:
        MATCH_AND_RETURN_INSTR(OR);
      case F3_AND:
        MATCH_AND_RETURN_INSTR(AND);
      }
      break;

    case F7_SUB:
      switch (instr.funct3()) {
      case F3_SUB:
        MATCH_AND_RETURN_INSTR(SUB);
      case F3_SRA:
        MATCH_AND_RETURN_INSTR(SRA);
      }
      break;

    case F7_MUL:
      switch (instr.funct3()) {
      case F3_MUL:
        MATCH_AND_RETURN_INSTR(MUL);
      case F3_MULH:
        MATCH_AND_RETURN_INSTR(MULH);
      case F3_MULHSU:
        MATCH_AND_RETURN_INSTR(MULHSU);
      case F3_MULHU:
        MATCH_AND_RETURN_INSTR(MULHU);
      case F3_DIV:
        MATCH_AND_RETURN_INSTR(DIV);
      case F3_DIVU:
        MATCH_AND_RETURN_INSTR(DIVU);
      case F3_REM:
        MATCH_AND_RETURN_INSTR(REM);
      case F3_REMU:
        MATCH_AND_RETURN_INSTR(REMU);
      }
      break;
    }
    break;
  }

  case OP_ADDW: {
    switch (instr.funct7()) {
    case F7_ADDW:
      switch (instr.funct3()) {
      case F3_ADDW:
        MATCH_AND_RETURN_INSTR(ADDW);
      case F3_SLLW:
        MATCH_AND_RETURN_INSTR(SLLW);
      case F3_SRLW:
        MATCH_AND_RETURN_INSTR(SRLW);
      }
      break;

    case F7_SUBW:
      switch (instr.funct3()) {
      case F3_SUBW:
        MATCH_AND_RETURN_INSTR(SUBW);
      case F3_SRAW:
        MATCH_AND_RETURN_INSTR(SRAW);
      }
      break;

    case F7_MULW:
      switch (instr.funct3()) {
      case F3_MULW:
        MATCH_AND_RETURN_INSTR(MULW);
      case F3_DIVW:
        MATCH_AND_RETURN_INSTR(DIVW);
      case F3_DIVUW:
        MATCH_AND_RETURN_INSTR(DIVUW);
      case F3_REMW:
        MATCH_AND_RETURN_INSTR(REMW);
      case F3_REMUW:
        MATCH_AND_RETURN_INSTR(REMUW);
      }
      break;
    }
    break;
  }

  case OP_FENCE: {
    switch (instr.funct3()) {
    case F3_FENCE:
      MATCH_AND_RETURN_INSTR(FENCE);
    case F3_FENCE_I:
      MATCH_AND_RETURN_INSTR(FENCE_I);
    }
    break;
  }

  case OP_ECALL: {
    switch (instr.funct3()) {
    case F3_SYS: {
      switch (instr.funct12()) {
      case F12_ECALL:
        MATCH_AND_RETURN_INSTR(ECALL);
      case F12_EBREAK:
        MATCH_AND_RETURN_INSTR(EBREAK);
      case F12_URET:
        MATCH_AND_RETURN_INSTR(URET);
      case F12_SRET:
        MATCH_AND_RETURN_INSTR(SRET);
      case F12_MRET:
        MATCH_AND_RETURN_INSTR(MRET);
      case F12_WFI:
        MATCH_AND_RETURN_INSTR(WFI);
      default:
        MATCH_AND_RETURN_INSTR(SFENCE_VMA);
      }
      break;
    }
    case F3_CSRRW:
      MATCH_AND_RETURN_INSTR(CSRRW);
    case F3_CSRRS:
      MATCH_AND_RETURN_INSTR(CSRRS);
    case F3_CSRRC:
      MATCH_AND_RETURN_INSTR(CSRRC);
    case F3_CSRRWI:
      MATCH_AND_RETURN_INSTR(CSRRWI);
    case F3_CSRRSI:
      MATCH_AND_RETURN_INSTR(CSRRSI);
    case F3_CSRRCI:
      MATCH_AND_RETURN_INSTR(CSRRCI);
    }
    break;
  }

  case OP_AMO: {
    switch (instr.funct5()) {
    case F5_LR_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(LR_D);
      } else {
        MATCH_AND_RETURN_INSTR(LR_W);
      }
    case F5_SC_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(SC_D);
      } else {
        MATCH_AND_RETURN_INSTR(SC_W);
      }
    case F5_AMOSWAP_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOSWAP_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOSWAP_W);
      }
    case F5_AMOADD_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOADD_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOADD_W);
      }
    case F5_AMOXOR_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOXOR_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOXOR_W);
      }
    case F5_AMOAND_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOAND_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOAND_W);
      }
    case F5_AMOOR_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOOR_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOOR_W);
      }
    case F5_AMOMIN_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOMIN_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOMIN_W);
      }
    case F5_AMOMAX_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOMAX_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOMAX_W);
      }
    case F5_AMOMINU_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOMINU_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOMINU_W);
      }
    case F5_AMOMAXU_W:
      if (instr.funct3() == F3_AMO_D) {
        MATCH_AND_RETURN_INSTR(AMOMAXU_D);
      } else {
        MATCH_AND_RETURN_INSTR(AMOMAXU_W);
      }
    }
    break;
  }

  // RV32/64 FD Extension
  case OP_FMADD_S:
    switch (instr.funct2()) {
    case F2_FMADD_S:
      MATCH_AND_RETURN_INSTR(FMADD_S);
    case F2_FMADD_D:
      MATCH_AND_RETURN_INSTR(FMADD_D);
    }
    break;

  case OP_FADD_S:
    switch (instr.funct7()) {
    case F7_FADD_S:
      MATCH_AND_RETURN_INSTR(FADD_S);
    case F7_FADD_D:
      MATCH_AND_RETURN_INSTR(FADD_D);
    case F7_FSUB_S:
      MATCH_AND_RETURN_INSTR(FSUB_S);
    case F7_FSUB_D:
      MATCH_AND_RETURN_INSTR(FSUB_D);
    case F7_FCVT_D_S:
      MATCH_AND_RETURN_INSTR(FCVT_D_S);
    case F7_FMUL_S:
      MATCH_AND_RETURN_INSTR(FMUL_S);
    case F7_FMUL_D:
      MATCH_AND_RETURN_INSTR(FMUL_D);
    case F7_FDIV_S:
      MATCH_AND_RETURN_INSTR(FDIV_S);
    case F7_FDIV_D:
      MATCH_AND_RETURN_INSTR(FDIV_D);
    case F7_FLE_S:
      switch (instr.funct3()) {
      case F3_FLE_S:
        MATCH_AND_RETURN_INSTR(FLE_S);
      case F3_FLT_S:
        MATCH_AND_RETURN_INSTR(FLT_S);
      case F3_FEQ_S:
        MATCH_AND_RETURN_INSTR(FEQ_S);
      }
      break;
    case F7_FSGNJ_D:
      switch (instr.funct3()) {
      case F3_FSGNJ_D:
        MATCH_AND_RETURN_INSTR(FSGNJ_D);
      case F3_FSGNJN_D:
        MATCH_AND_RETURN_INSTR(FSGNJN_D);
      case F3_FSGNJX_D:
        MATCH_AND_RETURN_INSTR(FSGNJX_D);
      }
      break;
    case F7_FMIN_S:
      switch (instr.funct3()) {
      case F3_FMIN_S:
        MATCH_AND_RETURN_INSTR(FMIN_S);
      case F3_FMAX_S:
        MATCH_AND_RETURN_INSTR(FMAX_S);
      }
      break;
    case F7_FMIN_D:
      switch (instr.funct3()) {
      case F3_FMIN_D:
        MATCH_AND_RETURN_INSTR(FMIN_D);
      case F3_FMAX_D:
        MATCH_AND_RETURN_INSTR(FMAX_D);
      }
      break;
    case F7_FCVT_S_D:
      MATCH_AND_RETURN_INSTR(FCVT_S_D);
    case F7_FSGNJ_S:
      switch (instr.funct3()) {
      case F3_FSGNJ_S:
        MATCH_AND_RETURN_INSTR(FSGNJ_S);
      case F3_FSGNJN_S:
        MATCH_AND_RETURN_INSTR(FSGNJN_S);
      case F3_FSGNJX_S:
        MATCH_AND_RETURN_INSTR(FSGNJX_S);
      }
      break;
    case F7_FLE_D:
      switch (instr.funct3()) {
      case F3_FLE_D:
        MATCH_AND_RETURN_INSTR(FLE_D);
      case F3_FLT_D:
        MATCH_AND_RETURN_INSTR(FLT_D);
      case F3_FEQ_D:
        MATCH_AND_RETURN_INSTR(FEQ_D);
      }
      break;
    case F7_FCVT_S_W:
      switch (instr.rs2()) {
      case RS2_FCVT_S_W:
        MATCH_AND_RETURN_INSTR(FCVT_S_W);
      case RS2_FCVT_S_WU:
        MATCH_AND_RETURN_INSTR(FCVT_S_WU);
      case RS2_FCVT_S_L:
        MATCH_AND_RETURN_INSTR(FCVT_S_L);
      case RS2_FCVT_S_LU:
        MATCH_AND_RETURN_INSTR(FCVT_S_LU);
      }
      break;
    case F7_FCVT_D_W:
      switch (instr.rs2()) {
      case RS2_FCVT_D_W:
        MATCH_AND_RETURN_INSTR(FCVT_D_W);
      case RS2_FCVT_D_WU:
        MATCH_AND_RETURN_INSTR(FCVT_D_WU);
      case RS2_FCVT_D_L:
        MATCH_AND_RETURN_INSTR(FCVT_D_L);
      case RS2_FCVT_D_LU:
        MATCH_AND_RETURN_INSTR(FCVT_D_LU);
      }
      break;
    case F7_FCVT_W_D:
      switch (instr.rs2()) {
      case RS2_FCVT_W_D:
        MATCH_AND_RETURN_INSTR(FCVT_W_D);
      case RS2_FCVT_WU_D:
        MATCH_AND_RETURN_INSTR(FCVT_WU_D);
      case RS2_FCVT_L_D:
        MATCH_AND_RETURN_INSTR(FCVT_L_D);
      case RS2_FCVT_LU_D:
        MATCH_AND_RETURN_INSTR(FCVT_LU_D);
      }
      break;
    case F7_FSQRT_S:
      MATCH_AND_RETURN_INSTR(FSQRT_S);
    case F7_FSQRT_D:
      MATCH_AND_RETURN_INSTR(FSQRT_D);
    case F7_FCVT_W_S:
      switch (instr.rs2()) {
      case RS2_FCVT_W_S:
        MATCH_AND_RETURN_INSTR(FCVT_W_S);
      case RS2_FCVT_WU_S:
        MATCH_AND_RETURN_INSTR(FCVT_WU_S);
      case RS2_FCVT_L_S:
        MATCH_AND_RETURN_INSTR(FCVT_L_S);
      case RS2_FCVT_LU_S:
        MATCH_AND_RETURN_INSTR(FCVT_LU_S);
      }
      break;
    case F7_FMV_X_W:
      switch (instr.funct3()) {
      case F3_FMV_X_W:
        MATCH_AND_RETURN_INSTR(FMV_X_W);
      case F3_FCLASS_S:
        MATCH_AND_RETURN_INSTR(FCLASS_S);
      }
      break;
    case F7_FMV_X_D:
      switch (instr.funct3()) {
      case F3_FMV_X_D:
        MATCH_AND_RETURN_INSTR(FMV_X_D);
      case F3_FCLASS_D:
        MATCH_AND_RETURN_INSTR(FCLASS_D);
      }
      break;
    case F7_FMV_W_X:
      MATCH_AND_RETURN_INSTR(FMV_W_X);
    case F7_FMV_D_X:
      MATCH_AND_RETURN_INSTR(FMV_D_X);
    }
    break;
  case OP_FLW:
    switch (instr.funct3()) {
    case F3_FLW:
      MATCH_AND_RETURN_INSTR(FLW);
    case F3_FLD:
      MATCH_AND_RETURN_INSTR(FLD);
    }
    break;
  case OP_FSW:
    switch (instr.funct3()) {
    case F3_FSW:
      MATCH_AND_RETURN_INSTR(FSW);
    case F3_FSD:
      MATCH_AND_RETURN_INSTR(FSD);
    }
    break;
  case OP_FMSUB_S:
    switch (instr.funct2()) {
    case F2_FMSUB_S:
      MATCH_AND_RETURN_INSTR(FMSUB_S);
    case F2_FMSUB_D:
      MATCH_AND_RETURN_INSTR(FMSUB_D);
    }
    break;
  case OP_FNMSUB_S:
    switch (instr.funct2()) {
    case F2_FNMSUB_S:
      MATCH_AND_RETURN_INSTR(FNMSUB_S);
    case F2_FNMSUB_D:
      MATCH_AND_RETURN_INSTR(FNMSUB_D);
    }
    break;
  case OP_FNMADD_S:
    switch (instr.funct2()) {
    case F2_FNMADD_S:
      MATCH_AND_RETURN_INSTR(FNMADD_S);
    case F2_FNMADD_D:
      MATCH_AND_RETURN_INSTR(FNMADD_D);
    }
    break;
  }

  return UNDEF;
}

Opcode::Mapping Instruction::decode(Architecture arch) {
  if (is_compressed())
    return decode_and_expand_compressed(arch);
  return decode_normal(arch);
}
