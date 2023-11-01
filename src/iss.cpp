#include "iss.h"
#include <limits>

using namespace rv32;

#include "to_string.h"

// TODO: apparently the RTL core just returns zero (which is consistent with the
// spec)
//  -> looks rather that the RTL core is inconsistent here ...
#define RAISE_ILLEGAL_INSTRUCTION() raise_trap(EXC_ILLEGAL_INSTR, instr.data());
//#define RAISE_ILLEGAL_INSTRUCTION() raise_trap(EXC_ILLEGAL_INSTR, 0);

#define REQUIRE_ISA(X)                                                         \
  if (!(csrs.misa.reg & X))                                                    \
  RAISE_ILLEGAL_INSTRUCTION()

#define RD instr.rd()
#define RS1 instr.rs1()
#define RS2 instr.rs2()
#define RS3 instr.rs3()

const char *regnames[] = {
    "zero (x0)", "ra   (x1)", "sp   (x2)", "gp   (x3)", "tp   (x4)",
    "t0   (x5)", "t1   (x6)", "t2   (x7)", "s0/fp(x8)", "s1   (x9)",
    "a0  (x10)", "a1  (x11)", "a2  (x12)", "a3  (x13)", "a4  (x14)",
    "a5  (x15)", "a6  (x16)", "a7  (x17)", "s2  (x18)", "s3  (x19)",
    "s4  (x20)", "s5  (x21)", "s6  (x22)", "s7  (x23)", "s8  (x24)",
    "s9  (x25)", "s10 (x26)", "s11 (x27)", "t3  (x28)", "t4  (x29)",
    "t5  (x30)", "t6  (x31)",
};

int regcolors[] = {
#if defined(COLOR_THEME_DARK)
    0,  1,  2,  3,  4,  5,  6,  52, 8,  9,  53, 54, 55, 56, 57, 58,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
#elif defined(COLOR_THEME_LIGHT)
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 153,
    154, 155, 156, 157, 158, 116, 117, 118, 119, 120, 121,
    122, 123, 124, 125, 126, 127, 128, 129, 130, 131,
#else

#endif
};

RegFile::RegFile() { std::fill(std::begin(regs), std::end(regs), 0); }

RegFile::RegFile(const RegFile &other) { regs = other.regs; }

void RegFile::write(uint32_t index, int32_t value) {
  // assert(index <= x31);
  // assert(index != x0);
  regs[index] = value;
}

int32_t RegFile::read(uint32_t index) {
  if (index > x31)
    throw std::out_of_range("out-of-range register access");
  return regs[index];
}

uint32_t RegFile::shamt(uint32_t index) {
  // assert(index <= x31);
  return BIT_RANGE(regs[index], 4, 0);
}

int32_t &RegFile::operator[](const uint32_t idx) { return regs[idx]; }

#if defined(COLOR_THEME_LIGHT) || defined(COLOR_THEME_DARK)
#define COLORFRMT "\e[38;5;%um%s\e[39m"
#define COLORPRINT(fmt, data) fmt, data
#else
#define COLORFRMT "%s"
#define COLORPRINT(fmt, data) data
#endif

void RegFile::show() {
  for (unsigned i = 0; i < NUM_REGS; ++i) {
    printf(COLORFRMT " = %8x\n", COLORPRINT(regcolors[i], regnames[i]),
           regs[i]);
  }
}

ISS::ISS() {
  csrs.mhartid.reg = 0;
  op = Opcode::UNDEF;
}

void ISS::exec_step() {
  // assert(((pc & ~pc_alignment_mask()) == 0) && "misaligned instruction");

  try {
    uint32_t mem_word = instr_mem->load_instr(pc);
    instr = Instruction(mem_word);
  } catch (SimulationTrap &e) {
    op = Opcode::UNDEF;
    instr = Instruction(0);
    throw;
  }

  if (instr.is_compressed()) {
    REQUIRE_ISA(C_ISA_EXT);
    op = instr.decode_and_expand_compressed(RV32);
    pc += 2;
  } else {
    op = instr.decode_normal(RV32);
    pc += 4;
  }

  if (trace) {
    std::cout << std::hex;
    std::string instruction = instr_to_string(instr);
    std::cout << "Instruction: " << instruction << std::endl;
    printf("core %2u: prv %1x: pc %8x: %s ", csrs.mhartid.reg, prv, last_pc,
           Opcode::mappingStr[op]);
    switch (Opcode::getType(op)) {
    case Opcode::Type::R:
      printf(COLORFRMT ", " COLORFRMT ", " COLORFRMT,
             COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
             COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
             COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]));
      break;
    case Opcode::Type::I:
      printf(COLORFRMT ", " COLORFRMT ", 0x%x",
             COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
             COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
             instr.I_imm());
      break;
    case Opcode::Type::S:
      printf(COLORFRMT ", " COLORFRMT ", 0x%x",
             COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
             COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]),
             instr.S_imm());
      break;
    case Opcode::Type::B:
      printf(COLORFRMT ", " COLORFRMT ", 0x%x",
             COLORPRINT(regcolors[instr.rs1()], regnames[instr.rs1()]),
             COLORPRINT(regcolors[instr.rs2()], regnames[instr.rs2()]),
             instr.B_imm());
      break;
    case Opcode::Type::U:
      printf(COLORFRMT ", 0x%x",
             COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
             instr.U_imm());
      break;
    case Opcode::Type::J:
      printf(COLORFRMT ", 0x%x",
             COLORPRINT(regcolors[instr.rd()], regnames[instr.rd()]),
             instr.J_imm());
      break;
    default:;
    }
    puts("");
  }

  switch (op) {
  case Opcode::UNDEF:
    if (trace)
      std::cout << "WARNING: unknown instruction '"
                << std::to_string(instr.data()) << "' at address '"
                << std::to_string(last_pc) << "'" << std::endl;
    raise_trap(EXC_ILLEGAL_INSTR, instr.data());
    break;

  case Opcode::ADDI:
    regs[instr.rd()] = regs[instr.rs1()] + instr.I_imm();
    break;

  case Opcode::SLTI:
    regs[instr.rd()] = regs[instr.rs1()] < instr.I_imm();
    break;

  case Opcode::SLTIU:
    regs[instr.rd()] =
        ((uint32_t)regs[instr.rs1()]) < ((uint32_t)instr.I_imm());
    break;

  case Opcode::XORI:
    regs[instr.rd()] = regs[instr.rs1()] ^ instr.I_imm();
    break;

  case Opcode::ORI:
    regs[instr.rd()] = regs[instr.rs1()] | instr.I_imm();
    break;

  case Opcode::ANDI:
    regs[instr.rd()] = regs[instr.rs1()] & instr.I_imm();
    break;

  case Opcode::ADD:
    regs[instr.rd()] = regs[instr.rs1()] + regs[instr.rs2()];
    break;

  case Opcode::SUB:
    regs[instr.rd()] = regs[instr.rs1()] - regs[instr.rs2()];
    break;

  case Opcode::SLL:
    regs[instr.rd()] = regs[instr.rs1()] << regs.shamt(instr.rs2());
    break;

  case Opcode::SLT:
    regs[instr.rd()] = regs[instr.rs1()] < regs[instr.rs2()];
    break;

  case Opcode::SLTU:
    regs[instr.rd()] =
        ((uint32_t)regs[instr.rs1()]) < ((uint32_t)regs[instr.rs2()]);
    break;

  case Opcode::SRL:
    regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) >> regs.shamt(instr.rs2());
    break;

  case Opcode::SRA:
    regs[instr.rd()] = regs[instr.rs1()] >> regs.shamt(instr.rs2());
    break;

  case Opcode::XOR:
    regs[instr.rd()] = regs[instr.rs1()] ^ regs[instr.rs2()];
    break;

  case Opcode::OR:
    regs[instr.rd()] = regs[instr.rs1()] | regs[instr.rs2()];
    break;

  case Opcode::AND:
    regs[instr.rd()] = regs[instr.rs1()] & regs[instr.rs2()];
    break;

  case Opcode::SLLI:
    regs[instr.rd()] = regs[instr.rs1()] << instr.shamt();
    break;

  case Opcode::SRLI:
    regs[instr.rd()] = ((uint32_t)regs[instr.rs1()]) >> instr.shamt();
    break;

  case Opcode::SRAI:
    regs[instr.rd()] = regs[instr.rs1()] >> instr.shamt();
    break;

  case Opcode::LUI:
    regs[instr.rd()] = instr.U_imm();
    break;

  case Opcode::AUIPC:
    regs[instr.rd()] = last_pc + instr.U_imm();
    break;

  case Opcode::JAL: {
    auto link = pc;
    pc = last_pc + instr.J_imm();
    trap_check_pc_alignment();
    regs[instr.rd()] = link;
  } break;

  case Opcode::JALR: {
    auto link = pc;
    pc = (regs[instr.rs1()] + instr.I_imm()) & ~1;
    trap_check_pc_alignment();
    regs[instr.rd()] = link;
  } break;

  case Opcode::SB: {
    uint32_t addr = regs[instr.rs1()] + instr.S_imm();
    mem->store_byte(addr, regs[instr.rs2()]);
  } break;

  case Opcode::SH: {
    uint32_t addr = regs[instr.rs1()] + instr.S_imm();
    trap_check_addr_alignment<2, false>(addr);
    mem->store_half(addr, regs[instr.rs2()]);
  } break;

  case Opcode::SW: {
    uint32_t addr = regs[instr.rs1()] + instr.S_imm();
    trap_check_addr_alignment<4, false>(addr);
    mem->store_word(addr, regs[instr.rs2()]);
  } break;

  case Opcode::LB: {
    uint32_t addr = regs[instr.rs1()] + instr.I_imm();
    regs[instr.rd()] = mem->load_byte(addr);
  } break;

  case Opcode::LH: {
    uint32_t addr = regs[instr.rs1()] + instr.I_imm();
    trap_check_addr_alignment<2, true>(addr);
    regs[instr.rd()] = mem->load_half(addr);
  } break;

  case Opcode::LW: {
    uint32_t addr = regs[instr.rs1()] + instr.I_imm();
    trap_check_addr_alignment<4, true>(addr);
    regs[instr.rd()] = mem->load_word(addr);
  } break;

  case Opcode::LBU: {
    uint32_t addr = regs[instr.rs1()] + instr.I_imm();
    regs[instr.rd()] = mem->load_ubyte(addr);
  } break;

  case Opcode::LHU: {
    uint32_t addr = regs[instr.rs1()] + instr.I_imm();
    trap_check_addr_alignment<2, true>(addr);
    regs[instr.rd()] = mem->load_uhalf(addr);
  } break;

  case Opcode::BEQ:
    if (regs[instr.rs1()] == regs[instr.rs2()]) {
      pc = last_pc + instr.B_imm();
      trap_check_pc_alignment();
    }
    break;

  case Opcode::BNE:
    if (regs[instr.rs1()] != regs[instr.rs2()]) {
      pc = last_pc + instr.B_imm();
      trap_check_pc_alignment();
    }
    break;

  case Opcode::BLT:
    if (regs[instr.rs1()] < regs[instr.rs2()]) {
      pc = last_pc + instr.B_imm();
      trap_check_pc_alignment();
    }
    break;

  case Opcode::BGE:
    if (regs[instr.rs1()] >= regs[instr.rs2()]) {
      pc = last_pc + instr.B_imm();
      trap_check_pc_alignment();
    }
    break;

  case Opcode::BLTU:
    if ((uint32_t)regs[instr.rs1()] < (uint32_t)regs[instr.rs2()]) {
      pc = last_pc + instr.B_imm();
      trap_check_pc_alignment();
    }
    break;

  case Opcode::BGEU:
    if ((uint32_t)regs[instr.rs1()] >= (uint32_t)regs[instr.rs2()]) {
      pc = last_pc + instr.B_imm();
      trap_check_pc_alignment();
    }
    break;

  case Opcode::FENCE:
  case Opcode::FENCE_I: {
    // not using out of order execution so can be ignored
  } break;

  case Opcode::ECALL: {
    switch (prv) {
    case MachineMode:
      raise_trap(EXC_ECALL_M_MODE, 0);
      break;
    case SupervisorMode:
      raise_trap(EXC_ECALL_S_MODE, 0);
      break;
    case UserMode:
      raise_trap(EXC_ECALL_U_MODE, 0);
      break;
    default:
      throw std::runtime_error("unknown privilege level " +
                               std::to_string(prv));
    }
  } break;

  case Opcode::EBREAK: {
    raise_trap(EXC_BREAKPOINT, last_pc);
  } break;

  case Opcode::CSRRW: {
    auto addr = instr.csr();
    if (m_is_invalid_csr_access(addr, true)) {
      RAISE_ILLEGAL_INSTRUCTION();
    } else {
      auto rd = instr.rd();
      auto rs1_val = regs[instr.rs1()];
      if (rd != RegFile::zero) {
        regs[instr.rd()] = m_get_csr_value(addr);
      }
      m_set_csr_value(addr, rs1_val);
    }
  } break;

  case Opcode::CSRRS: {
    auto addr = instr.csr();
    auto rs1 = instr.rs1();
    auto write = rs1 != RegFile::zero;
    if (m_is_invalid_csr_access(addr, write)) {
      RAISE_ILLEGAL_INSTRUCTION();
    } else {
      auto rd = instr.rd();
      auto rs1_val = regs[rs1];
      auto csr_val = m_get_csr_value(addr);
      if (rd != RegFile::zero)
        regs[rd] = csr_val;
      if (write)
        m_set_csr_value(addr, csr_val | rs1_val);
    }
  } break;

  case Opcode::CSRRC: {
    auto addr = instr.csr();
    auto rs1 = instr.rs1();
    auto write = rs1 != RegFile::zero;
    if (m_is_invalid_csr_access(addr, write)) {
      RAISE_ILLEGAL_INSTRUCTION();
    } else {
      auto rd = instr.rd();
      auto rs1_val = regs[rs1];
      auto csr_val = m_get_csr_value(addr);
      if (rd != RegFile::zero)
        regs[rd] = csr_val;
      if (write)
        m_set_csr_value(addr, csr_val & ~rs1_val);
    }
  } break;

  case Opcode::CSRRWI: {
    auto addr = instr.csr();
    if (m_is_invalid_csr_access(addr, true)) {
      RAISE_ILLEGAL_INSTRUCTION();
    } else {
      auto rd = instr.rd();
      if (rd != RegFile::zero) {
        regs[rd] = m_get_csr_value(addr);
      }
      m_set_csr_value(addr, instr.zimm());
    }
  } break;

  case Opcode::CSRRSI: {
    auto addr = instr.csr();
    auto zimm = instr.zimm();
    auto write = zimm != 0;
    if (m_is_invalid_csr_access(addr, write)) {
      RAISE_ILLEGAL_INSTRUCTION();
    } else {
      auto csr_val = m_get_csr_value(addr);
      auto rd = instr.rd();
      if (rd != RegFile::zero)
        regs[rd] = csr_val;
      if (write)
        m_set_csr_value(addr, csr_val | zimm);
    }
  } break;

  case Opcode::CSRRCI: {
    auto addr = instr.csr();
    auto zimm = instr.zimm();
    auto write = zimm != 0;
    if (m_is_invalid_csr_access(addr, write)) {
      RAISE_ILLEGAL_INSTRUCTION();
    } else {
      auto csr_val = m_get_csr_value(addr);
      auto rd = instr.rd();
      if (rd != RegFile::zero)
        regs[rd] = csr_val;
      if (write)
        m_set_csr_value(addr, csr_val & ~zimm);
    }
  } break;

  case Opcode::MUL: {
    REQUIRE_ISA(M_ISA_EXT);
    int64_t ans = (int64_t)regs[instr.rs1()] * (int64_t)regs[instr.rs2()];
    regs[instr.rd()] = ans & 0xFFFFFFFF;
  } break;

  case Opcode::MULH: {
    REQUIRE_ISA(M_ISA_EXT);
    int64_t ans = (int64_t)regs[instr.rs1()] * (int64_t)regs[instr.rs2()];
    regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
  } break;

  case Opcode::MULHU: {
    REQUIRE_ISA(M_ISA_EXT);
    int64_t ans = ((uint64_t)(uint32_t)regs[instr.rs1()]) *
                  (uint64_t)((uint32_t)regs[instr.rs2()]);
    regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
  } break;

  case Opcode::MULHSU: {
    REQUIRE_ISA(M_ISA_EXT);
    int64_t ans =
        (int64_t)regs[instr.rs1()] * (uint64_t)((uint32_t)regs[instr.rs2()]);
    regs[instr.rd()] = (ans & 0xFFFFFFFF00000000) >> 32;
  } break;

  case Opcode::DIV: {
    REQUIRE_ISA(M_ISA_EXT);
    auto a = regs[instr.rs1()];
    auto b = regs[instr.rs2()];
    if (b == 0) {
      regs[instr.rd()] = -1;
    } else if (a == REG_MIN && b == -1) {
      regs[instr.rd()] = a;
    } else {
      regs[instr.rd()] = a / b;
    }
  } break;

  case Opcode::DIVU: {
    REQUIRE_ISA(M_ISA_EXT);
    auto a = regs[instr.rs1()];
    auto b = regs[instr.rs2()];
    if (b == 0) {
      regs[instr.rd()] = -1;
    } else {
      regs[instr.rd()] = (uint32_t)a / (uint32_t)b;
    }
  } break;

  case Opcode::REM: {
    REQUIRE_ISA(M_ISA_EXT);
    auto a = regs[instr.rs1()];
    auto b = regs[instr.rs2()];
    if (b == 0) {
      regs[instr.rd()] = a;
    } else if (a == REG_MIN && b == -1) {
      regs[instr.rd()] = 0;
    } else {
      regs[instr.rd()] = a % b;
    }
  } break;

  case Opcode::REMU: {
    REQUIRE_ISA(M_ISA_EXT);
    auto a = regs[instr.rs1()];
    auto b = regs[instr.rs2()];
    if (b == 0) {
      regs[instr.rd()] = a;
    } else {
      regs[instr.rd()] = (uint32_t)a % (uint32_t)b;
    }
  } break;

    // RV32A Extension
  case Opcode::LR_W:
  case Opcode::SC_W:
  case Opcode::AMOSWAP_W:
  case Opcode::AMOADD_W:
  case Opcode::AMOXOR_W:
  case Opcode::AMOAND_W:
  case Opcode::AMOOR_W:
  case Opcode::AMOMIN_W:
  case Opcode::AMOMINU_W:
  case Opcode::AMOMAX_W:
  case Opcode::AMOMAXU_W:
    // RV32F Extension
  case Opcode::FLW:
  case Opcode::FSW:
  case Opcode::FADD_S:
  case Opcode::FSUB_S:
  case Opcode::FMUL_S:
  case Opcode::FDIV_S:
  case Opcode::FSQRT_S:
  case Opcode::FMIN_S:
  case Opcode::FMAX_S:
  case Opcode::FMADD_S:
  case Opcode::FMSUB_S:
  case Opcode::FNMADD_S:
  case Opcode::FNMSUB_S:
  case Opcode::FCVT_W_S:
  case Opcode::FCVT_WU_S:
  case Opcode::FCVT_S_W:
  case Opcode::FCVT_S_WU:
  case Opcode::FSGNJ_S:
  case Opcode::FSGNJN_S:
  case Opcode::FSGNJX_S:
  case Opcode::FMV_W_X:
  case Opcode::FMV_X_W:
  case Opcode::FEQ_S:
  case Opcode::FLT_S:
  case Opcode::FLE_S:
  case Opcode::FCLASS_S:
    // RV32D Extension
  case Opcode::FLD:
  case Opcode::FSD:
  case Opcode::FADD_D:
  case Opcode::FSUB_D:
  case Opcode::FMUL_D:
  case Opcode::FDIV_D:
  case Opcode::FSQRT_D:
  case Opcode::FMIN_D:
  case Opcode::FMAX_D:
  case Opcode::FMADD_D:
  case Opcode::FMSUB_D:
  case Opcode::FNMADD_D:
  case Opcode::FNMSUB_D:
  case Opcode::FSGNJ_D:
  case Opcode::FSGNJN_D:
  case Opcode::FSGNJX_D:
  case Opcode::FCVT_S_D:
  case Opcode::FCVT_D_S:
  case Opcode::FEQ_D:
  case Opcode::FLT_D:
  case Opcode::FLE_D:
  case Opcode::FCLASS_D:
  case Opcode::FCVT_W_D:
  case Opcode::FCVT_WU_D:
  case Opcode::FCVT_D_W:
  case Opcode::FCVT_D_WU:
    RAISE_ILLEGAL_INSTRUCTION();

    // privileged instructions

  case Opcode::WFI:
    // NOTE: only a hint, can be implemented as NOP
    // std::cout << "[sim:wfi] CSR mstatus.mie " << csrs.mstatus->mie <<
    // std::endl;

    if (s_mode() && csrs.mstatus.tw)
      raise_trap(EXC_ILLEGAL_INSTR, instr.data());

    if (u_mode() && csrs.misa.has_supervisor_mode_extension())
      raise_trap(EXC_ILLEGAL_INSTR, instr.data());
    break;

  case Opcode::SFENCE_VMA:
    if (!csrs.misa.has_supervisor_mode_extension())
      raise_trap(EXC_ILLEGAL_INSTR, instr.data());

    if (s_mode() && csrs.mstatus.tvm)
      raise_trap(EXC_ILLEGAL_INSTR, instr.data());
    break;

  case Opcode::URET:
    if (!csrs.misa.has_user_mode_extension())
      raise_trap(EXC_ILLEGAL_INSTR, instr.data());
    return_from_trap_handler(UserMode);
    break;

  case Opcode::SRET:
    if (!csrs.misa.has_supervisor_mode_extension() ||
        (s_mode() && csrs.mstatus.tsr))
      raise_trap(EXC_ILLEGAL_INSTR, instr.data());
    return_from_trap_handler(SupervisorMode);
    break;

  case Opcode::MRET:
    return_from_trap_handler(MachineMode);
    break;

    // instructions accepted by decoder but not by this RV32IMACF ISS -> do
    // normal trap RV64I
  case Opcode::LWU:
  case Opcode::LD:
  case Opcode::SD:
  case Opcode::ADDIW:
  case Opcode::SLLIW:
  case Opcode::SRLIW:
  case Opcode::SRAIW:
  case Opcode::ADDW:
  case Opcode::SUBW:
  case Opcode::SLLW:
  case Opcode::SRLW:
  case Opcode::SRAW:
    // RV64M
  case Opcode::MULW:
  case Opcode::DIVW:
  case Opcode::DIVUW:
  case Opcode::REMW:
  case Opcode::REMUW:
    // RV64A
  case Opcode::LR_D:
  case Opcode::SC_D:
  case Opcode::AMOSWAP_D:
  case Opcode::AMOADD_D:
  case Opcode::AMOXOR_D:
  case Opcode::AMOAND_D:
  case Opcode::AMOOR_D:
  case Opcode::AMOMIN_D:
  case Opcode::AMOMAX_D:
  case Opcode::AMOMINU_D:
  case Opcode::AMOMAXU_D:
    // RV64F
  case Opcode::FCVT_L_S:
  case Opcode::FCVT_LU_S:
  case Opcode::FCVT_S_L:
  case Opcode::FCVT_S_LU:
    // RV64D
  case Opcode::FCVT_L_D:
  case Opcode::FCVT_LU_D:
  case Opcode::FMV_X_D:
  case Opcode::FCVT_D_L:
  case Opcode::FCVT_D_LU:
  case Opcode::FMV_D_X:
    RAISE_ILLEGAL_INSTRUCTION();

  default:
    throw std::runtime_error("unknown opcode");
  }
}

bool ISS::m_is_invalid_csr_access(uint32_t csr_addr, bool is_write) {
  PrivilegeLevel csr_prv = (0x300 & csr_addr) >> 8;
  bool csr_readonly = ((0xC00 & csr_addr) >> 10) == 3;
  return (is_write && csr_readonly) || (prv < csr_prv);
}

bool ISS::is_invalid_csr_access(uint32_t csr_addr, bool is_write) {
  if (csr_addr == csr::FFLAGS_ADDR || csr_addr == csr::FRM_ADDR ||
      csr_addr == csr::FCSR_ADDR) {
    REQUIRE_ISA(F_ISA_EXT);
  }
  PrivilegeLevel csr_prv = (0x300 & csr_addr) >> 8;
  bool csr_readonly = ((0xC00 & csr_addr) >> 10) == 3;
  bool s_invalid =
      (csr_prv == SupervisorMode) && !csrs.misa.has_supervisor_mode_extension();
  bool u_invalid =
      (csr_prv == UserMode) && !csrs.misa.has_user_mode_extension();
  return (is_write && csr_readonly) || (prv < csr_prv) || s_invalid ||
         u_invalid;
}

void ISS::validate_csr_counter_read_access_rights(uint32_t addr) {
  // match against counter CSR addresses, see RISC-V privileged spec for the
  // address definitions
  if ((addr >= 0xC00 && addr <= 0xC1F) || (addr >= 0xC80 && addr <= 0xC9F)) {
    auto cnt = addr & 0x1F; // 32 counter in total, naturally aligned with the
                            // mcounteren and scounteren CSRs

    if (s_mode() && !csr::is_bitset(csrs.mcounteren, cnt))
      RAISE_ILLEGAL_INSTRUCTION();

    if (u_mode() && (!csr::is_bitset(csrs.mcounteren, cnt) ||
                     !csr::is_bitset(csrs.scounteren, cnt)))
      RAISE_ILLEGAL_INSTRUCTION();
  }
}

uint32_t ISS::m_get_csr_value(uint32_t addr) {
  using namespace csr;

  switch (addr) {
  case TIME_ADDR:
    csrs.time.reg = csrs.instret.reg;
    return csrs.time.low;

  case TIMEH_ADDR:
    csrs.time.reg = csrs.instret.reg;
    return csrs.time.high;

  case CYCLE_ADDR:
  case MCYCLE_ADDR:
    return csrs.cycle.low;

  case CYCLEH_ADDR:
  case MCYCLEH_ADDR:
    return csrs.cycle.high;

  case INSTRET_ADDR:
  case MINSTRET_ADDR:
    return csrs.instret.low;

  case INSTRETH_ADDR:
  case MINSTRETH_ADDR:
    return csrs.instret.high;

  case MVENDORID_ADDR:
    return csrs.mvendorid.reg;
  case MARCHID_ADDR:
    return csrs.marchid.reg;
  case MIMPID_ADDR:
    return csrs.mimpid.reg;
  case MHARTID_ADDR:
    return csrs.mhartid.reg;

  case MSTATUS_ADDR:
    return csrs.mstatus.reg & M_MSTATUS_MASK;
  case MISA_ADDR:
    return csrs.misa.reg;
  case MIE_ADDR:
    return csrs.mie.reg & M_MIE_MASK;
  case MTVEC_ADDR:
    return csrs.mtvec.reg & M_MTVEC_MASK;

  case MSCRATCH_ADDR:
    return csrs.mscratch.reg;
  case MEPC_ADDR:
    return csrs.mepc.reg & pc_alignment_mask();
  case MCAUSE_ADDR:
    return csrs.mcause.reg & M_MCAUSE_MASK;
  case MTVAL_ADDR:
    return csrs.mtval.reg;
  case MIP_ADDR:
    return csrs.mip.reg & M_MIP_MASK;

  SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32:
  case MCOUNTEREN_ADDR:
    return 0;
  }

  RAISE_ILLEGAL_INSTRUCTION();

  // Should not be reached.
  // returns maximum value, because return 0 is already used
  return std::numeric_limits<uint32_t>::max(); 
}

void ISS::m_set_csr_value(uint32_t addr, uint32_t value) {
  auto write = [=](auto &x, uint32_t mask) {
    x.reg = (x.reg & ~mask) | (value & mask);
  };

  using namespace csr;

  switch (addr) {
  case MCYCLE_ADDR:
    csrs.cycle.low = value;
    return;
  case MCYCLEH_ADDR:
    csrs.cycle.high = value;
    return;

  case MINSTRET_ADDR:
    csrs.instret.low = value;
    if (!csrs.mcountinhibit.IR)
      csrs.instret.reg--;
    return;
  case MINSTRETH_ADDR:
    csrs.instret.high = value;
    if (!csrs.mcountinhibit.IR)
      csrs.instret.reg--;
    return;

  case MSTATUS_ADDR:
    write(csrs.mstatus, M_MSTATUS_MASK);
    csrs.mstatus.mpp = 0b11; // TODO: assume for now
    return;
  case MISA_ADDR:
    return; // read-only, thus cannot be changed at runtime
  case MIE_ADDR:
    write(csrs.mie, M_MIE_MASK);
    return;
  case MTVEC_ADDR:
    write(csrs.mtvec, M_MTVEC_MASK);
    return;

  case MSCRATCH_ADDR:
    csrs.mscratch.reg = value;
    return;
  case MEPC_ADDR:
    write(csrs.mepc, ~1);
    return;
  case MCAUSE_ADDR:
    write(csrs.mcause, M_MCAUSE_MASK);
    return;
  case MTVAL_ADDR:
    csrs.mtval.reg = value;
    return;
  case MIP_ADDR:
    // write(csrs.mip, M_MIP_WRITE_MASK); // match the read-only part of the RTL
    // core
    return; // actually seems like it should be read-only according to the
            // RISC-V spec (and seems to be the behavior of the RTL core as
            // well)

  SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32:
  case MCOUNTEREN_ADDR:
    return; // hard-wired to zero
  }

  RAISE_ILLEGAL_INSTRUCTION();
}

uint32_t ISS::get_csr_value(uint32_t addr) {
  validate_csr_counter_read_access_rights(addr);

  auto read = [=](auto &x, uint32_t mask) { return x.reg & mask; };

  using namespace csr;

  switch (addr) {
  case TIME_ADDR:
  case MTIME_ADDR: {
    csrs.time.reg = csrs.instret.reg;
    return csrs.time.low;
  }

  case TIMEH_ADDR:
  case MTIMEH_ADDR: {
    csrs.time.reg = csrs.instret.reg;
    return csrs.time.high;
  }

  case CYCLE_ADDR:
  case MCYCLE_ADDR:
    return csrs.cycle.low;

  case CYCLEH_ADDR:
  case MCYCLEH_ADDR:
    return csrs.cycle.high;

  case INSTRET_ADDR:
  case MINSTRET_ADDR:
    return csrs.instret.low;

  case INSTRETH_ADDR:
  case MINSTRETH_ADDR:
    return csrs.instret.high;

  SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32: // not implemented
    return 0;

  case MSTATUS_ADDR:
    return read(csrs.mstatus, MSTATUS_MASK);
  case SSTATUS_ADDR:
    return read(csrs.mstatus, SSTATUS_MASK);
  case USTATUS_ADDR:
    return read(csrs.mstatus, USTATUS_MASK);

  case MIP_ADDR:
    return read(csrs.mip, MIP_READ_MASK);
  case SIP_ADDR:
    return read(csrs.mip, SIP_MASK);
  case UIP_ADDR:
    return read(csrs.mip, UIP_MASK);

  case MIE_ADDR:
    return read(csrs.mie, MIE_MASK);
  case SIE_ADDR:
    return read(csrs.mie, SIE_MASK);
  case UIE_ADDR:
    return read(csrs.mie, UIE_MASK);

  case SATP_ADDR:
    if (csrs.mstatus.tvm)
      RAISE_ILLEGAL_INSTRUCTION();
    break;

  case FCSR_ADDR:
    return read(csrs.fcsr, FCSR_MASK);

  case FFLAGS_ADDR:
    return csrs.fcsr.fflags;

  case FRM_ADDR:
    return csrs.fcsr.frm;

  // debug CSRs not supported, thus hardwired
  case TSELECT_ADDR:
    return 1; // if a zero write by SW is preserved, then debug mode is
              // supported (thus hardwire to non-zero)
  case TDATA1_ADDR:
  case TDATA2_ADDR:
  case TDATA3_ADDR:
  case DCSR_ADDR:
  case DPC_ADDR:
  case DSCRATCH0_ADDR:
  case DSCRATCH1_ADDR:
    return 0;
  }

  if (!csrs.is_valid_csr32_addr(addr))
    RAISE_ILLEGAL_INSTRUCTION();

  return csrs.default_read32(addr);
}

void ISS::set_csr_value(uint32_t addr, uint32_t value) {
  auto write = [=](auto &x, uint32_t mask) {
    x.reg = (x.reg & ~mask) | (value & mask);
  };

  using namespace csr;

  switch (addr) {
  case MISA_ADDR: // currently, read-only, thus cannot be changed at runtime
  SWITCH_CASE_MATCH_ANY_HPMCOUNTER_RV32: // not implemented
    break;

  case SATP_ADDR: {
    if (csrs.mstatus.tvm)
      RAISE_ILLEGAL_INSTRUCTION();
    write(csrs.satp, SATP_MASK);
    // std::cout << "[iss] satp=" << boost::format("%x") % csrs.satp.reg <<
    // std::endl;
  } break;

  case MTVEC_ADDR:
    write(csrs.mtvec, MTVEC_MASK);
    break;
  case STVEC_ADDR:
    write(csrs.stvec, MTVEC_MASK);
    break;
  case UTVEC_ADDR:
    write(csrs.utvec, MTVEC_MASK);
    break;

  case MEPC_ADDR:
    write(csrs.mepc, pc_alignment_mask());
    break;
  case SEPC_ADDR:
    write(csrs.sepc, pc_alignment_mask());
    break;
  case UEPC_ADDR:
    write(csrs.uepc, pc_alignment_mask());
    break;

  case MSTATUS_ADDR:
    write(csrs.mstatus, MSTATUS_MASK);
    break;
  case SSTATUS_ADDR:
    write(csrs.mstatus, SSTATUS_MASK);
    break;
  case USTATUS_ADDR:
    write(csrs.mstatus, USTATUS_MASK);
    break;

  case MIP_ADDR:
    write(csrs.mip, MIP_WRITE_MASK);
    break;
  case SIP_ADDR:
    write(csrs.mip, SIP_MASK);
    break;
  case UIP_ADDR:
    write(csrs.mip, UIP_MASK);
    break;

  case MIE_ADDR:
    write(csrs.mie, MIE_MASK);
    break;
  case SIE_ADDR:
    write(csrs.mie, SIE_MASK);
    break;
  case UIE_ADDR:
    write(csrs.mie, UIE_MASK);
    break;

  case MIDELEG_ADDR:
    write(csrs.mideleg, MIDELEG_MASK);
    break;

  case MEDELEG_ADDR:
    write(csrs.medeleg, MEDELEG_MASK);
    break;

  case SIDELEG_ADDR:
    write(csrs.sideleg, SIDELEG_MASK);
    break;

  case SEDELEG_ADDR:
    write(csrs.sedeleg, SEDELEG_MASK);
    break;

  case MCOUNTEREN_ADDR:
    write(csrs.mcounteren, MCOUNTEREN_MASK);
    break;

  case SCOUNTEREN_ADDR:
    write(csrs.scounteren, MCOUNTEREN_MASK);
    break;

  case MCOUNTINHIBIT_ADDR:
    write(csrs.mcountinhibit, MCOUNTINHIBIT_MASK);
    break;

  case FCSR_ADDR:
    write(csrs.fcsr, FCSR_MASK);
    break;

  case FFLAGS_ADDR:
    csrs.fcsr.fflags = value;
    break;

  case FRM_ADDR:
    csrs.fcsr.frm = value;
    break;

  // debug CSRs not supported, thus hardwired
  case TSELECT_ADDR:
  case TDATA1_ADDR:
  case TDATA2_ADDR:
  case TDATA3_ADDR:
  case DCSR_ADDR:
  case DPC_ADDR:
  case DSCRATCH0_ADDR:
  case DSCRATCH1_ADDR:
    break;

  default:
    if (!csrs.is_valid_csr32_addr(addr))
      RAISE_ILLEGAL_INSTRUCTION();

    csrs.default_write32(addr, value);
  }
}

void ISS::return_from_trap_handler(PrivilegeLevel return_mode) {
  switch (return_mode) {
  case MachineMode:
    prv = csrs.mstatus.mpp;
    csrs.mstatus.mie = csrs.mstatus.mpie;
    csrs.mstatus.mpie = 1;
    pc = csrs.mepc.reg & pc_alignment_mask();
    if (csrs.misa.has_user_mode_extension())
      csrs.mstatus.mpp = UserMode;
    else
      csrs.mstatus.mpp = MachineMode;
    break;

  case SupervisorMode:
    prv = csrs.mstatus.spp;
    csrs.mstatus.sie = csrs.mstatus.spie;
    csrs.mstatus.spie = 1;
    pc = csrs.sepc.reg;
    if (csrs.misa.has_user_mode_extension())
      csrs.mstatus.spp = UserMode;
    else
      csrs.mstatus.spp = SupervisorMode;
    break;

  case UserMode:
    prv = UserMode;
    csrs.mstatus.uie = csrs.mstatus.upie;
    csrs.mstatus.upie = 1;
    pc = csrs.uepc.reg;
    break;

  default:
    throw std::runtime_error("unknown privilege level " +
                             std::to_string(return_mode));
  }
}

void ISS::trigger_external_interrupt(PrivilegeLevel level) {
  switch (level) {
  case UserMode:
    csrs.mip.ueip = true;
    break;
  case SupervisorMode:
    csrs.mip.seip = true;
    break;
  case MachineMode:
    csrs.mip.meip = true;
    break;
  }
}

void ISS::clear_external_interrupt(PrivilegeLevel level) {
  switch (level) {
  case UserMode:
    csrs.mip.ueip = false;
    break;
  case SupervisorMode:
    csrs.mip.seip = false;
    break;
  case MachineMode:
    csrs.mip.meip = false;
    break;
  }
}

void ISS::trigger_timer_interrupt(bool status) { csrs.mip.mtip = status; }

void ISS::trigger_software_interrupt(bool status) { csrs.mip.msip = status; }

PrivilegeLevel ISS::prepare_trap(SimulationTrap &e) {
  // undo any potential pc update (for traps the pc should point to the
  // originating instruction and not it's successor)
  pc = last_pc;
  unsigned exc_bit = (1 << e.reason);

  // 1) machine mode execution takes any traps, independent of delegation
  // setting 2) non-delegated traps are processed in machine mode, independent
  // of current execution mode
  if (prv == MachineMode || !(exc_bit & csrs.medeleg.reg)) {
    csrs.mcause.interrupt = 0;
    csrs.mcause.exception_code = e.reason;
    csrs.mtval.reg = e.mtval;
    return MachineMode;
  }

  // see above machine mode comment
  if (prv == SupervisorMode || !(exc_bit & csrs.sedeleg.reg)) {
    csrs.scause.interrupt = 0;
    csrs.scause.exception_code = e.reason;
    csrs.stval.reg = e.mtval;
    return SupervisorMode;
  }

  // assert(prv == UserMode && (exc_bit & csrs.medeleg.reg) && (exc_bit &
  // csrs.sedeleg.reg));
  csrs.ucause.interrupt = 0;
  csrs.ucause.exception_code = e.reason;
  csrs.utval.reg = e.mtval;
  return UserMode;
}

void ISS::prepare_interrupt(const PendingInterrupts &e) {
  if (trace) {
    std::cout << "[vp::iss] prepare interrupt, pending=" << e.pending
              << ", target-mode=" << e.target_mode << std::endl;
  }

  csr_mip x{e.pending};

  ExceptionCode exc;
  if (x.meip)
    exc = EXC_M_EXTERNAL_INTERRUPT;
  else if (x.msip)
    exc = EXC_M_SOFTWARE_INTERRUPT;
  else if (x.mtip)
    exc = EXC_M_TIMER_INTERRUPT;
  else if (x.seip)
    exc = EXC_S_EXTERNAL_INTERRUPT;
  else if (x.ssip)
    exc = EXC_S_SOFTWARE_INTERRUPT;
  else if (x.stip)
    exc = EXC_S_TIMER_INTERRUPT;
  else if (x.ueip)
    exc = EXC_U_EXTERNAL_INTERRUPT;
  else if (x.usip)
    exc = EXC_U_SOFTWARE_INTERRUPT;
  else if (x.utip)
    exc = EXC_U_TIMER_INTERRUPT;
  else
    throw std::runtime_error("some pending interrupt must be available here");

  switch (e.target_mode) {
  case MachineMode:
    csrs.mcause.exception_code = exc;
    csrs.mcause.interrupt = 1;
    break;

  case SupervisorMode:
    csrs.scause.exception_code = exc;
    csrs.scause.interrupt = 1;
    break;

  case UserMode:
    csrs.ucause.exception_code = exc;
    csrs.ucause.interrupt = 1;
    break;

  default:
    throw std::runtime_error("unknown privilege level " +
                             std::to_string(e.target_mode));
  }
}

PendingInterrupts ISS::compute_pending_interrupts() {
  uint32_t pending = csrs.mie.reg & csrs.mip.reg;

  if (!pending)
    return {NoneMode, 0};

  auto m_pending = pending & ~csrs.mideleg.reg;
  if (m_pending &&
      (prv < MachineMode || (prv == MachineMode && csrs.mstatus.mie))) {
    return {MachineMode, m_pending};
  }

  pending = pending & csrs.mideleg.reg;
  auto s_pending = pending & ~csrs.sideleg.reg;
  if (s_pending &&
      (prv < SupervisorMode || (prv == SupervisorMode && csrs.mstatus.sie))) {
    return {SupervisorMode, s_pending};
  }

  auto u_pending = pending & csrs.sideleg.reg;
  if (u_pending && (prv == UserMode && csrs.mstatus.uie)) {
    return {UserMode, u_pending};
  }

  return {NoneMode, 0};
}

void ISS::switch_to_trap_handler(PrivilegeLevel target_mode) {
  auto pp = prv;
  prv = target_mode;

  switch (target_mode) {
  case MachineMode:
    csrs.mepc.reg = pc;

    csrs.mstatus.mpie = csrs.mstatus.mie;
    csrs.mstatus.mie = 0;
    csrs.mstatus.mpp = pp;

    pc = csrs.mtvec.get_base_address();

    if (csrs.mcause.interrupt && csrs.mtvec.mode == csrs.mtvec.Vectored)
      pc += 4 * csrs.mcause.exception_code;
    break;

  case SupervisorMode:
    // assert(prv == SupervisorMode || prv == UserMode);

    csrs.sepc.reg = pc;

    csrs.mstatus.spie = csrs.mstatus.sie;
    csrs.mstatus.sie = 0;
    csrs.mstatus.spp = pp;

    pc = csrs.stvec.get_base_address();

    if (csrs.scause.interrupt && csrs.stvec.mode == csrs.stvec.Vectored)
      pc += 4 * csrs.scause.exception_code;
    break;

  case UserMode:
    // assert(prv == UserMode);

    csrs.uepc.reg = pc;

    csrs.mstatus.upie = csrs.mstatus.uie;
    csrs.mstatus.uie = 0;

    pc = csrs.utvec.get_base_address();

    if (csrs.ucause.interrupt && csrs.utvec.mode == csrs.utvec.Vectored)
      pc += 4 * csrs.ucause.exception_code;
    break;

  default:
    throw std::runtime_error("unknown privilege level " +
                             std::to_string(target_mode));
  }
}

void ISS::performance_and_sync_update() {
  ++total_num_instr;

  if (!csrs.mcountinhibit.IR)
    ++csrs.instret.reg;

  if (!csrs.mcountinhibit.CY)
    ++csrs.cycle.reg;
}

void ISS::run_step() {
  // assert(regs.read(0) == 0);

  last_pc = pc;
  try {
    exec_step();

    auto x = compute_pending_interrupts();
    if (x.target_mode != NoneMode) {
      prepare_interrupt(x);
      switch_to_trap_handler(x.target_mode);
    }
  } catch (SimulationTrap &e) {
    if (trace)
      std::cout << "take trap " << e.reason << ", mtval=" << e.mtval
                << std::endl;
    auto target_mode = prepare_trap(e);
    switch_to_trap_handler(target_mode);

    if (!csrs.mcountinhibit.IR)
      --csrs.instret.reg;
  }

  // NOTE: writes to zero register are supposedly allowed but must be ignored
  // (reset it after every instruction, instead of checking *rd != zero*
  // before every register write)
  regs.regs[regs.zero] = 0;

  performance_and_sync_update();
}

void ISS::run() {
  // run a single step until either a breakpoint is hit or the execution
  // terminates
  do {
    run_step();
  } while (status == CoreExecStatus::Runnable);
}

void ISS::show() {
  std::cout << "=[ core : " << csrs.mhartid.reg
            << " ]===========================" << std::endl;
  regs.show();
  std::cout << "pc = " << std::hex << pc << std::endl;
  std::cout << "num-instr = " << std::dec << csrs.instret.reg << std::endl;
}
