#include "to_string.h"

#include <iomanip>
#include <sstream>

std::string instr_to_string(Instruction instr)
{
	return to_string(instr);
}

std::string to_string(Compressed::Opcode c_op, Instruction& instr) {
    std::ostringstream ss;
    auto arch = RV32;

    switch(c_op) {
    // quadrant zero
    case Compressed::C_Illegal:
        ss << "c.illegal";
        break;

    case Compressed::C_Reserved:
        ss << "c.reserved";
        break;

    case Compressed::C_ADDI4SPN: {
        auto n = C_ADDI4SPN_NZUIMM(instr.data());
        if(n == 0)
            ss << "c.illegal (c.addi4spn with nzuimm==0)";
        else
            ss << "c.addi4spn"
               << " x" << instr.c_rs2_small() << ", " << n;
    } break;

    case Compressed::C_LW:
        ss << "c.lw"
           << " x" << instr.c_rs2_small() << ", x" << instr.c_rd_small() << ", " << C_LW_UIMM(instr.data());
        break;

    case Compressed::C_SW:
        ss << "c.sw"
           << " x" << instr.c_rd_small() << ", x" << instr.c_rs2_small() << ", " << C_SW_UIMM(instr.data());
        break;

    // quadrant one
    case Compressed::C_NOP:
        ss << "c.nop";
        break;

    case Compressed::C_ADDI:
        ss << "c.addi"
           << " x" << instr.c_rd() << ", " << instr.c_imm();
        break;

    case Compressed::C_JAL:
        ss << "c.jal"
           << " " << C_JAL_IMM(instr.data());
        break;

    case Compressed::C_LI:
        ss << "c.li"
           << " x" << instr.c_rd() << ", " << instr.c_imm();
        break;

    case Compressed::C_ADDI16SP: {
        auto n = C_ADDI16SP_NZIMM(instr.data());
        if(n == 0)
            ss << "c.illegal (c.addi16sp with nzimm==0)";
        else
            ss << "c.addi16sp"
               << " " << n;
    } break;

    case Compressed::C_LUI: {
        auto n = C_LUI_NZIMM(instr.data());
        if(n == 0)
            ss << "c.illegal (c.lui with nzimm==0)";
        else
            ss << "c.lui"
               << " x" << instr.c_rd() << ", " << n;
    } break;

    case Compressed::C_SRLI: {
        auto n = instr.c_uimm();
        if(arch == RV32 && n > 31)
            ss << "c.illegal (c.srli with uimm>31)";
        else
            ss << "c.srli"
               << " x" << instr.c_rd_small() << ", " << n;
    } break;

    case Compressed::C_SRAI: {
        auto n = instr.c_uimm();
        if(arch == RV32 && n > 31)
            ss << "c.illegal (c.srai with uimm>31)";
        else
            ss << "c.srai"
               << " x" << instr.c_rd_small() << ", " << n;
    } break;

    case Compressed::C_ANDI:
        ss << "c.andi"
           << " x" << instr.c_rd_small() << ", " << instr.c_imm();
        break;

    case Compressed::C_SUB:
        ss << "c.sub"
           << " x" << instr.c_rd_small() << ", x" << instr.c_rs2_small();
        break;

    case Compressed::C_XOR:
        ss << "c.xor"
           << " x" << instr.c_rd_small() << ", x" << instr.c_rs2_small();
        break;

    case Compressed::C_OR:
        ss << "c.or"
           << " x" << instr.c_rd_small() << ", x" << instr.c_rs2_small();
        break;

    case Compressed::C_AND:
        ss << "c.and"
           << " x" << instr.c_rd_small() << ", x" << instr.c_rs2_small();
        break;

    case Compressed::C_J:
        ss << "c.j"
           << " " << C_J_IMM(instr.data());
        break;

    case Compressed::C_BEQZ:
        ss << "c.beqz"
           << " x" << instr.c_rd_small() << ", " << C_BRANCH_IMM(instr.data());
        break;

    case Compressed::C_BNEZ:
        ss << "c.bnez"
           << " x" << instr.c_rd_small() << ", " << C_BRANCH_IMM(instr.data());
        break;

    // quadrant two
    case Compressed::C_SLLI: {
        auto n = instr.c_uimm();
        if(arch == RV32 && n > 31)
            ss << "c.illegal (c.slli with uimm>31)";
        else
            ss << "c.slli"
               << " x" << instr.c_rd() << ", " << n;
    } break;

    case Compressed::C_LWSP: {
        if(instr.c_rd() == 0)
            ss << "c.illegal (c.lwsp with rd==0)";
        else
            ss << "c.lwsp"
               << " x" << instr.c_rd() << ", " << C_LWSP_UIMM(instr.data());
    } break;

    case Compressed::C_JR: {
        if(instr.c_rd() == 0)
            ss << "c.illegal (c.jr with rs1==0)";
        else
            ss << "c.jr"
               << " x" << instr.c_rd();
    } break;

    case Compressed::C_MV:
        ss << "c.mv"
           << " x" << instr.c_rd() << ", x" << instr.c_rs2();
        break;

    case Compressed::C_EBREAK:
        ss << "c.ebreak";
        break;

    case Compressed::C_JALR:
        ss << "c.jalr"
           << " x" << instr.c_rd();
        break;

    case Compressed::C_ADD:
        ss << "c.add"
           << " x" << instr.c_rd() << ", x" << instr.c_rs2();
        break;

    case Compressed::C_SWSP:
        ss << "c.swsp"
           << " x" << instr.c_rs2() << ", " << C_SWSP_UIMM(instr.data());
        break;

    default:
        ss << Compressed::mappingStr[c_op] << " [...]";
        // throw std::runtime_error("unknown opcode '" + std::to_string(op) + "'");
    }

    return ss.str();
}

std::string to_string(Opcode::Mapping map)
{

	Instruction instr(map);
	return to_string(instr);
}

std::string to_string(Instruction instr) {
    Opcode::Mapping op;
    if(instr.is_compressed()) {
        // op = instr.decode_and_expand_compressed(RV32);
        auto c_op = instr.decode_compressed(RV32);
        return to_string(c_op, instr);
    } else {
        op = instr.decode_normal(RV32);
    }
    std::ostringstream ss;

    auto imm_op = [&](const char* name, bool is_signed = true) {
        ss << name << " x" << instr.rd() << ", x" << instr.rs1() << ", "
           << (is_signed ? instr.I_imm() : (uint32_t)instr.I_imm());
    };

    auto reg_op = [&](const char* name) {
        ss << name << " x" << instr.rd() << ", x" << instr.rs1() << ", x" << instr.rs2();
    };

    auto shift_op = [&](const char* name) {
        ss << name << " x" << instr.rd() << ", x" << instr.rs1() << ", " << instr.shamt();
    };

    auto store_op = [&](const char* name) {
        ss << name << " x" << instr.rs2() << ", x" << instr.rs1() << ", " << instr.S_imm();
    };

    auto load_op = imm_op;

    auto branch_op = [&](const char* name) {
        ss << name << " x" << instr.rs1() << ", x" << instr.rs2() << ", " << instr.B_imm();
    };

    auto csr_reg_op = [&](const char* name) {
        ss << name << " x" << instr.rd() << ", x" << instr.rs1() << ", " << csr_to_string(instr.csr());
    };

    auto csr_imm_op = [&](const char* name) {
        ss << name << " x" << instr.rd() << ", " << instr.zimm() << ", " << csr_to_string(instr.csr());
    };

    switch(op) {
    case Opcode::UNDEF:
        ss << "illegal";
        break;

    case Opcode::ADDI:
        imm_op("addi");
        break;

    case Opcode::SLTI:
        imm_op("slti");
        break;

    case Opcode::SLTIU:
        imm_op("sltiu", false);
        break;

    case Opcode::XORI:
        imm_op("xori", false);
        break;

    case Opcode::ORI:
        imm_op("ori", false);
        break;

    case Opcode::ANDI:
        imm_op("andi", false);
        break;

    case Opcode::ADD:
        reg_op("add");
        break;

    case Opcode::SUB:
        reg_op("sub");
        break;

    case Opcode::SLL:
        reg_op("sll");
        break;

    case Opcode::SLT:
        reg_op("slt");
        break;

    case Opcode::SLTU:
        reg_op("sltu");
        break;

    case Opcode::SRL:
        reg_op("srl");
        break;

    case Opcode::SRA:
        reg_op("sra");
        break;

    case Opcode::XOR:
        reg_op("xor");
        break;

    case Opcode::OR:
        reg_op("or");
        break;

    case Opcode::AND:
        reg_op("and");
        break;

    case Opcode::SLLI:
        shift_op("slli");
        break;

    case Opcode::SRLI:
        shift_op("srli");
        break;

    case Opcode::SRAI:
        shift_op("srai");
        break;

    case Opcode::LUI:
        ss << "lui x" << instr.rd() << ", " << instr.U_imm();
        break;

    case Opcode::AUIPC:
        ss << "auipc x" << instr.rd() << ", " << instr.U_imm();
        break;

    case Opcode::JAL:
        ss << "jal x" << instr.rd() << ", " << instr.J_imm();
        break;

    case Opcode::JALR:
        ss << "jalr x" << instr.rd() << ", x" << instr.rs1() << ", " << instr.I_imm();
        break;

    case Opcode::SB:
        store_op("sb");
        break;

    case Opcode::SH:
        store_op("sh");
        break;

    case Opcode::SW:
        store_op("sw");
        break;

    case Opcode::LB:
        load_op("lb");
        break;

    case Opcode::LH:
        load_op("lh");
        break;

    case Opcode::LW:
        load_op("lw");
        break;

    case Opcode::LBU:
        load_op("lbu");
        break;

    case Opcode::LHU:
        load_op("lhu");
        break;

    case Opcode::BEQ:
        branch_op("beq");
        break;

    case Opcode::BNE:
        branch_op("bne");
        break;

    case Opcode::BLT:
        branch_op("blt");
        break;

    case Opcode::BGE:
        branch_op("bge");
        break;

    case Opcode::BLTU:
        branch_op("bltu");
        break;

    case Opcode::BGEU:
        branch_op("bgeu");
        break;

    case Opcode::FENCE:
        ss << "fence";
        break;

    case Opcode::FENCE_I:
        ss << "fence_i";
        break;

    case Opcode::ECALL:
        ss << "ecall";
        break;

    case Opcode::EBREAK:
        ss << "ebreak";
        break;

    case Opcode::CSRRW:
        csr_reg_op("csrrw");
        break;

    case Opcode::CSRRS:
        csr_reg_op("csrrs");
        break;

    case Opcode::CSRRC:
        csr_reg_op("csrrc");
        break;

    case Opcode::CSRRWI:
        csr_imm_op("csrrwi");
        break;

    case Opcode::CSRRSI:
        csr_imm_op("csrrsi");
        break;

    case Opcode::CSRRCI:
        csr_imm_op("csrrci");
        break;

    case Opcode::MUL:
        reg_op("mul");
        break;

    case Opcode::MULH:
        reg_op("mulh");
        break;

    case Opcode::MULHU:
        reg_op("mulhu");
        break;

    case Opcode::MULHSU:
        reg_op("mulhsu");
        break;

    case Opcode::DIV:
        reg_op("div");
        break;

    case Opcode::DIVU:
        reg_op("divu");
        break;

    case Opcode::REM:
        reg_op("rem");
        break;

    case Opcode::REMU:
        reg_op("remu");
        break;

    case Opcode::WFI:
        ss << "wfi";
        break;

    case Opcode::MRET:
        ss << "mret";
        break;

    case Opcode::URET:
        ss << "uret";
        break;

    default:
        ss << Opcode::mappingStr[op] << " [...]";
        // throw std::runtime_error("unknown opcode '" + std::to_string(op) + "'");
    }

    return ss.str();
}

// see: https://stackoverflow.com/questions/5100718/integer-to-hex-string-in-c
template <typename T> static std::string int_to_hex(T i) {
    std::stringstream stream;
    stream << "0x" << std::hex << i;
    return stream.str();
}

std::string csr_to_string(unsigned csr) {
    auto it = csr_to_string_mapping.find(csr);
    if(it == csr_to_string_mapping.end()) {
        return int_to_hex(csr);
    }
    return it->second;
}
