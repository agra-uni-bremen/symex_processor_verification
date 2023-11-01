#include <iostream>
#include <nl_types.h>
#include <stdio.h>
#include <verilated.h>

#include "iss.h"
#include "mem.h"
#include "obj_dir/Vconfig.h"

struct CPUValues {
  bool IMem_fetchEnable;
  uint32_t IMem_address;

  bool DMem_readWrite;
  bool DMem_enable;
  uint32_t DMem_wrStrobe;
  uint32_t DMem_address;
  uint32_t io_memIF_DMem_writeData;

  bool io_halted;
  bool io_fetchSync;
  uint32_t io_dbgState;

  bool rvfi_trap;
  bool rvfi_intr;

  uint32_t rvfi_mode;
  uint32_t rvfi_ixl;
  uint32_t rvfi_rs1_addr;
  uint32_t rvfi_rs2_addr;
  uint32_t rvfi_rd_addr;
  uint32_t rvfi_mem_rmask;
  uint32_t rvfi_mem_wmask;

  bool rvfi_valid;
  bool rvfi_halt;

  uint32_t rvfi_insn;
  uint32_t rvfi_rs1_rdata;
  uint32_t rvfi_rs2_rdata;

  uint32_t rvfi_rd_wdata;
  uint32_t rvfi_pc_rdata;
  uint32_t rvfi_pc_wdata;
  uint32_t rvfi_mem_addr;
  uint32_t rvfi_mem_rdata;
  uint32_t rvfi_mem_wdata;
  uint64_t rvfi_order;
};

void getValues(CPUValues &values, Vconfig *top) {
  values.IMem_fetchEnable = top->io_memIF_IMem_fetchEnable;
  values.IMem_address = top->io_memIF_IMem_address;

  values.DMem_readWrite = top->io_memIF_DMem_readWrite;
  values.DMem_enable = top->io_memIF_DMem_enable;
  values.DMem_wrStrobe = top->io_memIF_DMem_wrStrobe;
  values.DMem_address = top->io_memIF_DMem_address;
  values.io_memIF_DMem_writeData = top->io_memIF_DMem_writeData;

  values.io_halted = top->io_halted;
  values.io_fetchSync = top->io_fetchSync;
  values.io_dbgState = top->io_dbgState;

  values.rvfi_trap = top->rvfi_trap;
  values.rvfi_intr = top->rvfi_intr;

  values.rvfi_mode = top->rvfi_mode;
  values.rvfi_ixl = top->rvfi_ixl;
  values.rvfi_rs1_addr = top->rvfi_rs1_addr;
  values.rvfi_rs2_addr = top->rvfi_rs2_addr;
  values.rvfi_rd_addr = top->rvfi_rd_addr;
  values.rvfi_mem_rmask = top->rvfi_mem_rmask;
  values.rvfi_mem_wmask = top->rvfi_mem_wmask;

  values.rvfi_valid = top->rvfi_valid;
  values.rvfi_halt = top->rvfi_halt;

  values.rvfi_insn = top->rvfi_insn;
  values.rvfi_rs1_rdata = top->rvfi_rs1_rdata;
  values.rvfi_rs2_rdata = top->rvfi_rs2_rdata;

  values.rvfi_rd_wdata = top->rvfi_rd_wdata;
  values.rvfi_pc_rdata = top->rvfi_pc_rdata;
  values.rvfi_pc_wdata = top->rvfi_pc_wdata;
  values.rvfi_mem_addr = top->rvfi_mem_addr;
  values.rvfi_mem_rdata = top->rvfi_mem_rdata;
  values.rvfi_mem_wdata = top->rvfi_mem_wdata;
  values.rvfi_order = top->rvfi_order;
}

int main(int argc, char **argv, char **env) {

  rv32::ISS iss;
#ifdef TRACE
  iss.trace = true;
#endif

  iss.csrs.misa.extensions = rv32::csr_misa::I;

  // unsigned int mem_size = 1024 * 1024 * 32;
  // unsigned int mem_size = 1024 * 32;
  unsigned int mem_size = 64;

  rv32::InstructionMemoryXX instrMem;

  rv32::MemoryProxy mem(mem_size);
  iss.instr_mem = &instrMem;
  iss.mem = &mem;

  rv32::MemoryProxy rtl_mem(mem_size);

  if (false && argc && argv && env) {
  }

  Vconfig *top = new Vconfig;

  vluint64_t main_time = 0;

  vluint64_t fetch_time = 0;

  vluint64_t load_time = 0;
  vluint64_t store_time = 0;
#ifdef TRACE  
  vluint64_t unknown_time = 0;
#endif

  vluint64_t default_dbus = 0;

  bool init = false;

  bool first_fetch = false;

  struct CPUValues values;

  uint32_t ready_count = 0;

  uint32_t valid_count = 0;

  uint32_t executed_instructions = 0;

  std::set<uint32_t> already_executed_pcs;

  while (!Verilated::gotFinish()) {
    main_time++;

    top->clk = !top->clk;

    getValues(values, top);

    if (first_fetch && values.rvfi_valid) {
      valid_count++;
    } else {
      valid_count = 0;
    }

    if (init && first_fetch && valid_count == 2) {
#ifdef TRACE
      std::cout << std::hex;
      std::cout << "===" << std::endl;
      std::cout << "rvfi_trap: " << values.rvfi_trap << std::endl;
      std::cout << "rvfi_intr: " << values.rvfi_intr << std::endl;
      std::cout << "rvfi_mode: " << values.rvfi_mode << std::endl;
      std::cout << "rvfi_ixl: " << values.rvfi_ixl << std::endl;
      std::cout << "rvfi_rs1_addr: " << values.rvfi_rs1_addr << std::endl;
      std::cout << "rvfi_rs2_addr: " << values.rvfi_rs2_addr << std::endl;
      std::cout << "rvfi_rd_addr: " << values.rvfi_rd_addr << std::endl;
      std::cout << "rvfi_mem_rmask: " << values.rvfi_mem_rmask << std::endl;
      std::cout << "rvfi_mem_wmask: " << values.rvfi_mem_wmask << std::endl;
      std::cout << "rvfi_valid: " << values.rvfi_valid << std::endl;
      std::cout << "rvfi_halt: " << values.rvfi_halt << std::endl;
      std::cout << "rvfi_insn: " << values.rvfi_insn << std::endl;
      std::cout << "rvfi_rs1_rdata: " << values.rvfi_rs1_rdata << std::endl;
      std::cout << "rvfi_rs2_rdata: " << values.rvfi_rs2_rdata << std::endl;
      std::cout << "rvfi_rd_wdata: " << values.rvfi_rd_wdata << std::endl;
      std::cout << "rvfi_pc_rdata: " << values.rvfi_pc_rdata << std::endl;
      std::cout << "rvfi_pc_wdata: " << values.rvfi_pc_wdata << std::endl;
      std::cout << "rvfi_mem_addr: " << values.rvfi_mem_addr << std::endl;
      std::cout << "rvfi_mem_rdata: " << values.rvfi_mem_rdata << std::endl;
      std::cout << "rvfi_mem_wdata: " << values.rvfi_mem_wdata << std::endl;
      std::cout << "rvfi_order: " << values.rvfi_order << std::endl;
#endif

      iss.run_step();

#ifdef TRACE
      std::cout << "iss.instr.data(): " << std::dec << iss.instr.data()
                << std::endl;
#endif
      assert(iss.instr.data() == values.rvfi_insn);
      assert(iss.last_pc == values.rvfi_pc_rdata);
#ifdef TRACE
      std::cout << "iss.pc:" << iss.pc << std::endl;
#endif
      assert(iss.pc == values.rvfi_pc_wdata);

      auto rd = values.rvfi_rd_addr;
      assert(static_cast<uint32_t>(iss.regs[rd]) == values.rvfi_rd_wdata);
      executed_instructions++;

      if (already_executed_pcs.count(iss.last_pc)) {
#ifdef TRACE
        std::cout << "LOOP was detected! Exit..." << std::endl;
#endif
        goto theEND;
      }
      already_executed_pcs.insert(iss.last_pc);
    }

    if (top->io_memIF_IMem_instructionReady) {
      ready_count++;
    }

    if (ready_count > 1) {
      top->io_memIF_IMem_instructionReady = false;
      ready_count = 0;
    }
    top->io_memIF_DMem_dataReady = false;

    if (!top->io_memIF_DMem_enable && top->io_memIF_DMem_wrStrobe == 15 &&
        !top->io_memIF_DMem_readWrite) {
      init = true;
      default_dbus++;
    }

    if (top->io_memIF_DMem_enable && top->io_memIF_DMem_wrStrobe &&
        top->io_memIF_DMem_readWrite) {

      rtl_mem.store_strobe(top->io_memIF_DMem_wrStrobe,
                           top->io_memIF_DMem_address,
                           top->io_memIF_DMem_writeData);
      top->io_memIF_DMem_dataReady = true;

      store_time++;
    }

    if (top->io_memIF_DMem_enable && top->io_memIF_DMem_wrStrobe &&
        !top->io_memIF_DMem_readWrite) {
      top->io_memIF_DMem_readData = rtl_mem.load_strobe(
          top->io_memIF_DMem_wrStrobe, top->io_memIF_DMem_address);
      top->io_memIF_DMem_dataReady = true;

      load_time++;
    }

    if (top->io_memIF_IMem_fetchEnable) {
      first_fetch = true;
      fetch_time++;

      uint32_t instruction = instrMem.load_instr(top->io_memIF_IMem_address);
      top->io_memIF_IMem_instruction = instruction; // write instruction
      top->io_memIF_IMem_instructionReady = true;
    }


    if (main_time > 1000 || executed_instructions >= 1) {
    theEND:
#ifdef TRACE
      std::cout << "eval: " << main_time << std::endl;
      std::cout << "fetch: " << fetch_time << std::endl;
      std::cout << "load: " << load_time << std::endl;
      std::cout << "store: " << store_time << std::endl;
      std::cout << "default_dbus: " << default_dbus << std::endl;
      std::cout << "unknown: " << unknown_time << std::endl;
#endif
      break;
    }

    top->eval();
  }

  top->final();
  delete top;
  exit(0);
}
