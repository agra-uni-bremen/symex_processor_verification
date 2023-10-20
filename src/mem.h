#pragma once

#include <stdint.h>
#include <functional>
#include <map>
#include <cstdlib>
#include <ctime>

#include "instr_encoding.h"
#ifdef ENABLE_KLEE
#include "klee_conf.h"
#endif

#define STR(x) #x
#define MY_ASSERT(x) if (!(x)) { fprintf(stderr,"My custom assertion failed: (%s), function %s, file %s, line %d.\n", STR(x), __PRETTY_FUNCTION__, __FILE__, __LINE__); abort(); }


namespace rv32 {

struct InstructionMemoryXX  : public instr_memory_if
{

	InstructionMemoryXX() 
	{
	}

	std::map<uint32_t, uint32_t> instruction_map;
	
	uint32_t ADDI(unsigned rd, unsigned rs1, int I_imm) {
		return ((I_imm & 4095) << 20) | ((rs1 & 0x1f) << 15) | ((rd & 0x1f) << 7) | 0b0010011;
	}
	
	virtual uint32_t load_instr(uint64_t pc)
	{
		if(instruction_map.count(pc) == 0)
		{
			//uint32_t instruction = rand();
			uint32_t instruction;
#ifdef ENABLE_KLEE
			klee_make_symbolic(&instruction, sizeof(instruction), "instruction");
			
			//klee_assume((instruction & (LW_MASK)) != (LW_ENCODING)); 
#endif
			instruction_map[pc] = instruction;
		}

		return instruction_map[pc];
	}
};

struct MemoryProxy : public data_memory_if {
	
	unsigned int mem_size_;
	
	std::vector<uint8_t> mem = {0};
	
	MemoryProxy(unsigned int mem_size) : mem_size_(mem_size)
	{
		mem.resize(mem_size);
	}
	
	virtual int64_t load_double(uint64_t addr)
	{
		MY_ASSERT(addr + sizeof(int64_t) < mem.size());
		int64_t return_value = 0;
		memcpy(&return_value, &mem[addr], sizeof(int64_t));
		return return_value;
	}
	
	virtual int32_t load_word(uint64_t addr)
	{
		MY_ASSERT(addr + sizeof(int32_t) < mem.size());
		int32_t return_value = 0;
		memcpy(&return_value, &mem[addr], sizeof(int32_t));
		return return_value;
	}
	
	virtual int32_t load_half(uint64_t addr)
	{
		MY_ASSERT(addr + sizeof(int16_t) < mem.size());
		int32_t return_value = 0;
		memcpy(&return_value, &mem[addr], sizeof(int16_t));
		return return_value;
	}
	
	virtual int32_t load_byte(uint64_t addr)
	{
		MY_ASSERT(addr + sizeof(int8_t) < mem.size());
		int32_t return_value = 0;
		memcpy(&return_value, &mem[addr], sizeof(int8_t));
		return return_value;
	}
	
	virtual uint32_t load_uhalf(uint64_t addr)
	{
		MY_ASSERT(addr + sizeof(uint16_t) < mem.size());
		uint32_t return_value = 0;
		memcpy(&return_value, &mem[addr], sizeof(uint16_t));
		return return_value;
	}
	
	virtual uint32_t load_ubyte(uint64_t addr)
	{
		MY_ASSERT(addr + sizeof(uint8_t) < mem.size());
		uint32_t return_value = 0;
		memcpy(&return_value, &mem[addr], sizeof(uint8_t));
		return return_value;
	}
	
	virtual uint32_t load_strobe(uint32_t strobes, uint32_t addr)
	{
		uint32_t return_value = 0;
		switch(strobes)
		{
			case 0b0001:
				MY_ASSERT(addr + 3 < mem.size());
				memcpy(&return_value, &mem[addr+3], sizeof(uint8_t));
			break;
			case 0b0010:
				MY_ASSERT(addr + 2 < mem.size());
				memcpy(&return_value, &mem[addr+2], sizeof(uint8_t));
			break;
			case 0b0100:
				MY_ASSERT(addr + 1 < mem.size());
				memcpy(&return_value, &mem[addr+1], sizeof(uint8_t));
			break;
			case 0b1000:
				MY_ASSERT(addr < mem.size());
				memcpy(&return_value, &mem[addr], sizeof(uint8_t));
			break;
			case 0b0011:
				MY_ASSERT(addr + 2 + sizeof(uint16_t) < mem.size());
				memcpy(&return_value, &mem[addr+2], sizeof(uint16_t));
			break;
			case 0b1100:
				MY_ASSERT(addr + sizeof(uint16_t) < mem.size());
				memcpy(&return_value, &mem[addr], sizeof(uint16_t));
			break;
			case 0b1111:
				MY_ASSERT(addr + sizeof(uint32_t) < mem.size());
				memcpy(&return_value, &mem[addr], sizeof(int32_t));
			break;	
			default:
				MY_ASSERT(false && "Unknown Strobe");	
		}
		return return_value;
	}
	
	virtual void store_strobe(uint32_t strobes, uint32_t addr, uint32_t value)
	{
		switch(strobes)
		{
			case 0b0001:
				MY_ASSERT(addr + 3 + sizeof(uint8_t) < mem.size());
				memcpy(&mem[addr+3], &value , sizeof(uint8_t));
			break;
			case 0b0010:
				MY_ASSERT(addr + 2 + sizeof(uint8_t) < mem.size());
				memcpy(&mem[addr+2], &value, sizeof(uint8_t));
			break;
			case 0b0100:
				MY_ASSERT(addr + 1 + sizeof(uint8_t) < mem.size());
				memcpy(&mem[addr+1], &value, sizeof(uint8_t));
			break;
			case 0b1000:
				MY_ASSERT(addr + sizeof(uint8_t) < mem.size());
				memcpy(&mem[addr], &value, sizeof(uint8_t));
			break;
			case 0b0011:
				MY_ASSERT(addr + 2 + sizeof(uint16_t) < mem.size());
				memcpy(&mem[addr+2], &value, sizeof(uint16_t));
			break;
			case 0b1100:
				MY_ASSERT(addr + sizeof(uint16_t) < mem.size());
				memcpy(&mem[addr], &value, sizeof(uint16_t));
			break;
			case 0b1111:
				MY_ASSERT(addr + sizeof(uint32_t) < mem.size());
				memcpy(&mem[addr], &value, sizeof(int32_t));
			break;	
			default:
				//exit(-1);
				MY_ASSERT(false && "Unknown Strobe");	
		}
	}

	virtual void store_double(uint64_t addr, uint64_t value)
	{
		MY_ASSERT(addr + sizeof(uint64_t) < mem.size());
		memcpy(&mem[addr] , &value, sizeof(uint64_t));
	}
	
	virtual void store_word(uint64_t addr, uint32_t value)
	{
		MY_ASSERT(addr + sizeof(uint32_t) < mem.size());
		memcpy(&mem[addr] , &value, sizeof(uint32_t));
	}
	
	virtual void store_half(uint64_t addr, uint16_t value)
	{
		MY_ASSERT(addr + sizeof(uint16_t) < mem.size());
		memcpy(&mem[addr] , &value, sizeof(uint16_t));
	}
	
	virtual void store_byte(uint64_t addr, uint8_t value)
	{
		MY_ASSERT(addr + sizeof(uint8_t) < mem.size());
		memcpy(&mem[addr] , &value, sizeof(uint8_t));
	}
    
};

}  // namespace rv32
