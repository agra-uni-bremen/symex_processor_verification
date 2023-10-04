#pragma once

#include <stdexcept>

#define LIKELY(x) __builtin_expect((x), 1)
#define UNLIKELY(x) __builtin_expect((x), 0)

#define UNUSED(x) (void)(x)

inline void ensure(bool cond) {
	if (UNLIKELY(!cond))
		throw std::runtime_error("runtime assertion failed");
}

inline void ensure(bool cond, const std::string &reason) {
	if (UNLIKELY(!cond))
		throw std::runtime_error(reason);
}

inline uint64_t rv64_align_address(uint64_t addr) {
	return addr - addr % 8;
}

inline uint32_t rv32_align_address(uint32_t addr) {
	return addr - addr % 4;
}
