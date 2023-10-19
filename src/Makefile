klee-exit-on-assert: bytecode
	cp RiscV32Core.v_toplevel_regs_regFile.bin build/RiscV32Core.v_toplevel_regs_regFile.bin
	date '+%s'
	klee --optimize --posix-runtime --libc=uclibc --libcxx -exit-on-error-type=Assert ./build/main
	date '+%s'
	
bytecode: obj_dir/Vconfig.cmake
	rm -rf build
	mkdir build
	cd build && cmake ../ -DBUILD=bytecode && make
	cp RiscV32Core.v_toplevel_regs_regFile.bin build/RiscV32Core.v_toplevel_regs_regFile.bin
	
	
obj_dir/Vconfig.cmake: ../microrv32/microrv32/rtl/RiscV32Core.v
	verilator config.vlt -Wall -O3 --no-merge-const-pool -x-assign fast -x-initial fast --noassert  --cc --make cmake --compiler clang sim_main.cpp ../microrv32/microrv32/rtl/RiscV32Core.v
	
runTest: obj_dir/Vconfig.cmake
	rm -rf build
	mkdir build
	cd build && cmake ../ -DBUILD=runTest && make
	cp RiscV32Core.v_toplevel_regs_regFile.bin build/RiscV32Core.v_toplevel_regs_regFile.bin
	
codestyle:
	find . -type d \( -name .git -o -name dependencies \) -prune -o -name '*.h' -o -name '*.hpp' -o -name '*.cpp' -print | xargs clang-format -i -style=file

clean:
	rm -rf obj_dir
	rm -rf build
	
.PHONY: klee-exit-on-assert bytecode clean runTest codestyle
