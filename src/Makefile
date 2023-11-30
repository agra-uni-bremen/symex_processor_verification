klee-exit-on-assert: byte_code_main
	cp RiscV32Core.v_toplevel_regs_regFile.bin build/RiscV32Core.v_toplevel_regs_regFile.bin
	date '+%s'
	klee --optimize --posix-runtime --libc=uclibc --libcxx -exit-on-error-type=Assert ./byte_code_main
	date '+%s'
	
byte_code_main: bytecode
	cp ./build/main byte_code_main
	
bytecode: obj_dir/Vconfig.cmake
	rm -rf build
	mkdir build
	cd build && cmake ../ -DBUILD=bytecode && make
	cp RiscV32Core.v_toplevel_regs_regFile.bin build/RiscV32Core.v_toplevel_regs_regFile.bin

clang-tidy: obj_dir/Vconfig.cmake
	rm -rf build
	mkdir build
	cp ignore.clang-tidy obj_dir/.clang-tidy
	cd build && cmake ../ -DBUILD=clang-tidy && make
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
	
.PHONY: klee-exit-on-assert bytecode clean runTest codestyle clang-tidy
