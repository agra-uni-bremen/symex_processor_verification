cmake_minimum_required(VERSION 3.27.7)

project(Weltherrschaft NONE)

set(CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=address -fno-omit-frame-pointer -g -O1")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake")

set(UCLIBC_TARGET_TRIPLET x86_64-pc-linux-gnu)	#uclibc is built in this target triple

message("Build is set to: ${BUILD}")
if(BUILD STREQUAL "bytecode")
    set(CMAKE_BUILD_TYPE DEBUG)
    MESSAGE("Building LLVM BYTECODE for Klee")
    set(CMAKE_BYTECODE_COMPILER_TARGET ${UCLIBC_TARGET_TRIPLET})
    set(CMAKE_C_COMPILER_TARGET ${UCLIBC_TARGET_TRIPLET})
    set(CMAKE_CXX_COMPILER_TARGET ${UCLIBC_TARGET_TRIPLET})
    add_compile_definitions(ENABLE_KLEE)
    enable_language(BYTECODE)
elseif(BUILD STREQUAL "runTest")
    enable_language(CXX)
    MESSAGE("Building native C/C++")
    set(CMAKE_BUILD_TYPE DEBUG)
    set(CMAKE_CXX_COMPILER "clang++")
    set(CMAKE_C_COMPILER "clang")
    set(SystemC_LIBRARIES ${SystemC_CXX_LIBRARIES})
    SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl")	# for klee_int() and konsorten
elseif(BUILD STREQUAL "clang-tidy")
    enable_language(CXX)
    MESSAGE("Building with clang-tidy ")
    set(CMAKE_BUILD_TYPE DEBUG)
    set(CMAKE_CXX_COMPILER "clang++")
    set(CMAKE_C_COMPILER "clang")
    set(SystemC_LIBRARIES ${SystemC_CXX_LIBRARIES})
    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
    set(CMAKE_CXX_CLANG_TIDY "clang-tidy")#; -header-filter=.; -checks=*; warnings-as-errors=*")
    SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl")	# for klee_int() and konsorten
else() #native
    enable_language(CXX)
    MESSAGE("Building native C/C++")
    set(CMAKE_CXX_COMPILER "clang++")
    set(CMAKE_C_COMPILER "clang")
    set(SystemC_LIBRARIES ${SystemC_CXX_LIBRARIES})
    SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl")	# for klee_int() and konsorten
endif()

include(obj_dir/Vconfig.cmake)

SET(ISS_SRC iss.cpp instr.cpp to_string.cpp)

add_library(kleeRunTest SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(kleeRunTest PROPERTIES
  IMPORTED_LOCATION "/home/klee/klee_build/lib/libkleeRuntest.so"
  INTERFACE_INCLUDE_DIRECTORIES "/home/klee/klee_build/include/klee"
)

add_executable(main
       ${Vconfig_GLOBAL}
       ${Vconfig_CLASSES_SLOW} 
       ${Vconfig_CLASSES_FAST} 
       ${Vconfig_SUPPORT_SLOW} 
       ${Vconfig_USER_CLASSES}
       ${ISS_SRC}
)

target_include_directories(main PUBLIC "${VERILATOR_ROOT}/include" "${VERILATOR_ROOT}/include/vltstd")
if(BUILD STREQUAL "runTest")
add_definitions(-DTRACE)
target_link_libraries(main PUBLIC kleeRunTest)
endif()


