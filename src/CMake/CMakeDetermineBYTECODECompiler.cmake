# Find the compiler
find_program(
    CMAKE_BYTECODE_COMPILER 
        NAMES "clang++" 
        HINTS "${CMAKE_SOURCE_DIR}"
        DOC "LLVM Bytecode compiler" 
)
mark_as_advanced(CMAKE_FOO_COMPILER)

set(CMAKE_BYTECODE_COMPILER_ENV_VAR "clang++")

# Configure variables set in this file for fast reload later on
configure_file(${CMAKE_CURRENT_LIST_DIR}/CMakeBYTECODECompiler.cmake.in
               ${CMAKE_PLATFORM_INFO_DIR}/CMakeBYTECODECompiler.cmake)