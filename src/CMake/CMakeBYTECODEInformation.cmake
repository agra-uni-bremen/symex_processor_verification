# This file sets the basic flags for the BYTECODE compiler

set(CMAKE_BYTECODE_COMPILER
	"clang++"
)

set(CMAKE_BYTECODE_SOURCE_FILE_EXTENSIONS cpp)
set(CMAKE_BYTECODE_OUTPUT_EXTENSION .bc)
if(CMAKE_BYTECODE_COMPILER_TARGET)
    message("Setting target triplet to ${CMAKE_BYTECODE_COMPILER_TARGET}")
    set(SET_TARGET_TRIPLET --target=${CMAKE_BYTECODE_COMPILER_TARGET})
endif()
set(CMAKE_BYTECODE_FLAGS
	"-DUSE_KLEE ${SET_TARGET_TRIPLET} -stdlib=libc++ -std=c++14 -emit-llvm -flto -c -Xclang -disable-llvm-passes -D__NO_STRING_INLINES  -D_FORTIFY_SOURCE=0 -U__OPTIMIZE__" # -I/tmp/klee-uclibc-90/include/"
)
set(CMAKE_BYTECODE_FLAGS_DEBUG "-g3")
set(CMAKE_BYTECODE_FLAGS_RELEASE "-O3")
set(CMAKE_INCLUDE_FLAG_BYTECODE "-I ")
set(CMAKE_BYTECODE_LINK_FLAGS
	#"-fuse-ld=lld"
	#"-only-needed"
)

#-g -O1 

SET(CMAKE_BYTECODE_LINK_EXECUTABLE
	"llvm-link <CMAKE_BYTECODE_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> <LINK_LIBRARIES> -o <TARGET>"
)
set(CMAKE_BYTECODE_ARCHIVE_CREATE "<CMAKE_AR> qc <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_BYTECODE_ARCHIVE_APPEND "<CMAKE_AR> q <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_BYTECODE_ARCHIVE_FINISH "<CMAKE_RANLIB> <TARGET>")

SET(CMAKE_BYTECODE_CREATE_STATIC_LIBRARY
	"llvm-link <CMAKE_BYTECODE_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> <LINK_LIBRARIES> -o <TARGET>"
)

#if(NOT CMAKE_BYTECODE_COMPILE_OBJECT)
set(CMAKE_BYTECODE_COMPILE_OBJECT "<CMAKE_BYTECODE_COMPILER> <DEFINES> <INCLUDES> <FLAGS> <SOURCE> -o <OBJECT>")
#endif()
set(CMAKE_BYTECODE_INFORMATION_LOADED 1)