cmake_minimum_required(VERSION 3.10)
project(deye ASM C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O2")
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")

#---------------------------------------------------------------------------------------
# Application
#---------------------------------------------------------------------------------------
add_executable(deye
   src/crc.c
   src/main.cpp
)

#includes
target_include_directories(deye PRIVATE
   dep
   src
)

