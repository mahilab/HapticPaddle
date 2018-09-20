cmake_minimum_required(VERSION 3.7)

#===============================================================================
# SETUP CROSS COMPILER (uncomment or remove as needed)
#===============================================================================

# NI X64 CROSS COMPILER

# # add compiler to path
# LIST(APPEND CMAKE_PROGRAM_PATH ${NI_X64_ROOT}/sysroots/i686-nilrtsdk-mingw32/usr/bin/x86_64-nilrt-linux)
# # set compiler (must be called before project/enable)
# set(CMAKE_C_COMPILER x86_64-nilrt-linux-gcc)
# set(CMAKE_CXX_COMPILER x86_64-nilrt-linux-g++)
# # set to cross compile
# set(CMAKE_SYSTEM_NAME Generic)
# set(CMAKE_CROSSCOMPILING 1)
# # setup sysroot (for NI RT cross compiler)
# set(CMAKE_SYSROOT ${NI_X64_ROOT}/sysroots/core2-64-nilrt-linux)

# NI ARM CROSS COMPILER

# # add compiler to path
# LIST(APPEND CMAKE_PROGRAM_PATH ${NI_ARM_ROOT}/sysroots/i686-nilrtsdk-mingw32/usr/bin/arm-nilrt-linux-gnueabi)
# # set compiler (must be called before project/enable)
# set(CMAKE_C_COMPILER arm-nilrt-linux-gnueabi-gcc)
# set(CMAKE_CXX_COMPILER arm-nilrt-linux-gnueabi-g++)
# # set to cross compile
# set(CMAKE_SYSTEM_NAME Generic)
# set(CMAKE_CROSSCOMPILING 1)
# # setup sysroot (for NI RT cross compiler)
# set(CMAKE_SYSROOT ${NI_ARM_ROOT}/sysroots/cortexa9-vfpv3-nilrt-linux-gnueabi)

#===============================================================================
# MY PROJECT
#===============================================================================

# enable C++11
enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 11)

# create project
project(HapticPaddle)

# set binary output location to conventional "bin" (optional, but recommended)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

# include directories
include_directories(
    "include"              # your include directory
    "/Git/MEL/include" # MEL's include directory
)

# indicate where MEL.lib is
link_directories("/Git/MEL/lib")

# remove or uncomment if linking MEL statically
# add_definitions(-DMEL_STATIC)

# find all source files
file(GLOB SRC_FILES
    "${CMAKE_SOURCE_DIR}/include/*.hpp"
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
)

# create application(s)
add_executable(haptic_paddle
               ${SRC_FILES})

# link MEL
target_link_libraries(haptic_paddle MEL)
