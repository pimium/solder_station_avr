##################################################################################
# "THE ANY BEVERAGE-WARE LICENSE" (Revision 42 - based on beer-ware
# license):
# <dev@layer128.net> wrote this file. As long as you retain this notice
# you can do whatever you want with this stuff. If we meet some day, and
# you think this stuff is worth it, you can buy me a be(ve)er(age) in
# return. (I don't like beer much.)
#
# Matthias Kleemann
##################################################################################

##################################################################################
# Sample CMakeLists.txt for a simple AVR project based on the toolchain
##################################################################################

cmake_minimum_required(VERSION 2.8)

### TOOLCHAIN SETUP AREA #################################################
# Set any variables used in the toolchain prior project() call. In that
# case they are already set and used.
##########################################################################

##################################################################################
# tools to be used for programming the AVR
##################################################################################
set(AVR_UPLOADTOOL avrdude)
set(AVR_PROGRAMMER usbasp)
set(AVR_UPLOADTOOL_PORT usb)
set(WITH_MCU ON)
# AVR and fuses needs to be set
#set(AVR_MCU attiny2313)
set(AVR_MCU atmega168p)
#set(AVR_MCU attiny85)
set(AVR_H_FUSE 0xdf)
set(AVR_L_FUSE 0x64)
set(AVR_E_FUSE 0xFF)

### END TOOLCHAIN SETUP AREA #############################################
# -DCMAKE_TOOLCHAIN_FILE=./generic-gcc-avr.cmake
#include(./generic-gcc-avr.cmake)
# -CMAKE_RUNTIME_OUTPUT_DIRECTORY output dir
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY cmake-build-default)
#set(CMAKE_BINARY_DIR cmake-build-default)
##########################################################################
# name your project
##########################################################################
project(oled)
enable_language(C ASM)

##################################################################################
# status messages
##################################################################################
message(STATUS "Current uploadtool is: ${AVR_UPLOADTOOL}")
message(STATUS "Current programmer is: ${AVR_PROGRAMMER}")
message(STATUS "Current upload port is: ${AVR_UPLOADTOOL_PORT}")
message(STATUS "Current uploadtool options are: ${AVR_UPLOADTOOL_OPTIONS}")
message(STATUS "Current MCU is set to: ${AVR_MCU}")
message(STATUS "Current H_FUSE is set to: ${AVR_H_FUSE}")
message(STATUS "Current L_FUSE is set to: ${AVR_L_FUSE}")

##################################################################################
# set build type, if not already set at cmake command line
##################################################################################
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif (NOT CMAKE_BUILD_TYPE)

##################################################################################
# needs to be defined for AVR toolchain
##################################################################################
set(MCU_SPEED "1000000UL")

##################################################################################
# some cmake cross-compile necessities
##################################################################################
if (DEFINED ENV{AVR_FIND_ROOT_PATH})
    set(CMAKE_FIND_ROOT_PATH $ENV{AVR_FIND_ROOT_PATH})
else (DEFINED ENV{AVR_FIND_ROOT_PATH})
    if (EXISTS "/opt/local/avr")
        set(CMAKE_FIND_ROOT_PATH "/opt/local/avr")
    elseif (EXISTS "/usr/avr")
        set(CMAKE_FIND_ROOT_PATH "/usr/avr")
    elseif (EXISTS "/usr/lib/avr")
        set(CMAKE_FIND_ROOT_PATH "/usr/lib/avr")
    elseif (EXISTS "/usr/local/CrossPack-AVR")
        set(CMAKE_FIND_ROOT_PATH "/usr/local/CrossPack-AVR")
    else (EXISTS "/opt/local/avr")
        message(FATAL_ERROR "Please set AVR_FIND_ROOT_PATH in your environment.")
    endif (EXISTS "/opt/local/avr")
endif (DEFINED ENV{AVR_FIND_ROOT_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# not added automatically, since CMAKE_SYSTEM_NAME is "generic"
set(CMAKE_SYSTEM_INCLUDE_PATH "${CMAKE_FIND_ROOT_PATH}/include")
set(CMAKE_SYSTEM_LIBRARY_PATH "${CMAKE_FIND_ROOT_PATH}/lib")

##################################################################################
# status messages for generating
##################################################################################
message(STATUS "Set CMAKE_FIND_ROOT_PATH to ${CMAKE_FIND_ROOT_PATH}")
message(STATUS "Set CMAKE_SYSTEM_INCLUDE_PATH to ${CMAKE_SYSTEM_INCLUDE_PATH}")
message(STATUS "Set CMAKE_SYSTEM_LIBRARY_PATH to ${CMAKE_SYSTEM_LIBRARY_PATH}")

##################################################################################
# set compiler options for build types
##################################################################################
if (CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS_RELEASE "-Os")
    set(CMAKE_CXX_FLAGS_RELEASE "-Os")
    set(CMAKE_ASM_FLAGS_RELEASE "-DF_CPU=${MCU_SPEED} -x assembler-with-cpp -Wa,-gdwarf2")
endif (CMAKE_BUILD_TYPE MATCHES Release)

if (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-Os -save-temps -g -gdwarf-3 -gstrict-dwarf")
    set(CMAKE_ASM_FLAGS_RELWITHDEBINFO "-DF_CPU=${MCU_SPEED} -x assembler-with-cpp -Wa,-gdwarf2")
endif (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf")
    set(CMAKE_ASM_FLAGS_DEBUG "-DF_CPU=${MCU_SPEED} -x assembler-with-cpp -Wa,-gdwarf2")
    SET(CMAKE_ASM_FLAGS_DEBUG "-O0 -save-temps -g -gdwarf-3 -gstrict-dwarf -x assembler-with-cpp")
endif (CMAKE_BUILD_TYPE MATCHES Debug)

##################################################################################
# compiler options for all build types
##################################################################################
add_definitions("-DF_CPU=${MCU_SPEED}")
add_definitions("-fpack-struct")
add_definitions("-fshort-enums")
add_definitions("-Wall")
#add_definitions("-Werror")
# http://gcc.gnu.org/onlinedocs/gcc-4.8.2/gcc/Alternate-Keywords.html#Alternate-Keywords
# [...]-pedantic and other options cause warnings for many GNU C extensions. You can prevent such warnings within
# one expression by writing __extension__ before the expression. __extension__ has no effect aside from this.[...]
add_definitions("-pedantic")
add_definitions("-pedantic-errors")
add_definitions("-funsigned-char")
add_definitions("-funsigned-bitfields")
add_definitions("-ffunction-sections")
add_definitions("-c")
add_definitions("-std=gnu99")

##########################################################################
# include search paths
##########################################################################
include_directories(
        .
#        ssd1306xled
        hv5812
#        ssd1306xled/print_3x7
#        ssd1306xled/print_5x7
#        ssd1306xled/print_6x14
#        ssd1306xled/print_10x14
#        num2str
)
#include_directories(. ssd1306xled i2c_bitbang num2str)

#add_subdirectory(ssd1306xled)
add_subdirectory(hv5812)
#add_subdirectory(num2str)


##################################################################################
# add AVR executable
##################################################################################
add_avr_executable(
        ${PROJECT_NAME}
        main.c
        cpufreq.h
)


##################################################################################
# add AVR library
##################################################################################
#add_avr_library(
#   tm1637lib
#        tm1637lib/tm1637.c tm1637lib/tm1637.h tm1637lib/tm1637font.h
#)

##################################################################################
# link library to executable
# NOTE: It needs to be the elf target.
##################################################################################
# target_link_libraries(<executable-name>-${AVR_MCU}.elf <library-name>-${AVR_MCU})
# OR easier
#target_link_libraries(<executable-name> <library-name> <some-external-libary>)

target_link_libraries(
        ${PROJECT_NAME}-${AVR_MCU}.elf
#        ssd1306xled-${AVR_MCU}
        hv5812-${AVR_MCU}
#        num2str-${AVR_MCU}
)
# target_link_libraries(${PROJECT_NAME}-${AVR_MCU}.elf)

