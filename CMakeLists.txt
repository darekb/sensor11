CMAKE_MINIMUM_REQUIRED(VERSION 2.8.4)

PROJECT(cmake-avr)

#SET(MCU_SPEED "16000000")
SET(AVR_MCU "atmega328p")
set(AVR_L_FUSE 0x62)
set(AVR_H_FUSE 0xD9)
# SET(AVR_MCU "atmega168")
# set(AVR_L_FUSE 0xFF)
# set(AVR_H_FUSE 0xDD)
#
set(AVR_UPLOADTOOL avrdude)
SET(F_CPU "16000000UL")
set(AVR_PROGRAMMER arduino)
set(AVR_UPLOADTOOL_PORT /dev/ttyUSB0)
#duomleanove
#set(AVR_UPLOADTOOL_OPTIONS -b19200)
#nano
set(AVR_UPLOADTOOL_OPTIONS -b57600)
#uno
#set(AVR_UPLOADTOOL_OPTIONS -b115200)

#usbasp
#SET(F_CPU "8000000UL")
#set(AVR_PROGRAMMER usbasp)
#set(AVR_UPLOADTOOL_PORT usb)


set(CMAKE_BUILD_TYPE Release)

SET(SOURCE_EXE
        main.c
        slUart.c
        )

SET(SOURCE_LIB

        )

include_directories(../common)

##########################################################################
# compiler options for all build types
##########################################################################
#add_definitions("-DF_CPU=${MCU_SPEED}")
add_definitions("-DF_CPU=${F_CPU}")
#add_definitions("-mcall-prologues")
add_definitions("-fpack-struct")
add_definitions("-fshort-enums")
add_definitions("-Wall")
add_definitions("-Werror")
#add_definitions("-Wstrict-prototypes")
add_definitions("-pedantic")
add_definitions("-pedantic-errors")
add_definitions("-funsigned-char")
add_definitions("-funsigned-bitfields")
add_definitions("-ffunction-sections")
add_definitions("-c")
add_definitions("-Os")
#add_definitions("-S")
add_definitions("-std=gnu99")
add_definitions("-mmcu=${AVR_MCU}")

##########################################################################

include(avr-gcc.cmake)