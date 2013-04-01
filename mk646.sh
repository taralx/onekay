#!/bin/bash
PATH=$HOME/gcc-avr/bin:$HOME/binutils-avr/bin:$PATH
#PATH=$HOME/binutils-avr/bin:$PATH
MCU=at90usb646
CPPFLAGS=${CPPFLAGS-}
OPT=${OPT--Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-move-loop-invariants -fno-inline-small-functions}
LDFLAGS=${LDFLAGS--g}
avr-cpp $CPPFLAGS asm.S | avr-as -mmcu=$MCU -ahls=asm.lst -o asm.o
avr-gcc -fverbose-asm $CPPFLAGS -mint8 -mtiny-stack -mno-interrupts -mshort-calls -mmcu=$MCU $OPT $CFLAGS -S bootloader.cc
avr-gcc -g $CPPFLAGS -mint8 -mtiny-stack -mno-interrupts -mshort-calls -mmcu=$MCU $OPT $CFLAGS -c bootloader.cc
avr-ld --relax --gc-sections -Map bootloader.map -z defs -z noexecstack --section-start=.text=0xfc00 --pmem-wrap-around=64k -Tbss 0x00800100 -m avr5 $LDFLAGS -o bootloader.elf {bootloader,asm}.o
avr-objcopy -j .text -O ihex bootloader.elf bootloader.hex
avr-size bootloader.hex
