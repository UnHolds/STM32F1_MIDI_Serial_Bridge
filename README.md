
# STM32F1 MIDI Serial Bridge
This code is a firmware for the STM32F1 that provides a
MIDI-USB -> MIDI-Serial Converter and a MIDI-Serial -> MIDI-USB Converter.
It can be used with the Arduino MIDI library.


## Status
working!

## Usage
The pins for the serial communication are A9 (Serial TX) and A10 (Serial RX).
[USART1]

connect those pins to your MIDI device, but if you connect it to a genuine
midi device you must change the baudrate to 31250 (current 115200)


## Build

Clone : 
`git clone https://github.com/DarkSunHD/STM32F1_MIDI_Serial_Bridge.git`

Go into the folder: 
`cd STM32F1_MIDI_Serial_Bridge`

Clone submodules: 
`git submodule update --init --recursive`

Go into the "libopencm3" folder: 
`cd libopencm3/`

Build "libopencm3": 
`make`

Go in the "src" folder: 
`cd ../src/`

Build the firmware: 
`make` -> *.elf


## Current Makefile

### Flash 1

ST-util must be installed.

Usage:

plug in the STM32 and run `make`
the script will compile and flash it to the stm32


## For the Makefile.old

remove the Makefile and rename the Makefile.old to Makefile

or Build the bin firmware
`make bin` -> *.bin


### Flash 2
Flash with st-utils


Open terminl (terminal1) and go to your *.elf file then type:
`arm-none-eabi-gdb file.elf`

Now open a new terminal (terminal2) and run the st-utils `st-util`

go back to terminal1 and type: `tar extended-remote :4242`
(4242 is the port st-util opens)

now write the file to the stm32 (terminal1) `load`

done
