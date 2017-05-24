
# STM32F1 MIDI Serial Bridge
This code is a firmware for the STM32F1 that provides a MIDI-USB -> MIDI-Serial Converter and a MIDI-Serial -> MIDI-USB Converter.
It can be used with the Arduino MIDI library.


## Usage
The pins for the serial communication are A9 (Serial1 TX) and A10 (Serial RX).	[USART1]


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

or Build the bin firmware 
`make bin` -> *.bin


## Flash
Flash by st-utils


