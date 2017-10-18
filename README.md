#Status
Not working!


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
Flash with st-utils


Open terminl (terminal1) and go to your *.elf file then type: `arm-none-eabi-gdb file.elf`

Now open a new terminal (terminal2) and run the st-utils `st-util`

go back to terminal1 and type: `tar extended-remote :4242` (4242 is the port st-util opens)

now write the file to the stm32 (terminal1) `load`

done
