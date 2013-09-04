nRF24L01+ to USB serial dongle
==============================

uses optiboot boot loader
external 12MHz clock from FTDI chip

NOTE: `makeall` boot loader script set fuses to EXTERNAL CLOCK.
This may brick your AVR if no actual external clock source!

program via avrdude (with applied udev from 60-FT232.rules):
$avrdude -p atmega328p -c arduino -b 57600 -P /dev/ttyUSB.NRF