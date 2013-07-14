uses optiboot boot loader
external 12MHz clock from FTDI chip

program via avrdude:
$avrdude -p atmega328p -c arduino -b 57600 -P /dev/ttyUSB.A8WP4PO3