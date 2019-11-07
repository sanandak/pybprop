# pybprop - flash a parallax inc., propeller (v.1 ) from a pyboard

For projects where a Parallax Inc. Propeller (http://www.parallax.com) and a pyboard are used together, flash an image (binary or eeprom) to the propeller from the pyboard.

# Setup
1. First, create a binary or eeprom image
  - $(PROPIDE)/openspin prog.spin [-e]
  - -e: create a prog.eeprom image
  - otherwise: create a prog.binary image
2. copy to the pyboard (either copy to the mounted pyboard volume or use rshell)

3. run the loader (either through `screen` or `rshell`)

# Usage
>> loader.py [filename] [-t]

With no arguments, query the prop for version number and exit
  - if filename ends with .binary, load to RAM and run
  - if filename ends with .eeprom, flash the eeprom and then run

-t: after loading, echo serial strings from the props to the terminal

# Notes
The serial port and the prop reset port are hard coded at the top of the file `loader.py`
You must connect 4 lines: UART TX, UART RX, RESET and GND



