# pybprop - flash a parallax inc., propeller (v.1 ) from a pyboard

For projects where a Parallax Inc. Propeller (http://www.parallax.com) and a pyboard are used together, flash an image (binary or eeprom) to the propeller from the pyboard.

# Setup
1. First, create a binary or eeprom image
    - $(PROPIDE)/openspin prog.spin [-e]
    - -e: create a prog.eeprom image
    - otherwise: create a prog.binary image
2. copy to the pyboard (either copy to the mounted pyboard volume or use rshell)

3. run the repl (e.g., `screen /dev/xxx 115200`)

# Usage
```python
>> import loader
>> loader._action_get_version()
>> loader._action_upload(prog.binary, progress=print_status, terminal=True)
```

  - if filename ends with .binary, load to RAM and run
  - if filename ends with .eeprom, flash the eeprom and then run
  - if `progress=print_status`, print debugging info. if `progress=do_nothing`, do not print debug messages
  - if `terminal=True` echo messages from the prop to the terminal.

# Hardware connections
The serial port and the prop reset port are hard coded at the top of the file `loader.py`

these are the defaults, but you can set them before calling `loader._xxx`
```
loader.UART_NUM = 4
loader.RESET_PIN = 'X3'
```

# Notes
You must connect 4 lines: UART TX, UART RX, RESET and GND



