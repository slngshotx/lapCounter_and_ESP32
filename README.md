## Race Coordinator Arduino and ESP32 sketch
This repository contains a modified version of the Race Coordinator Arduino sketch.

This version will work on an ESP32 (dev kit only tested) aswell as an Arduino R3/mega, although not all features have been tested i.e. RGB LED and fuel stutter. The code will detect which board is being used and **should** just work for both Arduino and ESP32 without any further code changes.

### Caveats
- Select the Mega board in Race Coordinator since that allows configuration of all pins.
- Do **not** configure pins 6,7,8,9,10 and 11 in the RC configuration screen, ensure they are set to UNUSED. On ESP32 these pins are reserved for flash memory use and will crash the board
- The sketch currently only works when connected with a USB cable, in future it may also work using Bluetooth on non LE mode. 
