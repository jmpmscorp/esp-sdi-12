# ESP-IDF SDI-12

SDI-12 bus implemntation using ESP-IDF framework. This component use RMT to code and decode SDI-12 frames.

Compatible with 4.1

## TESTED GPIOs

I tested every input/output GPIOs (0-32) in a esp32-devkitc-v4 board. I have found problems with 0, 14 and 15. Why? Need help.


## TODO
Add 4.0 and 4.2(master branch) IDF compatibility
Implement CRC commands.
Add aHA! command (High Volume)
Test aCx!, aRx!, aVx! and aHA! commands. I haven't any compatible sensor.
