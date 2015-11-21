volvo-kenwood
=============

Volvo steering wheel buttons interface for Kenwood head unit

Hardware used:
- Seeeduino (clone of Arduino Duemilanove)
- Seeeduino CANBUS shield
- 5v to 12v transistor amplifiers for controls: 5v is what AVR can drive, 12v is what Kenwood
  needs for dimmer control, park switch and reverse
- MAX823LEUT circuit supervisor: power line quality monitor, reset signal generator, watchdog

Controls and MAX823 are optional. Without hardware watchdog my Arduino board hangs quite often on
start, hardware watchdog is highly recommended.

Arduino's bootloader doesn't make it fast enough before watchdog triggers (~1.6s), so if you want to
use MAX823 make sure you burn faster bootloader (third-party or original one with reduced retries
count) or avoid bootloader at all and upload control software through ISP programmer.
