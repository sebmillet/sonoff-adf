Transcode
=========

Receives a code from a Sonoff telecommand and sends it to "Automatismes De
France" (adf) slater.

- Receive from an RF 433 Mhz receiver.
- Send with an RF 433 Mhz transmitter.

Schema:

- 'data' of RF433 receiver needs be plugged on PIN 'D2' of Arduino.
- 'data' of RF433 transmitter needs be plugged on PIN 'D3' of Arduino.

Sonoff codes are made of 24-bit codes encoded in a
'3-low-then-1-high versus 1-low-then-3-high' bit coding mechanism, see Sonoff
library.

Adf codes are made of 32-bit codes encoded in Manchester, see Adf library.


Usage
-----

Requires to install 2 libraries in the Arduino environment:

1. Sonoff, available here: https://github.com/sebmillet/sonoff

2. Adf, available here: https://github.com/sebmillet/adf

