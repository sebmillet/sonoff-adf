// vim: ts=4:sw=4:et:tw=80

/*
  transcode.ino

  Receives a code from a Sonoff telecommand and sends it to "Automatismes De
  France" (adf) slater.
  - Receive from an RF 433 Mhz receiver.
  - Send with an RF 433 Mhz transmitter.

  Schema:
    'data' of RF433 receiver needs be plugged on PIN 'D2' of Arduino.
    'data' of RF433 transmitter needs be plugged on PIN 'D3' of Arduino.
*/

/*
  Copyright 2020 Sébastien Millet

  transcode.ino is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  transcode.ino is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses>.
*/

  // Input (codes received from Sonoff telecommand)
#define CODE_BTN_HAUT     0x00b94d24
#define CODE_BTN_BAS      0x00b94d22

  // Output (codes to send to slater)
#define CODE_VOLET1_OPEN  0x40A2BBAE
#define CODE_VOLET1_CLOSE 0x40A2BBAD

#define NB_REPEAT_SEND             2

  // Comment the below line if you don't want a LED to show RF transmission is
  // underway.
#define PIN_LED          LED_BUILTIN

#include "sonoff.h"
#include "adf.h"

//#define DEBUG

#ifdef DEBUG

static char serial_printf_buffer[80];

static void serial_printf(const char* fmt, ...)
     __attribute__((format(printf, 1, 2)));

static void serial_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(serial_printf_buffer, sizeof(serial_printf_buffer), fmt, args);
    va_end(args);

    serial_printf_buffer[sizeof(serial_printf_buffer) - 1] = '\0';
    Serial.print(serial_printf_buffer);
}

static void serial_begin(long speed) {
    Serial.begin(speed);
}

#else // DEBUG

#define serial_printf(...)
#define serial_begin(speed)

#endif // DEBUG

Sonoff rx;
Adf adf;

void setup() {
    serial_begin(115200);
    serial_printf("Start\n");

    pinMode(PIN_RFINPUT, INPUT);

#ifdef PIN_LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
#endif

    adf.init();
}

void loop() {
    serial_printf("Waiting for signal\n");

    uint32_t val = rx.get_val();

    rx.wait_free_433();

    uint32_t fwd_code;
    bool do_fwd_code = false;

    if (val == CODE_BTN_HAUT) {
        fwd_code = CODE_VOLET1_OPEN;
        do_fwd_code = true;
    } else if (val == CODE_BTN_BAS) {
        fwd_code = CODE_VOLET1_CLOSE;
        do_fwd_code = true;
    }
    if (do_fwd_code) {

#ifdef PIN_LED
        digitalWrite(PIN_LED, HIGH);
#endif

        for (byte i = 0; i < NB_REPEAT_SEND; ++i) {
            adf.rf_send_instruction(fwd_code);
        }

#ifdef PIN_LED
        digitalWrite(PIN_LED, LOW);
#endif

    }

    serial_printf("== Received 0x%08lx\n", val);
    if (do_fwd_code) {
        serial_printf("   Sent     0x%08lx\n", fwd_code);
    } else {
        serial_printf("   No code sent (unknown input code)\n");
    }
}

