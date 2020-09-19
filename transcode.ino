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
  About the board being 'busy' or not.

  Scheduled routines are called from timer1 interrupt handler, timer1 being
  setup to execute at 50 Hz (every 20 milliseconds).
  These scheduled routines can consist in sending ('tx') a code, that could
  negatively interfere with an ongoing rx ot tx.
  We therefore implemented the function
    isr_is_board_busy()
  that will tell the interrupt handler whether or not, it is possible to
  start the execution of a possibly long-lasting code.
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

#define DEBUG

#define ARRAYSZ(a) (sizeof(a) / sizeof(*a))

  // Input (codes received from Sonoff telecommand)
#define CODE_BTN_HAUT     0x00b94d24
#define CODE_BTN_BAS      0x00b94d22

    // Salon
#define CODE_VOLET1_OPEN  0x40A2BBAE
#define CODE_VOLET1_CLOSE 0x40A2BBAD
    // Salle à manger
#define CODE_VOLET2_OPEN  0x4003894D
#define CODE_VOLET2_CLOSE 0x4003894E
    // Chambre
#define CODE_VOLET3_OPEN  0x4078495E
#define CODE_VOLET3_CLOSE 0x4078495D

unsigned long codes_openall[] = {
    CODE_VOLET1_OPEN,
    CODE_VOLET2_OPEN,
    CODE_VOLET3_OPEN
};

unsigned long codes_closeall[] = {
    CODE_VOLET1_CLOSE,
    CODE_VOLET2_CLOSE,
    CODE_VOLET3_CLOSE
};

typedef struct {
    unsigned long in_code;
    unsigned long *out_array_codes;
    byte out_nb_codes;
} code_t;

code_t codes[] = {
    { CODE_BTN_HAUT,  codes_openall,  ARRAYSZ(codes_openall)  },
    { CODE_BTN_BAS,   codes_closeall, ARRAYSZ(codes_closeall) }
};

  // Comment the below line if you don't want a LED to show RF transmission is
  // underway.
#define PIN_LED          LED_BUILTIN

#include "sonoff.h"
#include "adf.h"

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

typedef struct {
    bool is_set;
    unsigned long time_ms;
    void (*func)(void* data);
    void *data;
} schedule_t;
#define SCHEDULE_SLOTS 2
schedule_t schedules[SCHEDULE_SLOTS];

    // Adds a scheduled task in the list.
    // Returns:
    //   0 (no error).
    //   A non-null value (an error occurred).
byte schedule(unsigned long start_time_ms,
              void (*func)(void* data), void* data) {
    for (byte i = 0; i < SCHEDULE_SLOTS; ++i) {
        if (!schedules[i].is_set) {
            schedule_t* s = &schedules[i];
            s->time_ms = start_time_ms;
            s->func = func;
            s->data = data;
            s->is_set = true;
            return 0;
        }
    }
    return 1;
}

void cancel_schedules() {
    for (byte i = 0; i < SCHEDULE_SLOTS; ++i) {
        schedules[i].is_set = false;
    }
}

bool busy = false;

Sonoff rx;
Adf adf;

#define OP CODE_VOLET2_OPEN
#define CL CODE_VOLET2_CLOSE

void my_adf_rf_send_instruction(uint32_t code,
                                bool schedule_deferred_code_as_appropriate);
void my_adf_deferred(void *data) {
    serial_printf("my_adf_deferred: execution\n");
    my_adf_rf_send_instruction(OP, false);
}

void my_adf_rf_send_instruction(uint32_t code,
                                bool schedule_deferred_code_as_appropriate) {

    static bool last_is_close = false;
    static unsigned long last_close_ms = 0;

    busy = true;
    unsigned long t_ms = millis();
    adf.rf_send_instruction(code);
    serial_printf(">>> [SND] = 0x%08lx\n", code);

    if (schedule_deferred_code_as_appropriate) {

        bool do_sched = (code == CL);

        if (last_is_close) {
            if (code == CL) {
                if (t_ms - last_close_ms > 1000) {
                    do_sched = false;
                    last_close_ms = t_ms;
                }
            } else if (code == OP) {
                last_is_close = false;
            }
        } else {
            if (code == CL) {
                last_is_close = true;
                last_close_ms = t_ms;
            } else if (code == OP) {
                last_is_close = false;
            }
        }

        if (do_sched) {
            schedule(t_ms + 16500, my_adf_deferred, nullptr);
//            schedule(t_ms + 2650, my_adf_deferred, nullptr);
        }
    }

    busy = false;
}

bool isr_is_board_busy() {
    return busy || rx.is_busy();
}

    // timer1 is 50 Hz, see timer1 initialization in setup() function
ISR(TIMER1_COMPA_vect) {
    if (isr_is_board_busy()) {
        return;
    }
    unsigned long t_ms = millis();
    for (byte i = 0; i < SCHEDULE_SLOTS; ++i) {
        schedule_t* s = &schedules[i];
        if (s->is_set) {
                // NOT A TYPO
                // We convert unsigned to signed willingly
            long delta_ms = (long int)(t_ms - s->time_ms);
            if (delta_ms >= 0) {
                busy = true;
                sei();
                (*s->func)(s->data);
                s->is_set = false;
                busy = false;
            }
        }
    }
}

void setup() {
    serial_begin(115200);
    serial_printf("Start\n");

    pinMode(PIN_RFINPUT, INPUT);

#ifdef PIN_LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
#endif

        // DEFENSIVE PROGRAMMING
        // init() is called by Adf constructor, however we prefer to call it
        // here again, to make sure the transmitter PIN is setup in OUTPUT mode
        // and it is set to zero (no transmission).
        // Indeed, it could be that the call from constructor is done too early
        // in the board' start sequence.
        //
        // WHY WOULD THIS POINT BE SO IMPORTANT?
        //   If not handled properly, it could end up with TX left 'active' and
        //   spreading 433Mhz waves continuously, creating useless noise and
        //   damaging the TX device if left active for a long while.
    adf.init();

    cancel_schedules();

    cli();

        // From https://www.instructables.com/id/Arduino-Timer-Interrupts/
        // Arduino timer maths:
        //   FREQ = ARDCLOCKFREQ / (prescaler * (comparison match register + 1))
        //   With ARDCLOCKFREQ = 16000000 (Arduino board built-in frequency)

        // Setup timer1 interrupts
        // freq below = 16000000 / (256 * (1249 + 1)) = 50Hz
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 =  0;
    OCR1A =  1249;         // Comparison match register
    TCCR1B |= 1 << WGM12;  // Turn on CTC mode
    TCCR1B |= 1 << CS12;   // Set CS10 for 256 prescaler
    TIMSK1 |= 1 << OCIE1A; // Enable timer compare interrupt

    sei();
}

void loop() {
    serial_printf("Waiting for signal\n");

    uint32_t val = rx.get_val(true);

    code_t* c = nullptr;
    for (byte i = 0; i < ARRAYSZ(codes); ++i) {
        if (val == codes[i].in_code) {
            c = &codes[i];
        }
    }

    if (c) {

#ifdef PIN_LED
        digitalWrite(PIN_LED, HIGH);
#endif

        cancel_schedules();
        delay(10);
        for (byte i = 0; i < c->out_nb_codes; ++i) {
            my_adf_rf_send_instruction(c->out_array_codes[i], true);
        }

#ifdef PIN_LED
        digitalWrite(PIN_LED, LOW);
#endif

    }

    serial_printf("<<< [RCV] = 0x%08lx\n", val);
    if (!c) {
        serial_printf("   No code sent (unknown input code)\n");
    }
}

