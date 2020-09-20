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

  Also receives instructions from USB.
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

/*
  TODO (~FIXME actually...)
  The management of 'OP' code that must follow 'CL' code (to stop slater before
  it reaches the end) is uggly, needs proper redev. Not only is it ignoring
  *data argument (in the function registered for later execution), but it is
  deeply flawed. Proper way to manage it would be for example:
  - Create a Slater object that manages its open and close codes.
  - Have it recognize whether or not the CL code shall be followed by OP code,
    depending on its status.
  The slater will then be able to manage its status ('I am open', 'I am
  closed'), to make a decision.
  Also doing this way will allow smooth implementation of thresholds to avoid
  too frequent actions and protect slaters, like for example: no more than 1
  action per second, no more than 12 per minute, no more than 24 per hour, no
  more than 48 per day. This'd allow usual, regular actions while blocking crazy
  requests due to a bug or compromission of the source of orders.
  The current code would make it complicated to implement such thresholds.
*/

//#define DEBUG

#define NOOP_BLINK

#define ARRAYSZ(a) (sizeof(a) / sizeof(*a))

  // Input (codes received from Sonoff telecommand)
#define CODE_IN_BTN_HAUT     0x00b94d24
#define CODE_IN_BTN_BAS      0x00b94d22

    // Salon
#define CODE_OUT_VOLET1_OPEN  0x40A2BBAE
#define CODE_OUT_VOLET1_CLOSE 0x40A2BBAD
    // Salle à manger
#define CODE_OUT_VOLET2_OPEN  0x4003894D
#define CODE_OUT_VOLET2_CLOSE 0x4003894E
    // Chambre
#define CODE_OUT_VOLET3_OPEN  0x4078495E
#define CODE_OUT_VOLET3_CLOSE 0x4078495D

unsigned long codes_openall[] = {
    CODE_OUT_VOLET1_OPEN,
    CODE_OUT_VOLET2_OPEN,
    CODE_OUT_VOLET3_OPEN
};

unsigned long codes_closeall[] = {
    CODE_OUT_VOLET1_CLOSE,
    CODE_OUT_VOLET2_CLOSE,
    CODE_OUT_VOLET3_CLOSE
};

typedef struct {
    unsigned long in_code;
    unsigned long *out_array_codes;
    byte out_nb_codes;
} code_t;

code_t codes[] = {
    { CODE_IN_BTN_HAUT,  codes_openall,  ARRAYSZ(codes_openall)  },
    { CODE_IN_BTN_BAS,   codes_closeall, ARRAYSZ(codes_closeall) }
};

  // Comment the below line if you don't want a LED to show RF transmission is
  // underway.
#define PIN_LED          LED_BUILTIN

#include "sonoff.h"
#include "adf.h"
#include "serial_speed.h"

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

#else // DEBUG

#define serial_printf(...)

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

void turn_led_on() {
#ifdef PIN_LED
    digitalWrite(PIN_LED, HIGH);
#endif
}

void turn_led_off() {
#ifdef PIN_LED
    digitalWrite(PIN_LED, LOW);
#endif
}

Sonoff rx;
Adf adf;

#define OP CODE_OUT_VOLET2_OPEN
#define CL CODE_OUT_VOLET2_CLOSE

void my_adf_rf_send_instruction(uint32_t code,
                                bool schedule_deferred_code_as_appropriate);

// FIXME
//   OP sent (instead of using *data);
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
    turn_led_on();
    adf.rf_send_instruction(code);
    turn_led_off();
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
                // FIXME, nullptr (whereas pointer to OP shall be used)
            schedule(t_ms + 16500, my_adf_deferred, nullptr); // PROD
//            schedule(t_ms + 2650, my_adf_deferred, nullptr); // TEST
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
    Serial.begin(SERIAL_SPEED_INTEGER);
    serial_printf("Start\n");

    pinMode(PIN_RFINPUT, INPUT);

    turn_led_off();

        // Defensive programming:
        //   init() is called by Adf constructor, however we prefer to call it
        //   here again, to make sure the transmitter PIN is setup in OUTPUT
        //   mode and it is set to zero (no transmission).
        //   Indeed, it could be that the call from constructor is done too
        //   early in the board' start sequence.
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

void manage_recv_from_rx(uint32_t val) {
    code_t* c = nullptr;
    for (byte i = 0; i < ARRAYSZ(codes); ++i) {
        if (val == codes[i].in_code) {
            c = &codes[i];
        }
    }

    if (c) {
        cancel_schedules();
        delay(10);
        for (byte i = 0; i < c->out_nb_codes; ++i) {
            my_adf_rf_send_instruction(c->out_array_codes[i], true);
        }
    }
    serial_printf("<<< [RCV] = 0x%08lx\n", val);
    if (!c) {
        serial_printf("   No code sent (unknown input code)\n");
    }
}


//
// SerialLine
//
// Manages USB input as lines.
//
// Interest = non blocking I/O. Serial.readString() works with timeout and a
// null timeout is not well documented (meaning: even if zeroing timeout leads
// to non-blocking I/O, I'm not sure it'll consistently and robustly *always*
// behave this way).
class SerialLine {
    private:
        char buf[83]; // 80-character strings (then CR+LF then NULL-terminating)
        size_t head;
        bool got_a_line;
        void reset();

    public:
        SerialLine();

        static const size_t buf_len;

        void do_events();
        bool is_line_available();
        bool get_line(char *s, size_t len);
        void split_s_into_func_args(char *s, char **func, char **args) const;
};
const size_t SerialLine::buf_len = sizeof(SerialLine::buf);

SerialLine::SerialLine():head(0),got_a_line(false) { };

void SerialLine::do_events() {
    if (got_a_line)
        return;
    if (!Serial.available())
        return;

    int b;
    do {
        b = Serial.read();
        if (b == -1)
            break;
        buf[head++] = (char)b;
    } while (head < buf_len - 1 && b != '\n' && Serial.available());

    if (head < buf_len - 1 && b != '\n')
        return;

    buf[head] = '\0';

        // Remove trailing cr and/or nl
        // FIXME?
        //   WON'T WORK WITH MAC NEWLINES!
        //   (SEE ABOVE: NO STOP IF ONLY CR ENCOUNTERED)
    if (head >= 1 && buf[head - 1] == '\n')
        buf[--head] = '\0';
    if (head >= 1 && buf[head - 1] == '\r')
        buf[--head] = '\0';
    got_a_line = true;
}

bool SerialLine::is_line_available() {
    do_events();
    return got_a_line;
}

void SerialLine::reset() {
    head = 0;
    got_a_line = false;
}

// Get USB input as a simple line, copied in caller buffer.
// A 'line' is a set of non-null characters followed by 'new line', 'new line'
// being either as per Unix or Windows convention, see below.
// Returns true if a copy was done (there was a line available), false if not
// (in which case, s is not updated).
// The terminating newline character (or 2-character CR-LF sequence) is NOT part
// of the string given to the caller.
// If the line length is above the buffer size (SerialLine::buf_len), then it'll
// be cut into smaller pieces.
// Because of the way the received buffer is parsed, and when using CR-LF as
// end-of-line marker (default even under Linux), it can result in a empty
// string seen after a first string with a length close to the limit.
//
// About new lines:
// - Works fine with Unix new lines (\n), tested
// - Supposed to work fine with Windows new lines (\r\n), NOT TESTED
// - WON'T WORK WITH MAC NEW LINES (\r)
bool SerialLine::get_line(char *s, size_t len) {
    do_events();
    if (!got_a_line)
        return false;
    snprintf(s, len, buf);
    reset();
    return true;
}

// Take a string and splits it into 2 parts, one for the function name, one for
// the arguments.
//
// IMPORTANT
//   THE PROVIDED STRING, s, IS ALTERED BY THE OPERATION.
//
// The arguments are everything in parenthesis that follows the function name.
// For example, if the line received is:
//   myfunc(123, 456)
// then
//   func will point to the string
//     myfunc
//   args will point to the string
//     123, 456
//
// If the string is not made of a function name followed by arguments in
// parenthesis, then func will point to the complete line and args will point to
// an empty string.
//
// Note the arguments parsing is basic, and there is no escape mechanism.
// That is, a call like:
//   myfunc("(bla)", 123)
// won't be parsed properly, because the first closing parenthesis will be
// considered as being the one that closes the function arguments. As it is
// followed by trailing characters, it'll result in func pointing to the
// complete string, and args pointing to an empty string.
void SerialLine::split_s_into_func_args(char *s, char **func, char **args)
        const {
    size_t h = 0;
    *func = s;
    while (s[h] != '(' && s[h] != '\0')
        h++;
    if (s[h] == '(') {
        char *open_parenthesis = s + h;
        *args = s + h + 1;
        while (s[h] != ')' && s[h] != '\0')
            h++;
        if (s[h] == ')') {
            if (h < buf_len - 1 && s[h + 1] == '\0') {
                *open_parenthesis = '\0';
                s[h] = '\0';
            } else {
                    // Trailing characters after closing parenthesis -> no
                    // arguments.
                *args = NULL;
            }
        } else {
                // No closing parenthesis -> no arguments
            *args = NULL;
        }
    } else {
        *args = NULL;
    }
}

    // Convert a string (decimal or hex) into an unsigned long int.
    // Assume hex if start is "0x", otherwise, assume decimal.
unsigned long get_32bit_code(const char *code) {
    if (code[0] == '0' && code[1] == 'x') {
        return strtoul(code + 2, NULL, 16);
    }
    return strtoul(code, NULL, 10);
}

void rftx(const char *a) {
    unsigned long code = get_32bit_code(a);

    my_adf_rf_send_instruction(code, true);
}

void noop(const char *a) {
#ifdef NOOP_BLINK
    for (int i = 0; i < 2; ++i) {
        turn_led_on();
        delay(125);
        turn_led_off();
        delay(125);
    }
#endif // NOOP_BLINK
}

void manage_recv_serial(char *func, char*args) {
    if (!strcmp(func, "rftx")) {
        rftx(args);
    } else if (!strcmp(func, "noop")) {
        noop(args);
    } else if (!strcmp(func, "")) {
        // Do nothing if empty instruction
        // Alternative: treat as an error
    } else {
            // Unknown function: we blink 4 times on internal LED
        for (int i = 0; i < 4; ++i) {
            turn_led_on();
            delay(125);
            turn_led_off();
            delay(125);
        }
    }
    serial_printf("<<< [USB] = function: [%s], arguments: [%s]\n", func, args);
}

SerialLine sl;
char buffer[SerialLine::buf_len];

void loop() {
    uint32_t val;
    if (rx.get_val_non_blocking(&val, true)) {
        manage_recv_from_rx(val);
    }
    if (sl.get_line(buffer, sizeof(buffer))) {
        char *func, *args;
        sl.split_s_into_func_args(buffer, &func, &args);
        manage_recv_serial(func, args);
    }
}

