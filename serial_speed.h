//
// File shared by 2 very different programs, that MUST share information about
// the serial line (USB actually) speed.
//   mapper-devusb.c    daemon to pass on orders to Arduino board
//   transcode.ino      Arduino sketch to (among others things) read from USB
//                      the orders sent by mapper-devusb daemon

// Copyright 2020 Sébastien Millet


    // Used by mapper-devusb.c
    // Constant is speed_t, to call cfsetospeed
#define SERIAL_SPEED_SPEED_T B115200

    // Used by transcode.ino
    // Constant is a regular integer, to call Serial.begin
#define SERIAL_SPEED_INTEGER 115200

