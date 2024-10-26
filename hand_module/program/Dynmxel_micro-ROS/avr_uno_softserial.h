#pragma once

// Collection of routines for initialization, sending and receiving using SOFTWARESERIAL with GPIO of Arduino UNO.

#if HARDWARE_TYPE == AVR_UNO

#include  <stdint.h>
#include  <SoftwareSerial.h>

SoftwareSerial *mysoftuart = NULL;

#define MY_RX_PIN 14
#define MY_TX_PIN 15

uint32_t us_init (uint32_t baud) {
  mysoftuart = new SoftwareSerial (MY_RX_PIN, MY_TX_PIN);
  mysoftuart->begin (baud);
  mysoftuart->setTimeout (20);
  return baud;
}

void us_deinit (void) {
  mysoftuart->end();
  mysoftuart = NULL;
}

uint32_t us_setbaudrate (uint32_t baud) {
  us_deinit();
  return us_init (baud);
}

void us_rxpurge (void) {
  while (mysoftuart->available()) mysoftuart->read();
}

void us_putc (uint8_t c) {
  mysoftuart->write (c);
}

void us_puts (const uint8_t *buf, int len) {
  mysoftuart->write (buf, len);
}

int us_gets (uint8_t *buf, int len) {
  return mysoftuart->readBytes (buf, len);
}

void us_flush (void) {
  mysoftuart->flush();
}

#endif
