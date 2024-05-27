#ifndef KEYPAD__H
#define KEYPAD__H
#include "main.h"

#define KEYPAD_ROW	4
#define KEYPAD_COL	4

uint8_t KEYPAD_GetKey(void);
uint8_t key_scan_EOS(void);

#endif
