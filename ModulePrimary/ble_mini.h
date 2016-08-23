#ifndef  _BLE_MINI_H
#define _BLE_MINI_H

#include <Arduino.h>

void BLEMini_begin(unsigned long bound);
void BLEMini_end();
int BLEMini_available();
void BLEMini_write(unsigned char dat);
void BLEMini_write_bytes(unsigned char *dat, unsigned char len);
int BLEMini_read();
int BLEMini_parseInt();

#endif
