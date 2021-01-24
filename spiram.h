/*
 * driver for spi ram connected to ospi controller
 */
#ifndef __SPIRAM_H__
#define __SPIRAM_H__
#include <stdbool.h>
bool spiram_init(void);       // memory-map spiram
void *spiram_start(void);     // lowest spiram address
void *spiram_end(void);       // highest spiram address+1
bool spiram_test(bool fast);  // run memtest
void spiram_dmesg();          // print memtest result on console
#endif // __SPIRAM_H__
