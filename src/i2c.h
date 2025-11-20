/*
 * i2c.h
 *
 *  Created on: May 19, 2019
 *      Author: Sam
 */

#ifndef I2C_H_
#define I2C_H_

#include <msp430.h>
#define smclk_frequency 1000000
#define i2c_frequency 50000

extern unsigned char *PTxData;                                                       // Pointer to TX data
extern unsigned char TxByteCtr;                                                      // number of bytes to TX

/* ====================================================================
 * I2C Prototype Definitions
 * ==================================================================== */
void i2c_init(void);

void i2c_write(unsigned char slave_address, unsigned char *DataBuffer, unsigned char ByteCtr);
unsigned char i2c_read_byte(unsigned char slave_address) ;
#endif /* I2C_H_ */
