/*
 * i2c_prot.h
 *
 *  Created on: June 29, 2021
 *  @author Dawson Bancroft
 */
#ifndef I2C_PROT_H_
#define I2C_PROT_H_

// IF USING DIFFERENT PINS OR A DIFFERENT MICROCONTROLLER CHANGE THE NEXT 6 LINES
#include <msp430.h>

#define I2C_PORT_DIR            P5DIR
#define I2C_PORT_OUT            P5OUT
#define I2C_PORT_IN             P5IN
#define I2C_SDA                 BIT0
#define I2C_SCL                 BIT1
// ANYTHING AFTER THIS SHOULD BE FINE

#define I2C_DELAY               5
#define I2C_WRITE               0
#define I2C_READ                1
#define I2C_SLAVE_ACK           'Y'
#define I2C_SLAVE_NACK          'N'

// FUNCTION DEF
int i2c_init_proc(void);
void i2c_pulse_clock_proc(void);
void i2c_start_proc(void);
void i2c_stop_proc(void);
void i2c_send_ack_proc(void);
void i2c_send_nack_proc(void);
char i2c_receive_ack_or_nack_proc(void);
char i2c_send_byte_proc(char data);
char i2c_receive_byte_proc(void);
void i2c_send_address_proc(char address, int rW);

#endif /* IR_CAMERA_I2C_PROT_H_ */
