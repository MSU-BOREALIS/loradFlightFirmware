#include <msp430.h>
#include <i2c_prot.h>
/*
 * i2c_prot.c
 *
 *  Created on: June 29, 2021
 *  @author Dawson Bancroft
 */


// Initializes the ports for I2C
int i2c_init_proc(void)
{
    I2C_PORT_DIR |= (I2C_SDA | I2C_SCL);    // SET I2C_SDA and I2C_SCL AS OUTPUTS
    I2C_PORT_OUT |= (I2C_SDA | I2C_SCL);    // SET I2C_SDA and I2C_SCL HIGH
    return 0;
}

// Pulses I2C clock
void i2c_pulse_clock_proc(void)
{
    int i;
    I2C_PORT_OUT |= I2C_SCL;                // SET SCL HIGH
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_OUT &= ~I2C_SCL;               // SET SCL LOW
}

// I2C Start Condition
void i2c_start_proc(void)
{
    int i;
    I2C_PORT_OUT |= I2C_SDA;                // PULL SDA HIGH
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_OUT |= I2C_SCL;                // PULL SCL HIGH
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_OUT &= ~I2C_SDA;               // PULL SDA LOW
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_OUT &= ~I2C_SCL;               // PULL SCL LOW
}

// I2C Stop Condition
void i2c_stop_proc(void)
{
    int i;
    I2C_PORT_OUT &= ~I2C_SDA;               // PULL SDA LOW
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_OUT |= I2C_SCL;                // PULL SCL HIGH
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_OUT |= I2C_SDA;                // PULL SDA HIGH
}

// Send an I2C Ack
void i2c_send_ack_proc(void)
{
    int i;
    I2C_PORT_OUT &= ~I2C_SDA;               // SET SDA LOW
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    i2c_pulse_clock_proc();                      // PULSE THE CLOCK
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
}

// Send an I2C Nack
void i2c_send_nack_proc(void)
{
    int i;
    I2C_PORT_OUT |= I2C_SDA;                // SET SDA HIGH
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    i2c_pulse_clock_proc();                      // PULSE THE CLOCK
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_OUT &= ~I2C_SDA;               // SET SDA LOW
}

// Read an I2C Ack or Nack
char i2c_receive_ack_or_nack_proc(void)
{
    int i;
    I2C_PORT_DIR &= ~I2C_SDA;               // MAKE SDA INPUT
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    if(I2C_PORT_IN & I2C_SDA)               // CHECK IF SDA IS HIGH
    {
        i2c_pulse_clock_proc();                  // PULSE THE CLOCK
        for(i = 0; i < I2C_DELAY; i++){}    // DELAY
        I2C_PORT_DIR |= I2C_SDA;            // MAKE SDA OUTPUT
        for(i = 0; i < I2C_DELAY; i++){}    // DELAY
        return I2C_SLAVE_NACK;              // IF TRUE RETURN NACK
    }
    else
    {
        i2c_pulse_clock_proc();                  // PULSE THE CLOCK
        for(i = 0; i < I2C_DELAY; i++){}    // DELAY
        I2C_PORT_DIR |= I2C_SDA;            // MAKE SDA OUTPUT
        for(i = 0; i < I2C_DELAY; i++){}    // DELAY
        return I2C_SLAVE_ACK;
    }
}

// Send an I2C Byte
char i2c_send_byte_proc(char data)
{
    int i, j;
    for(i = 8; i > 0; i--)
    {
        if(data & 0x80)
            I2C_PORT_OUT |= I2C_SDA;        // SEND A HIGH BIT
        else
            I2C_PORT_OUT &= ~I2C_SDA;       // SEND A LOW BIT
        data = data << 1;
        for(j = 0; j < I2C_DELAY; j++){}    // DELAY
        i2c_pulse_clock_proc();                  // PULSE THE CLOCK
        for(j = 0; j < I2C_DELAY; j++){}    // DELAY

    }
    return i2c_receive_ack_or_nack_proc();
}

// Receive an I2C Byte
char i2c_receive_byte_proc(void)
{
    int i, j;
    I2C_PORT_DIR &= ~I2C_SDA;               // MAKE SDA INPUT
    char data = 0x00;                       // TO BE RECEIVED DATA
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    for(j = 0; j < 8; j++)
    {
        data = data << 1;
        if(I2C_PORT_IN & I2C_SDA)
        {
            data |= BIT0;
        }
        i2c_pulse_clock_proc();
    }
    for(i = 0; i < I2C_DELAY; i++){}        // DELAY
    I2C_PORT_DIR |= I2C_SDA;                // MAKE SDA OUTPUT
    return data;
}

// Send Address
void i2c_send_address_proc(char address, int rW)
{
    address = (address << 1) + rW;
    i2c_start_proc();
    i2c_send_byte_proc(address);
}
