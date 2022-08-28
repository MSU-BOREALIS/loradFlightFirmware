/*
 * sd_card_raw_library.h
 *
 *  Created on: June 29, 2021
 *  @author Dawson Bancroft
 */

#ifndef SD_CARD_RAW_LIBRARY_SD_CARD_RAW_LIBRARY_H_
#define SD_CARD_RAW_LIBRARY_SD_CARD_RAW_LIBRARY_H_
extern unsigned int address_error;

// IF USING DIFFERENT PINS OR A DIFFERENT MICROCONTROLLER CHANGE THE NEXT 6 LINES
#include <msp430.h>

#define SPI_PORT_DIR                    P1DIR
#define SPI_PORT_OUT                    P1OUT
#define SPI_PORT_IN                     P1IN
#define SPI_PORT_REN                    P1REN
#define SPI_STE                         BIT0
#define SPI_CLK                         BIT1
#define SPI_MOSI                        BIT2
#define SPI_MISO                        BIT3
#define SPI_CARD_DETECT                 BIT6
// ANYTHING AFTER THIS SHOULD BE FINE

// FUNCTION DEFS
char sendByteSPI(char data);
void stopSPI(void);
void pulseClock(int num);
void sendDataSPI(unsigned char* buffer, unsigned int size);
void sendCommandWithoutPullingCSHigh(char cmd, long data, char crc);
void sendCommand(char cmd, long data, char crc, unsigned char* received);
void SPIInit(void);
void sdCardInit(void);
void sendData(long address, unsigned char* data);
void receiveData(long address, unsigned char* data);

#endif /* SD_CARD_RAW_LIBRARY_SD_CARD_RAW_LIBRARY_H_ */
