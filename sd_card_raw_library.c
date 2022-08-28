#include <msp430.h>
#include "sd_card_raw_library.h"
/*
 * sd_card_raw_library.c
 *
 *  Created on: June 29, 2021
 *  @author Dawson Bancroft
 */

// Globals
unsigned char sdCardPacket[514];        // Data To the SD Card
unsigned int address_error = 0;

// Send a SPI Byte
char sendByteSPI(char data)
{
    int i;
    int moreThanOne = 1;
    if(SPI_PORT_OUT & SPI_STE != 0)             // Ensure STE is Down
    {
        SPI_PORT_OUT &= ~SPI_STE;
        moreThanOne = 0;
    }
    for(i = 8; i > 0; i--)
    {
        if(data & 0x80)                         // Send Data Out for Slave
            SPI_PORT_OUT |= SPI_MOSI;
        else
            SPI_PORT_OUT &= ~SPI_MOSI;
        data = data << 1;

        SPI_PORT_OUT &= ~SPI_CLK;               // Cycle Clock
        SPI_PORT_OUT |= SPI_CLK;

        if(SPI_PORT_IN & SPI_MISO)              // Read Data from Slave
            data |= BIT0;
        else
            data &= ~BIT0;
    }
    if(moreThanOne == 0)
    {
        SPI_PORT_OUT |= SPI_STE;
    }
    return data;
}

// Stop SPI to Ensure Correct Protocol
void stopSPI(void)
{
    SPI_PORT_OUT |= SPI_STE | SPI_CLK | SPI_MOSI;
}

// Clock Pulse Function to Set Slave Clock Speed
void pulseClock(int num)
{
    int i, j;
    for(j = num; j > 0; j--)
    {
        for(i = 8; i > 0; i--)
        {
            int data = 0x00, dummy = 0x00;
            SPI_PORT_OUT |= SPI_STE;
            if(data & 0x80)
                dummy |= BIT0;
            else
                dummy &= ~BIT0;
            data = data << 1;

            SPI_PORT_OUT &= ~SPI_CLK;               // Cycle Clock
            SPI_PORT_OUT |= SPI_CLK;

            if(SPI_PORT_IN & SPI_MISO)              // Read Data from Slave
                data |= BIT0;
            else
                data &= ~BIT0;

        }
    }
}

// Send an Array of SPI Data
void sendDataSPI(unsigned char* buffer, unsigned int size)
{
    int i;
    int moreThanOne = 1;
    if(SPI_PORT_OUT & SPI_STE != 0)
    {
        SPI_PORT_OUT &= ~SPI_STE;                   // Ensure STE is Down
        moreThanOne = 0;
    }
    for(i = 0; i < size; i++)
    {
        sendByteSPI(buffer[i]);
    }
    if(moreThanOne == 0)
    {
        SPI_PORT_OUT |= SPI_STE;
    }
}

// Sends a Command Without Pulling CS High
void sendCommandWithoutPullingCSHigh(char cmd, long data, char crc)
{
    unsigned char frame[6];
    char temp;
    int i;
    frame[0] = cmd;
    for(i = 3; i >= 0; i--)
    {
        temp = (char)(data>>(8*i));
        frame[4 - i] = (temp);
    }
    frame[5] = (crc);
    SPI_PORT_OUT &= ~SPI_STE;
    sendDataSPI(frame, 6);
}

// Sends a Command to the SD Card
void sendCommand(char cmd, long data, char crc, unsigned char* received)
{
    unsigned char frame[6];
    char temp;
    int i;
    frame[0] = cmd;
    for(i = 3; i >= 0; i--)
    {
        temp = (char)(data>>(8*i));
        frame[4 - i] = (temp);
    }
    frame[5] = (crc);
    SPI_PORT_OUT &= ~SPI_STE;
    sendDataSPI(frame, 6);
    int cnt = 0;
    while(!(cnt >= 5))
    {
        received[cnt] = sendByteSPI(0xFF);
        cnt++;
    }
    SPI_PORT_OUT |= SPI_STE;
}

// SPI Ports Initialization
void SPIInit(void)
{
    SPI_PORT_DIR = SPI_STE | SPI_CLK | SPI_MOSI;
    SPI_PORT_OUT = SPI_STE | SPI_CLK | SPI_MOSI | SPI_MISO;
    SPI_PORT_REN = SPI_MISO;
}

// SD Card Initialization Sequence
void sdCardInit(void)
{
    sdCardPacket[0] = 0xFE;
    sdCardPacket[513] = 0xFF;

    __delay_cycles(500000);
    pulseClock(10);
    unsigned char dataIn[6];
    sendCommand(0x40, 0, 0x95, dataIn);                 // CMD 0  : Software Reset              R1
    sendCommand(0x48, 0x000001AA, 0x87, dataIn);        // CMD 8  : Send Voltage Information    R3
    sendCommand(0x7A, 0, 0, dataIn);                    // CMD 58 : Read OCR Register           R3

    while(dataIn[1]!=0x00)
    {
        sendCommand(0x77, 0, 0x00, dataIn);             // CMD 55 : Preface for ACMD Commands   R1
        sendCommand(0x69, 0x40000000, 0x00, dataIn);    //ACMD 41 : Initialization              R1
    }
}

// Used for Sending Data to be Written to the SD Card, data must be 512 bytes
void sendData(long address, unsigned char* data)
{
    char in = 0xFF;
    unsigned int i;
    for(i = 0; i < 512; i++)
    {
        sdCardPacket[i + 1] = data[i];                      // Build the Packet
    }
    sendCommandWithoutPullingCSHigh(0x58, address, 0xFF);   // CMD 24 : Send a Single Block of Data
    int k = 0;                                                // Restart condition
    while(in == 0xFF && k != 100)
    {
        in = sendByteSPI(0xFF);
        k++;
    }
    if(in != 0x00 || k == 100)
    {
        sendByteSPI(0xFF);
        stopSPI();
        address_error = address;
        sdCardInit();
        return;                        // Changed so when SD card is full program continues to run
    }
    sendByteSPI(0xFF);
    for(i = 0; i < sizeof(sdCardPacket); i++)
    {
        sendByteSPI(sdCardPacket[i]);
    }
    SPI_PORT_OUT |= SPI_STE;
    unsigned char dataIn[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};     // Ensure Card Isn't Busy Before Continuing
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    sendCommand(0x4D, 0, 0, dataIn);
    while((dataIn[0] != 0xFF) && (dataIn[1] != 0) && (dataIn[2] != 0) && (dataIn[3] != 0xFF) && (dataIn[4] != 0xFF)){
        sendCommand(0x4D, 0, 0, dataIn);
    }
    sendCommand(0x4D, 0, 0, dataIn);
    SPI_PORT_OUT |= SPI_STE;
}

void receiveData(long address, unsigned char* data)
{
    char in = 0xFF;
    unsigned int i;
    pulseClock(10);
    sendCommandWithoutPullingCSHigh(0x51, address, 0xFF);     // CMD 17 : Receive a single block of data
    while(in == 0xFF)
    {
        in = sendByteSPI(0xFF);
    }
    if(in != 0x01) return;
    SPI_PORT_OUT &= ~SPI_CLK;
    SPI_PORT_OUT |= SPI_CLK;
    while(in != 0xFC)
    {
        in = sendByteSPI(0xFF);
    }
    for(i = 0; i < 512; i++)
    {
        data[i] = sendByteSPI(0xFF);
    }
    sendByteSPI(0xFF);
    sendByteSPI(0xFF);
    SPI_PORT_OUT |= SPI_STE;
}
