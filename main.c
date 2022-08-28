/* main.c
 * This is the MAIN firmware (version 1.01) for running this firmware on the MSP430FR2355 based board
 *
 * Copyright © Montana Space Grant Consortium.
 *
 * @author Cameron Blegen
 * @author Larson Brandstetter
 *
 * ------------------------------------
 * Additions:
 * @author DawsonXBancroft
 */

#include <msp430.h> 
#include "i2c_prot.h"
#include "reg_map.h"
#include "I2C_defs.h"
#include "sd_card_raw_library.h"


//------Configurable Parameters------
#define PAYLOAD_LEN 0x7D
#define I2C_DELAY 5
#define SPI_DELAY 20

//----PAYLOAD STRUCT TO SEND DATA TO LORA-----
struct payload{
    long LORA_pressure;
    int LORA_int_temp;
    int LORA_ext_temp;
    int LORA_accel_x;
    int LORA_accel_y;
    int LORA_accel_z;
};

//--------------Globals--------------------
char j, k, m;
unsigned char h;
char i2c_status = 0x00; // 0b00000000 -- | 0 | 0 | 0 | 0 || 0 | 0 | 0 | 1-NAK | (currently unused)
// char temp_config_int, temp_config_ext; //uncomment this line if uncommenting the lines that get the config of the temp sensors
int i;
int int_temp, ext_temp, accel_x, accel_y, accel_z = 0; //actual temperature (internal and external) and acceleration values on each axis
unsigned int c[8]; //coefficients for the pressure sensor
unsigned int packet_tx_cnt = 0;
unsigned long digital_press, digital_press_temp = 0; //these are the variables that are used in calculating the pressure, not the actual pressure values
long dT, press_temp, pressure = 0; //actual temperature from pressure sensor and pressure
long long OFF, SENS = 0; //used for pressure calculation

// for SD
unsigned long address_cnt  = 0; // CNT for address of microSD
unsigned long address_last = 0; // For error flag

int SCL = BIT7;
int SDA = BIT6;

//-----GPS Globals-----
// The below character arrays are the strings needing to be sent to the GPS to properly configure the messages
char rmcOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X04, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X03, 0X3F};
char vtgOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X05, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X04, 0X46};
char gsaOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X02, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X01, 0X31};
char gsvOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X03, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X02, 0X38};
char gllOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X01, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X2A};
char ggaOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XFF, 0X23};
//char ggaOn[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X00, 0X00, 0X01, 0X01, 0X00, 0X00, 0X00, 0X01, 0X2C};       //gga on test
char zdaOff[] = {0XB5, 0X62, 0X06, 0X01, 0X08, 0X00, 0XF0, 0X08, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X07, 0X5B};
char polReq[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x19, 0xE5};
// GPS VARS
char gpsReceiveMessage[100];
unsigned int gpsCnt = 0;
int gpsDone = 0;
char gpsSunnySide[100];

//----LORA STRUCT----
struct payload LORA_PAYLOAD; //THIS IS THE LORA PAYLOAD ALL MEMBERS ARE DAYA TO BE SENT TO THE LORA, GLOBAL FOR INTERRUPT ACCESS

//--------------Functions--------------------

//-----SPI-----

//Delay for SPI
void delay(unsigned int j){
    unsigned int l;
    for(l = 0; l < j; l++){
        //P5OUT ^= BIT3; //Toggle LED3 Taken out
    }
}

void SPI_tx(char addr, char data){
    char temp;
    temp = 0x80 + addr; //0x80 is the send bit for SPI
    P4OUT &= ~NSS;
    UCA1TXBUF = temp;
    UCA1TXBUF = data;
    delay(SPI_DELAY);
    P4OUT |= NSS;
}

//currently unused
/*void SPI_burst_tx(char data[20]){
    for(i=0; i < PAYLOAD_LEN; i=i+2){
        UCA1TXBUF = data[i];
        delay(SPI_DELAY);
        UCA1TXBUF = data[i+1];
        delay(SPI_DELAY);
    }
    delay(PAYLOAD_LEN*SPI_DELAY);
}*/

char SPI_read_reg(char addr){
    char data;
    P4OUT &= ~NSS;
    UCA1TXBUF = addr;
    UCA1TXBUF = addr;
    data = UCA1RXBUF;
    delay(SPI_DELAY);
    P4OUT |= NSS;
    return data;
}

void SPI_tx_two_bytes(int val){
    UCA1TXBUF = (val >> 8);
    delay(SPI_DELAY);
    UCA1TXBUF = (val & 0x00FF);
    delay(SPI_DELAY);
}

//-----I2C-----

//cycle the i2c clock line high then low
void i2c_clock_cycle(void){
    P4OUT |= SCL; //SCL HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SCL; //SCL LOW
    P5OUT ^= BIT2; //Toggle LED3
}

//transmit a high bit on the i2c line
void i2c_tx_high_bit(void){
    P4OUT |= SDA; //SDA HIGH
    //toggle clock
    i2c_clock_cycle();
}

//transit a low bit on the i2c line
void i2c_tx_low_bit(void){
    P4OUT &= ~SDA; //SDA LOW
    //toggle clock
    i2c_clock_cycle();
}

//transmit the start condition on the i2c line, handling any starting position of the lines and setting the lines ready for the first bit
void i2c_start(void){
    P4OUT |= SDA; //SDA HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT |= SCL; //SCL HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SDA; //SDA LOW
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SCL; //SCL LOW
}

//transmit the stop condition on the i2c line, handling the starting position of the lines and setting the lines to "idle"
void i2c_stop(void){
    P4OUT &= ~SDA; //SDA LOW
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT |= SCL; //SCL HIGH
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT |= SDA; //SDA HIGH
}

//transmit a nack condition on the i2c line
void i2c_nack(void){
    P4OUT |= SDA; //SDA HIGH
    for(j=0; j<I2C_DELAY; j++){}
    i2c_clock_cycle();
    for(j=0; j<I2C_DELAY; j++){}
    P4OUT &= ~SDA; //SDA LOW
}

//transmit an ack on the i2c line
void i2c_ack(void){
    P4OUT &= ~SDA; //SDA LOW
    for(j=0; j<I2C_DELAY; j++){}
    i2c_clock_cycle();
    for(j=0; j<I2C_DELAY; j++){}
}

//read an ack from the slave and do something. Right now all we do is set a flag which is never cleared
void i2c_read_ack(void){
    P4DIR &= ~SDA; // SDA input port
    for(j=0; j<I2C_DELAY*2; j++){}
    if(P4IN & SDA){
        i2c_status |= NAK_FLAG;
    }
    i2c_clock_cycle();
    for(j=0; j<I2C_DELAY; j++){}
    P4DIR |= SDA; // SDA output port
    for(j=0; j<I2C_DELAY; j++){}
}

//given any byte, transmit it on the i2c line
void i2c_tx(char byte){
    for(i=8;i!=0;i--){
        if(byte & 0x80){
            i2c_tx_high_bit();
        }else{
            i2c_tx_low_bit();
        }
        byte = byte << 1;
    }
    i2c_read_ack();
}

//given any address and read/write mode (using the #define values from above) we send the address with the right mode bit
void i2c_tx_address(char addr, char mode){
    i2c_start();
    addr = (addr << 1) + mode;
    i2c_tx(addr);
}

//recieve the data from the slave device by setting the bit high or low in the data_in variable and shifting each bit recieved
char i2c_rx(void){
    char data_in = 0x00;
    P4DIR &= ~SDA; // SDA input port
    for(j=0; j<I2C_DELAY*2; j++){}

    for(i=8;i!=0;i--){
    data_in = data_in << 1;
        if(P4IN & SDA){
            data_in = data_in + 0x01;
        }
        i2c_clock_cycle();
    }
    for(j=0; j<I2C_DELAY; j++){}
    P4DIR |= SDA; // SDA output port
    return data_in;
}

//function to read the temperature value given a sensor address and address of the register we are trying to read. Returns the value.
char read_temp(char sensor, char addr){
    int data;
    i2c_tx_address(sensor, WRITE_MODE);
    i2c_tx(addr);
    i2c_tx_address(sensor, READ_MODE);
    data = i2c_rx();
    i2c_nack();
    i2c_stop();

    return data;
}

//read accelerometer axis data and return the value read. Must call this three times to get each axis.
//takes in the address of the axis register data (see define statements above)
char read_accel(char addr){
    char data;
    i2c_tx_address(ACCEL_ADDR, WRITE_MODE);
    i2c_tx(addr);
    i2c_tx_address(ACCEL_ADDR, READ_MODE);
    data = i2c_rx();
    i2c_nack();
    i2c_stop();

    return data;
}

//transmit to the pressure sensor, used inside the get pressure function
void i2c_tx_pressure(char addr){
    i2c_tx_address(PRESSURE_ADDR, WRITE_MODE);
    i2c_tx(addr);
    i2c_stop();
}

//Get the pressure coefficients, these coefficients are only received once and saved and then used for pressure calculation
void get_pressure_coeff(){
    for(k = 0; k < 8; k++){
        i2c_tx_pressure((0xA0+k*2));
        i2c_tx_address(PRESSURE_ADDR, READ_MODE);
        c[k] = i2c_rx();
        i2c_ack();
        c[k] = c[k] << 8;
        c[k] = c[k] + i2c_rx();
        i2c_nack();
        i2c_stop();
    }
}

//return the value read from the pressure sensor
unsigned long i2c_rx_pressure(){
    unsigned long data;
    i2c_tx_address(PRESSURE_ADDR, READ_MODE);
    data = i2c_rx();
    data = data << 8;
    i2c_ack();
    data = data + i2c_rx();
    data = data << 8;
    i2c_ack();
    data = data + i2c_rx();
    i2c_nack();
    i2c_stop();

    return data;
}

//calculate the value returned from the pressure sensor using the necessary equations to get the actual pressure value and pressure temp value
void convert_pressure(){
    long temp_var;
    long long calc_pressure;

    dT = c[5];
    dT = dT << 8;
    dT = digital_press_temp - dT;
    press_temp = c[6] >> 2;
    temp_var = dT >> 4;
    press_temp = press_temp * temp_var;
    press_temp = press_temp >> 17;
    press_temp = press_temp + 2000;

    OFF = c[2];
    OFF = OFF << 17;
    temp_var = temp_var * c[4];
    temp_var = temp_var >> 2;
    OFF = OFF + temp_var;
    SENS = c[1];
    SENS = SENS << 16;
    temp_var = dT >> 4;
    temp_var = temp_var * c[3];
    temp_var = temp_var >> 3;
    SENS = SENS + temp_var;

    calc_pressure = SENS >> 8;
    temp_var = digital_press;
    temp_var = temp_var >> 8;
    calc_pressure = calc_pressure * temp_var;
    calc_pressure = calc_pressure >> 5;
    calc_pressure = calc_pressure - OFF;
    calc_pressure = calc_pressure >> 15;
    pressure = calc_pressure;
    for(i=0; i < 10; i++){}
}


//-----UART-----

// Send a byte to the GPS
void sendByteGPS(char byte)
{
    int j;
    UCA0TXBUF = (byte);
    for(j=0; j<100;j++){}
}

// Send a message to the GPS
void sendGPSMessOff(char* message, int length)
{
    int i;
    for(i=0; i<length; i++){
        char temp = message[i];
        sendByteGPS(temp);
    }
}

// Descramble the GPS data to make it always the same
void descrambleGPSData(void)
{
    int i;
    int firstBestIndex = 0;
    for(i=0; i<sizeof(gpsSunnySide); i++)
    {
        if(gpsReceiveMessage[i] == 0xB5)
        {
            if(gpsReceiveMessage[i+1] == 0x62)
            {
                if(gpsReceiveMessage[i+2] == 0x01)
                {
                    if(gpsReceiveMessage[i+3] == 0x07)
                    {
                        if(gpsReceiveMessage[i+4] == 0x5C)
                        {
                            firstBestIndex = i;
                        }
                    }
                }
            }
        }
    }
    for(i=0; i<sizeof(gpsSunnySide); i++)
    {
        gpsSunnySide[i] = gpsReceiveMessage[(i+firstBestIndex)%sizeof(gpsReceiveMessage)];
    }
    return;
}

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //------------------Configure I2C as I/O ports (bit banged)-----------------
    P5DIR |= BIT2; //  LED 1 on Port 5.2 as output
    P5OUT |= BIT2; //  LED off initially

    P4DIR |= SCL; // SCL output port
    P4DIR |= SDA; // SDA output port

    P4OUT |= SCL; // Set SCL high as default
    P4OUT |= SDA; // Set SDA high ad default

    //-------SPI-------
    UCA1CTLW0 = UCSWRST;        //  Placing UCA1CTLW0 into SW reset
    UCA1CTLW0 |= UCSSEL__SMCLK; //  Clock set to SMCLK 1MHz
    UCA1BRW = 10;               //  Divide down to 100kHz
    UCA1CTLW0 |= UCSYNC;        //  Synchronous Mode
    UCA1CTLW0 |= UCMST;         //  SPI Master
    UCA1CTLW0 |= UCMSB;         //  MSB first
                                //  Settings from eUSCI - SPI Mode Excerpt from SLAU208 Texas Instruments
    //  SPI PORTS
    P4SEL1 &= ~SCLK;            //  P4.1 SCLK
    P4SEL0 |= SCLK;
    P4SEL1 &= ~MOSI;            //  P4.3 SIMO
    P4SEL0 |= MOSI;
    P4SEL1 &= ~MISO;            //  P4.2 SOMI
    P4SEL0 |= MISO;
    P4DIR |= NSS;               //  P4.0 NSS/STE (Manually Toggled)

    UCA1CTLW0 &= ~UCSWRST;      //  SPI operation Ready

    P4OUT |= NSS;               //  Turn off Chip Select

    delay(1500);

    //-------UART--------
    UCA0CTLW0 |= UCSWRST; //put A0 in software reset

    UCA0CTLW0 |= UCSSEL__ACLK; //Set clock

    //Clock dividing for 9600 baud rate on UART
    UCA0BRW = 3;
    UCA0MCTLW |= 0x9200;

    //Set the TX and RX channels to UART
    P1SEL1 &= ~BIT6;
    P1SEL0 |= BIT6;
    P1SEL1 &= ~BIT7;
    P1SEL0 |= BIT7;

    // for SD Card
    i2c_init_proc();
    SPIInit();                                          // Initialize SPI Ports

    PM5CTL0 &= ~LOCKLPM5; // Turn on GPIO

    //for UART
    UCA0CTLW0 &= ~UCSWRST; //Take A0 out of software reset

    for(i=0;i<25;i++){
      delay(10000);
    }

    // for SD Card
    sdCardInit();                                       // Send Initialization Commands to SD Card
    unsigned char dataIn[6];
    sendCommand(0x50, 0x200, 0xFF, dataIn);             // CMD 16 : Set Block Length
    unsigned char sd_buffer[512];                       // buffer for SD card
    unsigned int d;

    //set the LoRa Settings for TX
    SPI_tx(OPMODE_01, MODE_LORA_SLEEP); //Register Operation mode set to Sleep LoRa
    delay(100); // delay to ensure we are in sleep  mode
    SPI_tx(FIFO_TX_BASE_ADDR_0E, 0x00); //Set FIFO Tx Addr to 0x00
    SPI_tx(FIFO_RX_BASE_ADDR_0F, 0x00);
    SPI_tx(OPMODE_01, MODE_LORA_STBY); //Register Operation mode set to Standby LoRa
    SPI_tx(FR_MSB_06, 0xE4); //Set to 912 MHz MSB
    SPI_tx(FR_MID_07, 0x00); //Set to 912 MHz MidSB
    SPI_tx(FR_LSB_08, 0x00); //Set to 912 MHz LSB
    SPI_tx(POW_CONFIG_09, 0x8F); //Set Power Output
    SPI_tx(PREAMBLE_LEN_MSB_20, 0x00); //Set Preamble MSB
    SPI_tx(PREAMBLE_LEN_LSB_21, 0x08); //Set Preamble LSB
    SPI_tx(MODEM_CONFIG_1_1D, 0x72); //Modem Config 1 | 7 = bandwidth, 2 = coding rate + explicit header
    SPI_tx(MODEM_CONFIG_2_1E, 0xA4); //Modem Config 2 | A = Spreading factor, 0 = normal (single packet mode) + CRC enable (MUST BE ENABLED)
    SPI_tx(MODEM_CONFIG_3_26, 0x00); //Modem Config 3 | Low Data Rate Optimize, LNA
    SPI_tx(PAYLOAD_LEN_22, PAYLOAD_LEN); // Set Payload Len

    //-------Reset Temp Sensor------------

    i2c_tx_address(EXT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(0x2F); //Reset code
    i2c_stop();
    i2c_tx_address(EXT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(0x2F); //reset code
    i2c_stop();
    for(i=0; i<250; i++){}

    i2c_tx_address(INT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(TEMP_CONFIG);
    i2c_tx(0x80); //configure to 16-bit mode
    i2c_stop();
    i2c_tx_address(EXT_TEMP_ADDR, WRITE_MODE);
    i2c_tx(TEMP_CONFIG);
    i2c_tx(0x80); //configure to 16-bit mode
    i2c_stop();

    //read in the configuration registers, uncomment for debugging
    //temp_config_int = read_temp(INT_TEMP_ADDR, TEMP_CONFIG);
    //temp_config_ext = read_temp(EXT_TEMP_ADDR, TEMP_CONFIG);

    //---------Config Accelerometer--------------
    i2c_tx_address(ACCEL_ADDR, WRITE_MODE);
    i2c_tx(0x20); //write to the control register
    i2c_tx(0x17); //take out of power down mode and set to 10Hz generation of signal
    i2c_stop();

    //--------Config Pressure-------------------
    i2c_tx_pressure(0x1E); //reset

    for(i=0; i<275; i++){} //3ms delay

    //get pressure coeffs
    get_pressure_coeff();

    for(i=0; i<254; i++){}

    //init all values to 0 so we don't accidentally send funky data
    LORA_PAYLOAD.LORA_accel_x = 0;
    LORA_PAYLOAD.LORA_accel_y = 0;
    LORA_PAYLOAD.LORA_accel_z = 0;
    LORA_PAYLOAD.LORA_ext_temp = 0;
    LORA_PAYLOAD.LORA_int_temp = 0;
    LORA_PAYLOAD.LORA_pressure = 0;

    //--------Send UART config messages--------
	int j;
    for(j=0; j<5000; j++){}
	
	sendGPSMessOff(rmcOff, sizeof(rmcOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(vtgOff, sizeof(vtgOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(gsaOff, sizeof(gsaOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(gsvOff, sizeof(gsvOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(gllOff, sizeof(gllOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(ggaOff, sizeof(ggaOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(zdaOff, sizeof(zdaOff));
    for(j=0; j<2000; j++){}
    sendGPSMessOff(polReq, sizeof(polReq));
    for(j=0; j<2000; j++){}

    UCA0IE &= ~UCRXIE; //disable UART receive interrupt (do this here so we don't receive data too early)
    __enable_interrupt();


    delay(10000);

    while(1){

		//Uncomment to check that the airborne mode was set correctly
		/*for(m=0; m<8; m++){
			UCA0TXBUF = poll_airborne[m];
			for(j=0; j<150; j++){}
		}
		delay(5000);*/

		//get temperatures
		int_temp = read_temp(INT_TEMP_ADDR, TEMP_MSB);
		int_temp = int_temp << 8;
		int_temp = int_temp + read_temp(INT_TEMP_ADDR, TEMP_LSB);
		LORA_PAYLOAD.LORA_int_temp = int_temp;
		ext_temp = read_temp(EXT_TEMP_ADDR, TEMP_MSB);
		ext_temp = ext_temp << 8;
		ext_temp = ext_temp + read_temp(EXT_TEMP_ADDR, TEMP_LSB);
		LORA_PAYLOAD.LORA_ext_temp = ext_temp;
		for(i=0; i<10; i++){}

		//get accelerometer
		accel_x = read_accel(ACCEL_X_MSB);
		accel_x = accel_x << 8;
		accel_x = accel_x + read_accel(ACCEL_X_LSB);
		LORA_PAYLOAD.LORA_accel_x = accel_x;
		accel_y = read_accel(ACCEL_Y_MSB);
		accel_y = accel_y << 8;
		accel_y = accel_y + read_accel(ACCEL_Y_LSB);
		LORA_PAYLOAD.LORA_accel_y = accel_y;
		accel_z = read_accel(ACCEL_Z_MSB);
		accel_z = accel_z << 8;
		accel_z = accel_z + read_accel(ACCEL_Z_LSB);
		LORA_PAYLOAD.LORA_accel_z = accel_z;
		for(i=0; i < 10; i++){}

		//get pressure
		i2c_tx_pressure(GET_PRESSURE);
		for(i=0; i<900; i++){} //9ms delay
		i2c_tx_pressure(READ_PRESS);
		digital_press = i2c_rx_pressure();

		i2c_tx_pressure(GET_PRESS_TEMP);
		for(i=0; i<900; i++){} //9ms delay
		i2c_tx_pressure(READ_PRESS);
		digital_press_temp = i2c_rx_pressure();

		convert_pressure();
		LORA_PAYLOAD.LORA_pressure = pressure;

      

		gpsCnt = 0;
		gpsDone = 0;                                                                                                                        // Reset GPS Flag
		UCA0IE |= UCRXIE;                                                                                                                   // Enable GPS
		while(gpsDone != 1){}                                                                                                               // Wait for GPS to finish
		UCA0IE &= ~UCRXIE;                                                                                                                  // disable UART message receiving
		descrambleGPSData();
		delay(10000);

		//Send the character array of all the data to LoRa! Yay! Sending MSB first
        SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //set FIFO pointer to 0x00

        //Write the radiohead header
        P4OUT &= ~NSS;
        UCA1TXBUF = 0x80;           // Write + FIFO Transmit register (1 + 0000000)
        delay(SPI_DELAY);
        UCA1TXBUF = 0xFC;           // Header to
        delay(SPI_DELAY);
        UCA1TXBUF = 0XFB;           // Header from
        delay(SPI_DELAY);
        UCA1TXBUF = 0x5B;           // Header ID
        delay(SPI_DELAY);
        UCA1TXBUF = 0x5B;           // Header Key
        delay(SPI_DELAY);
        UCA1TXBUF = 0x5B;           // Start of transmission character
        delay(SPI_DELAY);

        //transmit sensor data
        SPI_tx_two_bytes(LORA_PAYLOAD.LORA_int_temp);
        SPI_tx_two_bytes(LORA_PAYLOAD.LORA_ext_temp);
        SPI_tx_two_bytes(LORA_PAYLOAD.LORA_accel_x);
        SPI_tx_two_bytes(LORA_PAYLOAD.LORA_accel_y);
        SPI_tx_two_bytes(LORA_PAYLOAD.LORA_accel_z);
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 24);
        delay(SPI_DELAY);
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 16);
        delay(SPI_DELAY);
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure >> 8);
        delay(SPI_DELAY);
        UCA1TXBUF = (LORA_PAYLOAD.LORA_pressure);
        delay(SPI_DELAY);
        UCA1TXBUF = (press_temp >> 24);
        delay(SPI_DELAY);
        UCA1TXBUF = (press_temp >> 16);
        delay(SPI_DELAY);
        UCA1TXBUF = (press_temp >> 8);
        delay(SPI_DELAY);
        UCA1TXBUF = (press_temp);
        delay(SPI_DELAY);
        SPI_tx_two_bytes(packet_tx_cnt);
        //transmit GPS data
        for(i=0; i <sizeof(gpsSunnySide); i++){
			UCA1TXBUF = gpsSunnySide[i];
			delay(SPI_DELAY);
        }
        P4OUT |= NSS; //SPI chip select

        SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //Set Pointer to FIFO LoRa back to 0x00

        SPI_tx(OPMODE_01, MODE_LORA_TX); //Set to Transmit LoRa

        packet_tx_cnt++;


        //Send the character array of all the data to LoRa! Yay! Sending MSB first
        SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //set FIFO pointer to 0x00

        //Write the radiohead header
        P4OUT &= ~NSS;
        UCA1TXBUF = 0x80;           // Write + FIFO Transmit register (1 + 0000000)
        delay(SPI_DELAY);
        UCA1TXBUF = 0xFC;           // Header to
        delay(SPI_DELAY);
        UCA1TXBUF = 0XFB;           // Header from
        delay(SPI_DELAY);
        UCA1TXBUF = 0x5B;           // Header ID
        delay(SPI_DELAY);
        UCA1TXBUF = 0x5B;           // Header Key
        delay(SPI_DELAY);
        UCA1TXBUF = 0x7B;           // Start of transmission character
        delay(SPI_DELAY);

        UCA1TXBUF = 0x7C;
        delay(SPI_DELAY);
        P4OUT |= NSS; //SPI chip select

        SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //Set Pointer to FIFO LoRa back to 0x00

        SPI_tx(OPMODE_01, MODE_LORA_TX); //Set to Transmit LoRa

        //Send the character array of all the data to LoRa! Yay! Sending MSB first
        SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //set FIFO pointer to 0x00

        //Write the radiohead header
        P4OUT &= ~NSS;
        UCA1TXBUF = 0x80;           // Write + FIFO Transmit register (1 + 0000000)
        delay(SPI_DELAY);
        UCA1TXBUF = 0xFC;           // Header to
        delay(SPI_DELAY);
        UCA1TXBUF = 0XFB;           // Header from
        delay(SPI_DELAY);
        UCA1TXBUF = 0x5B;           // Header ID
        delay(SPI_DELAY);
        UCA1TXBUF = 0x5B;           // Header Key
        delay(SPI_DELAY);
        UCA1TXBUF = 0x28;           // Start of transmission character
        delay(SPI_DELAY);

        UCA1TXBUF = 0x29;
        delay(SPI_DELAY);
        P4OUT |= NSS; //SPI chip select

        SPI_tx(FIFO_ADDR_PTR_0D, 0x00); //Set Pointer to FIFO LoRa back to 0x00

        SPI_tx(OPMODE_01, MODE_LORA_TX); //Set to Transmit LoRa

        if(address_error != 0){
			address_cnt = address_last;
			address_error = 0;
        }
        address_last = address_cnt;
        for(d = 0; d < sizeof(sd_buffer); d++){
			sd_buffer[d] = 0;
        }
        sd_buffer[0] = LORA_PAYLOAD.LORA_pressure & 0xFF;
        sd_buffer[1] = (LORA_PAYLOAD.LORA_pressure & 0xFF00) >> 8;
        sd_buffer[2] = (LORA_PAYLOAD.LORA_pressure & 0xFF0000) >> 16;
        sd_buffer[3] = (LORA_PAYLOAD.LORA_pressure & 0xFF000000) >> 24;
        sd_buffer[4] = LORA_PAYLOAD.LORA_int_temp & 0xFF;
        sd_buffer[5] = (LORA_PAYLOAD.LORA_int_temp & 0xFF00) >> 8;
        sd_buffer[6] = LORA_PAYLOAD.LORA_ext_temp & 0xFF;
        sd_buffer[7] = (LORA_PAYLOAD.LORA_ext_temp & 0xFF00) >> 8;
        sd_buffer[8] = LORA_PAYLOAD.LORA_accel_x & 0xFF;
        sd_buffer[9] = (LORA_PAYLOAD.LORA_accel_x & 0xFF00) >> 8;
        sd_buffer[10] = LORA_PAYLOAD.LORA_accel_y & 0xFF;
        sd_buffer[11] = (LORA_PAYLOAD.LORA_accel_y & 0xFF00) >> 8;
        sd_buffer[12] = LORA_PAYLOAD.LORA_accel_z & 0xFF;
        sd_buffer[13] = (LORA_PAYLOAD.LORA_accel_z & 0xFF00) >> 8;
        for(d = sizeof(gpsSunnySide); d > 0; d--){
			sd_buffer[d + 14] = gpsSunnySide[d];
        }
        sendData(address_cnt, sd_buffer);
        address_cnt++;
        __delay_cycles(10000);     // may need to increase (50000 originally)
        sendCommand(0x4D, 0, 0, dataIn);
        sendCommand(0x4D, 0, 0, dataIn);
        __delay_cycles(10000);    // this one too

        //long delay because we don't need to send that often and we wanna get the GPS data
        for(i=0;i<20;i++){
			delay(10000);
        }

    }
    //return 0; //never reached, left here if needed in the future
}

//UART receive interrupt
#pragma vector=EUSCI_A0_VECTOR
__interrupt void EUSCI_A0_RX_ISR(void)
{
    gpsReceiveMessage[gpsCnt] = UCA0RXBUF;
    if(gpsCnt == sizeof(gpsReceiveMessage)-1)
    {
        gpsCnt = 0;
        gpsDone = 1;
    }
    else
    {
        gpsCnt++;
    }
}
