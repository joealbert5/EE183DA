/*VL53L0X Duel Sensor Test Code
 * By: Kevin Pololu and steinerlein
 * Original Source: https://github.com/pololu/vl53l0x-arduino/issues/1
 * Edited for EE183DA by Nathan Pilbrough
 * 
 * Description: Basic code to test the functionality of two
 * VL53L0X sensors operating at the same time. Refer to the 
 * startup guide on CCLE for more information
 */
#ifndef __LASERMAG_H
#define __LASERMAG_H

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

uint8_t readByte(uint8_t address, uint8_t subAddress);

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);

void initAK8963(float * destination);

void magcalMPU9250(float * dest1, float * dest2);

int16_t* scanXY(float mxbias, float mybias, float mzbias);

uint16_t* testing();

void printArrD(double arr[], int len);

float* printMag();

void printArr(uint16_t arr[], int len);

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data);

void setupMagAndSensor();
#endif
