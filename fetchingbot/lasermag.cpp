/*VL53L0X Duel Sensor Test Code
 * By: Kevin Pololu and steinerlein
 * Original Source: https://github.com/pololu/vl53l0x-arduino/issues/1
 * Edited for EE183DA by Nathan Pilbrough
 * 
 * Description: Basic code to test the functionality of two
 * VL53L0X sensors operating at the same time. Refer to the 
 * startup guide on CCLE for more information
 */
#include <Wire.h>
#include <VL53L0X.h>
#include "lasermag.h"

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L   0x03  // data
#define AK8963_XOUT_H  0x04
#define AK8963_YOUT_L  0x05
#define AK8963_YOUT_H  0x06
#define AK8963_ZOUT_L  0x07
#define AK8963_ZOUT_H  0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

//#define RANGE_SENSORS
#define SDA_PORT 2    
#define SCL_PORT 0    
//#define HIGH_ACCURACY
#define HIGH_SPEED
//#define LONG_RANGE
VL53L0X sensor;
VL53L0X sensor2;

float magCalibration[3] = {0, 0, 0};
float mRes = 10.*4912. / 8190.;
long int cpt = 0;

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}


void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0 << 4 | 0x06); // Set magnetometer data resolution and sample ODR
  delay(10);
}

void magcalMPU9250(float * dest1, float * dest2)
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);


  // shoot for ~fifteen seconds of mag data
  //if(MPU9250Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  //if(MPU9250Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  sample_count = 1500;
  for (ii = 0; ii < sample_count; ii++) {
    // MPU9250readMagData(mag_temp);// Read the mag data
    uint8_t Mag1[7];
    I2Cread(MAG_ADDRESS, 0x03, 7, Mag1);

    // Create 16 bits values from 8 bits data

    // Magnetometer
    int16_t mxb = (Mag1[1] << 8 | Mag1[0]);
    int16_t myb = (Mag1[3] << 8 | Mag1[2]);
    int16_t mzb = (Mag1[5] << 8 | Mag1[4]);

    mag_temp[0] = mxb;
    mag_temp[1] = myb;
    mag_temp[2] = mzb;

    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    //if(MPU9250Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    //if(MPU9250Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
    delay(12);
  }


  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts


  dest1[0] = (float) mag_bias[0] * magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * magCalibration[1];
  dest1[2] = (float) mag_bias[2] * magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad / (mag_scale[0]);
  dest2[1] = avg_rad / (mag_scale[1]);
  dest2[2] = avg_rad / (mag_scale[2]);

  Serial.println("Mag Calibration done!");
}

int16_t* scanXY(float mxbias, float mybias, float mzbias){
  //Serial.print(sensor.readRangeSingleMillimeters());
  //Serial.println("inside scanxy");
  #if defined RANGE_SENSORS
    //Serial.println("inside if range_sensors");
    uint16_t initX1 = sensor.readRangeSingleMillimeters();
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    
    //Serial.print(sensor2.readRangeSingleMillimeters());
    uint16_t initY1 = sensor2.readRangeSingleMillimeters();
    if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  #else
    //Serial.println("inside else not range sensors");
    uint16_t initX1 = 69;
    uint16_t initY1 = 69;
  #endif

// :::  Magnetometer ::: 

  #if defined RANGE_SENSORS

    // Request first magnetometer single measurement
    I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
    
    // Read register Status 1 and wait for the DRDY: Data Ready
    
    uint8_t ST1;
    do
    {
      I2Cread(MAG_ADDRESS,0x02,1,&ST1);
    }
    while (!(ST1&0x01));
  
    // Read magnetometer data  
    uint8_t Mag[7];  
    I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
    // Create 16 bits values from 8 bits data
    
    // Magnetometer
    int16_t mx=(Mag[1]<<8 | Mag[0]);
    int16_t my=(Mag[3]<<8 | Mag[2]);
    int16_t mz=(Mag[5]<<8 | Mag[4]);
  
    float mxx = ((float)mx)- mxbias;
    float myy = ((float)my)- mybias; 
    float mzz = ((float)mz)- mzbias; 
  
    int16_t magnetx = (int16_t) (mxx);
    int16_t magnety = (int16_t) (myy);
    int16_t magnetz = (int16_t) (mzz);
  
    int16_t x1 = (int16_t) initX1;
    int16_t x2 = (int16_t) initY1;
    
    
    //Serial.println("before ret array");
    int16_t ret[6] = {4, x1, x2, magnetx, magnety, magnetz};
    Serial.println(" ");
    Serial.print(magnetx);
    Serial.print(" ");
    Serial.print(magnety);
    Serial.print(" ");
    Serial.print(magnetz);
    Serial.println(" ");
    Serial.print(x1);
    Serial.print(" ");
    Serial.print(x2);
  #else
    int16_t ret[6] = {4, 1, 1, 1, 1, 1};
  #endif
  //Serial.println(" ");
  //Serial.println("about to return ret");
  return ret;
}

uint16_t* testing()
{
  uint16_t x[4] = {1,2,3,4};
  return x;
}

float* printMag(){
    // :::  Magnetometer ::: 

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  
  // Read register Status 1 and wait for the DRDY: Data Ready
  
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t Mag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  float mxbias = -196.36;
  float mybias = -60.16;
  float mzbias = -41.06;
  
  int16_t mx=(Mag[1]<<8 | Mag[0]);
  int16_t my=(Mag[3]<<8 | Mag[2]);
  int16_t mz=(Mag[5]<<8 | Mag[4]);

  float mxx = ((float)mx)- mxbias;
  float myy = ((float)my)- mybias; 
  float mzz = ((float)mz)- mzbias; 
  
  
  float heading = atan2(mxx, myy);
  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404 
  // Find yours here: http://www.magnetic-declination.com/
  
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //float declinationAngle = 0.0404;
  //heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Convert radians to degrees for readability.
  
  float headingDegrees = (heading * 180/PI); 
     
  float ret[3] = {mxx, myy, mzz};
  return ret;
}

void printArrD(double arr[], int len){
  for(int i = 0; i < len; i++){
    Serial.print(*(arr + i));
    Serial.print(",");
  }
  //Serial.println("printArr values");
}

void printArr(uint16_t arr[], int len){
  //start at index 1 because index 0 is the size of the array
  for(int i = 1; i < len; i++){
    Serial.print(*(arr + i));
    Serial.print(",");
  }
  //Serial.println("printArr values");
}

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void setupMagAndSensor(){
  #if defined RANGE_SENSORS
    pinMode(D0, OUTPUT);
    pinMode(D8, OUTPUT);
  #endif
  //pinMode(SDA_PORT, OUTPUT); not needed?
  //pinMode(SCL_PORT, OUTPUT);
  delay(500);
  Wire.begin(SDA_PORT,SCL_PORT);


  //Serial.begin (115200);
  
  // start of added code
   I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
   I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  //initAK8963(magCalibration);
  // end of added code

  #if defined RANGE_SENSORS
    digitalWrite(D3, HIGH);
    delay(150);
    Serial.println("00");
    
    sensor.init(true);
    Serial.println("01");
    delay(100);
    sensor.setAddress((uint16_t)22);
  
    digitalWrite(D4, HIGH);
    delay(150);
    sensor2.init(true);
    Serial.println("03");
    delay(100);
    sensor2.setAddress((uint16_t)25);
    Serial.println("04");
  #endif

  Serial.println("addresses set");
  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");

  #if defined RANGE_SENSORS

    #if defined HIGH_SPEED
      // reduce timing budget to 20 ms (default is about 33 ms)
      sensor.setMeasurementTimingBudget(20000);
      sensor2.setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
      // increase timing budget to 200 ms
      sensor.setMeasurementTimingBudget(200000);
      sensor2.setMeasurementTimingBudget(200000);
    #endif
  
    #if defined LONG_RANGE
      // lower the return signal rate limit (default is 0.25 MCPS)
      sensor.setSignalRateLimit(0.1);
      sensor2.setSignalRateLimit(0.1);
      // increase laser pulse periods (defaults are 14 and 10 PCLKs)
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
      sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
      sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif

  #endif

  delay(3000);
}
