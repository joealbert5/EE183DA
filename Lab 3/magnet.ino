/*MPU9250 Sensor Test Code
   By: stevenvo
   Original Source: https://github.com/stevenvo/mpuarduino/blob/master/mpuarduino.ino
   Edited for EE183DA by Nathan Pilbrough

   Description: Basic code to test the functionality of the
   MPU9250 sensor. Refer to the startup guide on CCLE for
   more information. NOTE: this is meant to help confirm
   communication with the sensor, calibration is still required
*/
#include <Wire.h>

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

#define SDA_PORT D5
#define SCL_PORT D6

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

float magCalibration[3] = {0, 0, 0};
float mRes = 10.*4912. / 8190.;
float bias[3] = {0, 0, 0};
float what[3] = {0, 0, 0};
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
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
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



// Initializations
void setup()
{

  // Arduino initializations
  Wire.begin(SDA_PORT, SCL_PORT);
  Serial.begin(115200);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

  initAK8963(magCalibration);
}

void printBias()
{
  magcalMPU9250(bias, what);

  Serial.print(bias[0]);
  Serial.print(" ");
  Serial.print(bias[1]);
  Serial.print(" ");
  Serial.print(bias[2]);
  Serial.print(" ");
  Serial.println("");
  Serial.print(what[0]);
  Serial.print(" ");
  Serial.print(what[1]);
  Serial.print(" ");
  Serial.print(what[2]);
  Serial.print(" ");
}

void printReading()
{

  // Read magnetometer data
  uint8_t Mag[7];
  I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

  // Create 16 bits values from 8 bits data
  
  // Magnetometer
  float mxbias = 43.5;
  float mybias = 11.8;
  float mzbias = -21.67;

  int16_t mx = (Mag[1] << 8 | Mag[0]);
  int16_t my = (Mag[3] << 8 | Mag[2]);
  int16_t mz = (Mag[5] << 8 | Mag[4]);

  float mxx = ((float)mx) - mxbias;
  float myy = ((float)my) - mybias;
  float mzz = ((float)mz) - mzbias;

  float heading = atan2(mxx, myy);

  // Once you have your heading, you must then add your 'Declination Angle',
  // which is the 'Error' of the magnetic field in your location. Mine is 0.0404
  // Find yours here: http://www.magnetic-declination.com/

  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0404;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  float headingDegrees = (heading * 180 / PI);

  Serial.print("\rHeading:\t");
  Serial.print(heading);
  Serial.print(" Radians   \t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \t");

  Serial.print ("Magnetometer readings:");
  Serial.print ("\tMx:");
  Serial.print (mxx);
  Serial.print ("\tMy:");
  Serial.print (myy);
  Serial.print ("\tMz:");
  Serial.print (mzz);
  Serial.println ("\t");
}

// Main loop, read and display data
void loop()
{
  // _______________
  // ::: Counter :::

  // Display data counter
  //Serial.print (cpt++, DEC);
  //Serial.print ("\t");

  // _____________________
  // :::  Magnetometer :::

  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01);

  // Read register Status 1 and wait for the DRDY: Data Ready

  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
  }
  while (!(ST1 & 0x01));
  
  // 0 for bias mode, 1 for reading mode
  bool printMode = 0;

  if (printMode == 0)
    printBias();
  else
    printReading();
  
  // End of line
  delay(100);
}








