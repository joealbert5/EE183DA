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

#define SDA_PORT 14
#define SCL_PORT 12
//#define HIGH_ACCURACY
#define HIGH_SPEED
//#define LONG_RANGE
VL53L0X sensor;
VL53L0X sensor2;

uint16_t* scanXY(){
  //Serial.print(sensor.readRangeSingleMillimeters());
  uint16_t initX1 = sensor.readRangeSingleMillimeters();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  //Serial.print(sensor2.readRangeSingleMillimeters());
  uint16_t initY1 = sensor2.readRangeSingleMillimeters();
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  uint16_t ret[3] = {3, initX1, initY1};
  Serial.println(" ");
  return ret;
}

void printArr(uint16_t arr[], int len){
  //start at index 1 because index 0 is the size of the array
  for(int i = 1; i < len; i++){
    Serial.print(*(arr + i));
    Serial.print(",");
  }
  //Serial.println("printArr values");
}

void setup()
{

  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT,SCL_PORT);

  Serial.begin (115200);

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

  delay(3000);
}

void loop()
{
  /*Serial.print("Lidar 1 range(mm): ");
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.print("  Lidar 2 range(mm): ");
  Serial.println(sensor2.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.println(" TIMEOUT"); }*/
  uint16_t* p = scanXY();
  int len = *p;
  printArr(p, len);
  
  //Serial.print("finished loop");
}
