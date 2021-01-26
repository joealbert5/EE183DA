/*
  Wireless Servo Control, with ESP as Access Point

  Usage: 
    Connect phone or laptop to "ESP_XXXX" wireless network, where XXXX is the ID of the robot
    Go to 192.168.4.1. 
    A webpage with four buttons should appear. Click them to move the robot.

  Installation: 
    In Arduino, go to Tools > ESP8266 Sketch Data Upload to upload the files from ./data to the ESP
    Then, in Arduino, compile and upload sketch to the ESP

  Requirements:
    Arduino support for ESP8266 board
      In Arduino, add URL to Files > Preferences > Additional Board Managers URL.
      See https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

    Websockets library
      To install, Sketch > Include Library > Manage Libraries... > Websockets > Install
      https://github.com/Links2004/arduinoWebSockets
    
    ESP8266FS tool
      To install, create "tools" folder in Arduino, download, and unzip. See 
      https://github.com/esp8266/Arduino/blob/master/doc/filesystem.md#uploading-files-to-file-system

  Hardware: 
  * NodeMCU Amica DevKit Board (ESP8266 chip)
  * Motorshield for NodeMCU 
  * 2 continuous rotation servos plugged into motorshield pins D1, D2
  * Ultra-thin power bank 
  * Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#include <Servo.h>
#include "debug.h"
#include "file.h"
#include "server.h"
#include "lasermag.h"
#include "kalman.h"

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;

char dxn = 'X';
double q_processNoise [4] = {.125,.125,.125,.125};
double r_sensorNoise [4] = {6,6,6,6};
double p_estimateError [4] = {1000,1000,1,1};
double x_initVal [4] = {10,10,0,0};
Kalman kalman(q_processNoise,r_sensorNoise,p_estimateError,x_initVal);

// WiFi AP parameters
char ap_ssid[13];
char* ap_password = "";

// WiFi STA parameters
char* sta_ssid = 
  "...";
char* sta_password = 
  "...";

char* mDNS_name = "paperbot";

String html;
String css;

float bias[3] = {0,0,0};
float scale[3] = {0,0,0};
float mxbias = 0;
float mybias = 0;
float mzbias = 0;

// max coords of environment (the box)
int16_t xmax = 55; 
int16_t ymax = 90;

void setup() {
    setupPins();

    sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());
    Serial.println(ESP.getChipId());

    for(uint8_t t = 4; t > 0; t--) {
        Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
        Serial.flush();
        LED_ON;
        delay(500);
        LED_OFF;
        delay(500);
    }
    LED_ON;
    //setupSTA(sta_ssid, sta_password);
    setupAP(ap_ssid, ap_password);
    LED_OFF;

    setupFile();
    html = loadFile("/controls.html");
    css = loadFile("/style.css");
    registerPage("/", "text/html", html);
    registerPage("/style.css", "text/css", css);

    setupHTTP();
    setupWS(webSocketEvent);
    setupMagAndSensor();
    //setupMDNS(mDNS_name);

    stop();
}

void loop() {
    wsLoop();
    httpLoop();
    sendCoords(0);
}


//
// Movement Functions //
//

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  LED_OFF;
}

void forward() {
  DEBUG("forward");
  drive(0, 180);
}

void backward() {
  DEBUG("backward");
  drive(180, 0);
}

void left() {
  DEBUG("left");
  drive(0, 0);
}

void right() {
  DEBUG("right");
  drive(180, 180);
}

int16_t convertX(int16_t x){
  return .09302*x + 3; // in cm
}

int16_t convertY(int16_t x){
  return .10312*x + 2.9; // in cm
}

float convertDeg(float angle)
{
  return angle*180/PI;
}

void coordCalc(int16_t &x, int16_t &y, float angle)
{
  if (angle >= 0 && angle < (PI/2))
  { 
    x = x*cos(angle);
    y = y*cos(angle);
  }
  else if (angle >= (PI/2) && angle < PI)
  { 
    x = y*cos(angle-(PI/2));
    y = (ymax-x)*cos(angle-(PI/2));
  }
  else if (angle >= PI && angle < (3*PI/2))
  { 
    x = (xmax-x)*cos(angle-PI);
    y = (ymax-y)*cos(angle-PI);
  }
  else 
  {
    x = (xmax-y)*cos(angle-(3*PI/2));
    y = x*cos(angle-(3*PI/2));
  }
}

float headingCalc(int16_t magx, int16_t magy)
{
  float heading = atan2(magx, magy);
  if(heading < 0)
    heading += 2*PI;
    
  return heading;
/*
  if (headingD < 7.5)
    headingD = 0;
  else if (headingD >= 7.5 && < 22.5)
    headingD = 15;
  else if (headingD >= 22.5 && < 37.5)
    headingD = 30;
  else if (headingD >= 37.5 && < 52.5)
    headingD = 45;
  else if (headingD >= 52.5 && < 67.5)
    headingD = 60;
  else if (headingD >= 67.5 && < 82.5)
    headingD = 75;
  else
    headingD = 90;
    */ 
}

void calibrate() //function only used to determine magnetometer bias
{
   int16_t mag_max[3] = { -32767, -32767, -32767};
   int16_t mag_min[3] = {32767, 32767, 32767};

   int count = 0;

   while (count < 2000)
   {
    uint8_t Mag1[7];
    I2Cread(MAG_ADDRESS, 0x03, 7, Mag1);
    int16_t mxb = (Mag1[1] << 8 | Mag1[0]);
    int16_t myb = (Mag1[3] << 8 | Mag1[2]);
    int16_t mzb = (Mag1[5] << 8 | Mag1[4]);

    if (mxb > mag_max[0])
      mag_max[0] = mxb;
    if (mxb < mag_min[0])
      mag_min[0] = mxb;

    if (myb > mag_max[1])
      mag_max[1] = myb;
    if (myb < mag_min[1])
      mag_min[1] = myb;

    if (mzb > mag_max[2])
      mag_max[2] = mzb;
    if (mzb < mag_min[2])
      mag_min[2] = mzb;

    count++;    
   }

   mxbias = ((float) (mag_max[0] + mag_min[0]))/2;
   mybias = ((float) (mag_max[1] + mag_min[1]))/2;
   mzbias = ((float) (mag_max[2] + mag_min[2]))/2;  
}

void sendCoords(uint8_t id){
  char buff [50];
  int16_t* p = scanXY(-157,-53,bias[2]);
  //printArr(p, *p);
  double sensX = (double) *(p + 1);
  double sensY = (double) *(p + 2);
  //float* theta = printMag();
  double sensTx = (double) *(p + 3);
  double sensTy = (double) *(p + 4);
  double measurements [4] = {sensX, sensY, sensTx, sensTy};
  //printArr((uint16_t)theta, *theta);
  double* filteredM = kalman.getFilteredValue(measurements, dxn);
  int16_t filX = (int16_t) *(filteredM + 0);
  int16_t filY = (int16_t) *(filteredM + 1);
  int16_t filTx = (int16_t) *(filteredM + 2);
  int16_t filTy = (int16_t) *(filteredM + 3);
  float headingRad = headingCalc(filTx,filTy);
  float headingDeg = convertDeg(headingRad);
  // remove coordX and coordY and make it an array
  int16_t conFilX = convertX(filX);
  int16_t conFilY = convertY(filY);
  //coordCalc(conFilX,conFilY,headingRad);
  //int16_t coordX = coordCalc(convertX(filX),headingRad);
  //int16_t coordY = coordCalc(convertY(filY),headingRad);
  //calibrate();
  //sprintf (buff, "h: %f mx: %d my: %d", headingDeg, filTx, filTy);
  sprintf (buff, "x: %d y: %d h: %f", conFilX, conFilY, headingDeg);
  //fsprintf (buff, "x: %d y: %d dxn: %c", filX, filY, dxn);
  
  //Serial.println(buff);
  wsSend(id, buff);
}

void instruxToDrive(char c){
  dxn = c;
  if(c == 'F') 
    forward();
  else if(c == 'B')
    backward();
  else if(c == 'L')
    left();
  else if(c == 'R')
    right();
  sendCoords(0);
}

//
// Setup //
//

void setupPins() {
    // setup Serial, LEDs and Motors
    Serial.begin(115200);
    DEBUG("Started serial.");

    pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
    LED_OFF;                     //Turn off LED
    DEBUG("Setup LED pin.");

    servo_left.attach(SERVO_LEFT);
    servo_right.attach(SERVO_RIGHT);
    DEBUG("Setup motor pins");
}

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            DEBUG("Web socket disconnected, id = ", id);
            break;
        case WStype_CONNECTED: 
        {
            // IPAddress ip = webSocket.remoteIP(id);
            // Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", id, ip[0], ip[1], ip[2], ip[3], payload);
            DEBUG("Web socket connected, id = ", id);

            // send message to client
            wsSend(id, "Connected to ");
            wsSend(id, ap_ssid);
            break;
        }
        case WStype_BIN:
            DEBUG("On connection #", id)
            DEBUG("  got binary of length ", length);
            for (int i = 0; i < length; i++)
              DEBUG("    char : ", payload[i]);

            if (payload[0] == '~') 
              drive(180-payload[1], payload[2]);

        case WStype_TEXT:
            DEBUG("On connection #", id)
            DEBUG("  got text: ", (char *)payload);

            if (payload[0] == '#') {
                if(payload[1] == '#'){
                  char instrux;
                  for (int i = 2; payload[i] != '@'; i++){
                    instrux = (char) payload[i];
                    instruxToDrive(instrux);
                  }
                  drive(90,90);
                }
                else if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                }
                else if(payload[1] == 'F'){ 
                  forward();
                  dxn = payload[1];
                }
                else if(payload[1] == 'B') {
                  backward();
                  dxn = payload[1];
                }
                else if(payload[1] == 'L') {
                  left();
                  dxn = payload[1];
                }
                else if(payload[1] == 'R') {
                  right();
                  dxn = payload[1];
                }
                //ecode H = UL, I = UR, J=DL, K=DR
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') {
                    servo_left_ctr -= 1;
                    dxn = 'H';
                    sendCoords(id);
                  }
                  else if(payload[2] == 'R') {
                    servo_right_ctr += 1;
                    dxn = 'I';
                    sendCoords(id);
                  }
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') {
                    servo_left_ctr += 1;
                    dxn = 'J';
                    sendCoords(id);
                  }
                  else if(payload[2] == 'R') {
                    servo_right_ctr -= 1;
                    dxn = 'K';
                    sendCoords(id);
                  }
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else{ 
                  dxn = 'X';
                  stop();
                }
            }

            break;
    }
}
