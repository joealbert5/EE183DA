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
    NodeMCU Amica DevKit Board (ESP8266 chip)
    Motorshield for NodeMCU
    2 continuous rotation servos plugged into motorshield pins D1, D2
    Ultra-thin power bank
    Paper chassis

*/

#include <Arduino.h>

#include <Hash.h>
#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ESP8266mDNS.h>

#define    RANGE_SENSORS    //update in lasermag.cpp too
#define    PIXY_CAMERA

#include <Servo.h>
#include "debug.h"
#include "file.h"
#include "server.h"
#include "lasermag.h"
#include "kalman.h"
#if defined PIXY_CAMERA
  #include "pixycam.h"
  #include "ballsearch2.h"
  #include "ballapproach.h"
#endif

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

const int SERVO_GRAB = D7;
const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_grab;
Servo servo_left;
Servo servo_right;
bool grab = 0;
int servo_left_ctr = 90;
int servo_right_ctr = 90;

#if defined PIXY_CAMERA
  Ballsearch ballsearch(make_tuple(0,0));
#endif
double q_processNoise [4] = {.125,.125,.125,.125};
double r_sensorNoise [4] = {36,6,6,6};
double p_estimateError [4] = {100,1000,1,1};
double x_initVal [4] = {0,0,0,0};
Kalman kalman(q_processNoise,r_sensorNoise,p_estimateError,x_initVal);


char dxn = 'X';

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

float bias[3] = {0, 0, 0};
float scale[3] = {0, 0, 0};
float mxbias = 0;
float mybias = 0;
float mzbias = 0;

// max coords of environment (the box)
int16_t xmax = 55;
int16_t ymax = 90;

int16_t leftSensor = 0;
int16_t rightSensor = 0;

float sendCoords(uint8_t id, int32_t area = 0);
float getHeading(bool sendToWebApp = false, uint8_t id = 0, int32_t area = 0, bool calibrate = false);

void setup() {
  setupPins();    //sets up D0, 2 servo motors, serial port

  sprintf(ap_ssid, "ESP_%08X", ESP.getChipId());
  Serial.println(ESP.getChipId());

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush();
    //LED_ON;
    delay(500);
    //LED_OFF;
    delay(500);
  }
  //LED_ON;
  //setupSTA(sta_ssid, sta_password);
  setupAP(ap_ssid, ap_password);
  //LED_OFF;

  setupFile();
  html = loadFile("/controls.html");
  css = loadFile("/style.css");
  registerPage("/", "text/html", html);
  registerPage("/style.css", "text/css", css);

  setupHTTP();
  setupWS(webSocketEvent);
  setupMagAndSensor();
  
  #if defined PIXY_CAMERA
    setupPixy();
    
  #endif
  //setupMDNS(mDNS_name);

  stop();
}

void loop() {
  //Serial.println("entered loop");
  wsLoop();
  //Serial.println("passed wsloop");
  httpLoop();
  //Serial.println("passed httploop");
  int32_t area = 0;
  #if defined PIXY_CAMERA
    //Serial.println("entered if defined");
    area = scanBlocks();
    double areaA[4] = {area,0,0,0};
    area = (kalman.getFilteredValue(areaA, 'X'))[0];
    
    //Serial.println("passed scanBlocks");
    
  #endif
  sendCoords(0, area);
  //Serial.println("passed sendCoords");
  //Serial.println("finished loop");
  //obstacleAvoid();
}


//
// Movement Functions //
//

int MIN_DIS = 15;
int CLEAR_DIS = 20;

bool obstacleCheck()
{
  getHeading();
  if (leftSensor <= MIN_DIS || rightSensor <= MIN_DIS)
    return true;
  else
    return false;

  return false;
}

void obstacleAvoid()
{
  getHeading();
  if (leftSensor > MIN_DIS && rightSensor > MIN_DIS)
    return;
  else if (leftSensor <= MIN_DIS && rightSensor > MIN_DIS)
  {
    while (leftSensor < CLEAR_DIS)
    {
      right();
      getHeading();
    }
  }
  else if (rightSensor <= MIN_DIS && leftSensor > MIN_DIS)
  {
    while (rightSensor < CLEAR_DIS)
    {
      left();
      getHeading();
    }
  }
  else if (leftSensor <= MIN_DIS && rightSensor <= MIN_DIS)
  {
    float current = sendCoords(0);
    if (sendCoords(0) > current - 89)
      right();
  }
}

void drive(int left, int right) {
  servo_left.write(left);
  servo_right.write(right);
}

void stop() {
  DEBUG("stop");
  drive(servo_left_ctr, servo_right_ctr);
  //LED_OFF;
}

void forward() {
  //DEBUG("forward");
  drive(180, 0);
}

void backward() {
  DEBUG("backward");
  drive(0, 180);
}

void left() {
  DEBUG("left");
  drive(0, 0);
}

void right() {
  DEBUG("right");
  drive(180, 180);
}

void rightSlow(){
  drive(110, 110);
}

void leftSlow(){
  drive(70, 70);
}

int16_t convertX(int16_t x) {
  return .09302 * x + 3; // in cm
}

int16_t convertY(int16_t x) {
  return .10312 * x + 2.9; // in cm
}

float convertDeg(float angle)
{
  return angle * 180 / PI;
}

void coordCalc(int16_t &x, int16_t &y, float angle)
{
  if (angle >= 0 && angle < (PI / 2))
  {
    x = x * cos(angle);
    y = y * cos(angle);
  }
  else if (angle >= (PI / 2) && angle < PI)
  {
    x = y * cos(angle - (PI / 2));
    y = (ymax - x) * cos(angle - (PI / 2));
  }
  else if (angle >= PI && angle < (3 * PI / 2))
  {
    x = (xmax - x) * cos(angle - PI);
    y = (ymax - y) * cos(angle - PI);
  }
  else
  {
    x = (xmax - y) * cos(angle - (3 * PI / 2));
    y = x * cos(angle - (3 * PI / 2));
  }
}

float headingCalc(int16_t magx, int16_t magy)
{
  float heading = atan2(magx, magy);
  if (heading < 0)
    heading += 2 * PI;
  return heading;
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

  mxbias = ((float) (mag_max[0] + mag_min[0])) / 2;
  mybias = ((float) (mag_max[1] + mag_min[1])) / 2;
  mzbias = ((float) (mag_max[2] + mag_min[2])) / 2;
}

int16_t minX = 200;
int16_t maxX = -200;
int16_t minY = 200;
int16_t maxY = -200;

int16_t findMinMax(int16_t x, int16_t y)
{
  if (x > maxX)
    maxX = x;
  if (x < minX)
    minX = x;
  
  if (y > maxY)
    maxY = y;
  if (y < minY)
    minY = y;
}

int MX_BIAS = 37;
int MY_BIAS = -16;

float getHeading(bool sendToWebApp, uint8_t id, int32_t area, bool calibrate){
  char buff [50];
  Serial.println(" ");
  int16_t* p = scanXY(MX_BIAS, MY_BIAS, bias[2]);
  double sensX = (double) * (p + 1);
  double sensY = (double) * (p + 2);
  double sensTx = (double) * (p + 3);
  double sensTy = (double) * (p + 4);
  int16_t filX = (int16_t) sensX;
  int16_t filY = (int16_t) sensY;
  int16_t filTx = (int16_t) sensTx;
  int16_t filTy = (int16_t) sensTy;
  findMinMax(sensTx, sensTy);
  //<<<<<<<<auto update biases
  int newXbias = (maxX + minX)/2;
  int newYbias = (maxY + minY)/2;
  if (calibrate){
    MX_BIAS += newXbias;  //might have to be += instead of -=
    MY_BIAS += newYbias;  //this might have to only be called when call spins 360°, such as in scan2(), not totally sure
  }
  printWebApp(String(newXbias) + " " + String(newYbias));
  //>>>>>>>end autoupdate
  float headingRad = headingCalc(filTx, filTy);
  float headingDeg = convertDeg(headingRad);
  #if defined LASER_SENSORS
    Serial.print("headingDeg: ");
    Serial.println(headingDeg);
  #endif
  int16_t conFilX = convertX(filX);
  int16_t conFilY = convertY(filY);
  int16_t area16 = (int16_t) area;
  leftSensor = conFilX;
  rightSensor = conFilY;
  if (sendToWebApp) {
    //sprintf (buff, "x: %d y: %d h: %f a: %d mx: %f my: %f", conFilX, conFilY, headingDeg, area16, (maxX + minX)/2, (maxY + minY)/2);
    sprintf (buff, "x: %d y: %d h: %f a: %d", conFilX, conFilY, headingDeg, area16);
    //sprintf (buff, "a: %d", area16);
    wsSend(id, buff);
  }
  return headingDeg;
}

float sendCoords(uint8_t id, int32_t area) {
  return getHeading(true, id, area);
}

void wsSendWrapper(uint8_t id, char * buff){
  wsSend(id, buff);
}

void instruxToDrive(char c) {
  dxn = c;
  if (c == 'F')
    forward();
  else if (c == 'B')
    backward();
  else if (c == 'L')
    left();
  else if (c == 'R')
    right();
  //sendCoords(0);
}

String tupToInstrux(tuple<double,double> dirs){
  String instrux = "";
  double dRadius = get<0>(dirs);
  double dHeading = get<1>(dirs)*180/PI;  //degrees
  double inchesPerSecF = 7.789;
  double timeFmillis = dRadius/inchesPerSecF*1000;
  Serial.print("dHeading: ");
  Serial.println(dHeading);
  Serial.print("timeFmillis: ");
  Serial.println(timeFmillis);
  float initHeading = getHeading();
  float targetHeading = initHeading + (float) dHeading;
  float HEADING_ERROR = 5;
  float ahe;
  if (dHeading > 0) {
    while (true) {
      left();
      ahe = adjustedHeadingError(targetHeading, getHeading());
      if (ahe < HEADING_ERROR)
        break;
    }
  }
  else if (dHeading < 0) {
    while (true) {
      right();
      ahe = adjustedHeadingError(targetHeading, getHeading());
      if (ahe < HEADING_ERROR)
        break;
    }
  }
  long t3 = millis();
  while (true) {
    if (!obstacleCheck()){
      forward();
      if (millis() - t3 > timeFmillis)
        break;
    }
    else
      obstacleAvoid();
  }
  Serial.println(instrux);
  return instrux;
}

int32_t getArea() {
  int32_t area = scanBlocks();
  double areaA[4] = {area, 0, 0, 0};
  area = (kalman.getFilteredValue(areaA, 'X'))[0];
  return area;
}

bool scan() {
  long t1 = millis();
  int count = 0;
  //double timeR360 = 1.6*1000;
  double timeR360 = 2.4 * 1000;
  bool found = foundBall();
  while (!found) {
    long t2 = millis();
    rightSlow();
    if (millis() - t1 > timeR360)
      return false;
    found = foundBall();
    Serial.print("scanIterTime: ");
    Serial.println(millis() - t2);
    count++;
  }
  return true;
}

void track2(int32_t sig = 2);
void track2(int32_t sig) {
  int X_ERROR_ERROR = 20;
  int Y_ERROR_ERROR = 10;
  Block ball = foundBall2(sig);
  printWebApp("passed foundBall2()");
  Ballapproach bApproach(ball, servo_left, servo_right);
  printWebApp("passed bApproach");
  int32_t area = getArea();
  bApproach.setArea(area);
  printWebApp("set the area, now tracking");
  if(area){
      int32_t x_error = bApproach.getX() - CENTER_X;
      int32_t y_error = bApproach.getY() - CENTER_Y;
      Serial.print("x_error: ");
      Serial.println(x_error);
      Serial.print("y_error: ");
      Serial.println(y_error);
      while(((abs(x_error) > 20) || (abs(y_error) > 10)) && area){
        ball = foundBall2(sig);
        bApproach.update2(ball);
        bApproach.setArea(getArea());
        x_error = bApproach.getX() - CENTER_X;
        y_error = bApproach.getY() - CENTER_Y;
        Serial.print("x_error: ");
        Serial.println(x_error);
        Serial.print("y_error: ");
        Serial.println(y_error);
      }
      x_error = bApproach.getX() - CENTER_X;
      y_error = bApproach.getY() - CENTER_Y;
      Serial.println("final x and y errors");
      Serial.print("x_error: ");
      Serial.println(x_error);
      Serial.print("y_error: ");
      Serial.println(y_error);
      String s = "finished loop " + String(x_error);
      printWebApp(s);
  }
  /*else{
    Serial.println("can't find ball");
  }*/
  return;
}

float adjustedHeadingError(float f1, float f2){
  float spinerror = abs(f1 - f2);
  if (spinerror > 350){ //might be 355° and 3°.  should be small difference but is calculated very big
    if (f1 >= 350){
      f1 -= 360;
    }
    else if (f2 >= 350){
      f2 -= 360;
    }
    Serial.print("ahe: ");
    Serial.println(abs(f1 - f2));
    return abs(f1 - f2);
  }
  else {
    Serial.print("ahe: ");
    Serial.println(abs(f1 - f2));
    return spinerror;
  }
}

bool scan2(){
  long t1 = millis();
  int count = 0;
  bool found = foundBall();
  float startHeading = getHeading(false, 0,0, true);
  Serial.print("startHeading: ");
  Serial.println(startHeading);
  float SPINERROR = 10;   //5 degrees
  float currHeading;
  while(!found){
    rightSlow();
    //currHeading = getHeading(false, 0,0, true);
    currHeading = getHeading();
    float ahe = adjustedHeadingError(currHeading, startHeading);
    if (millis() - t1 > 1000 &&  ahe < SPINERROR)
      return false;
    found = foundBall();
    Serial.print("currHeading: ");
    Serial.println(currHeading);
    count++;
  }
  return true;
}

float adjustHeading(float f) {
  float adjusted = f;
  if (f < 0)
    adjusted = f + 360;
  if (f > 360)
    adjusted = f - 360;
  Serial.print("adjusted: ");
  Serial.println(adjusted);
  return adjusted;
}

bool sweep(float range, int32_t sig) {
  long t1 = millis();
  bool found = foundBall(sig);
  float startHeading = getHeading();
  float sweepLeft = adjustHeading(startHeading + range);
  float sweepRight = adjustHeading(startHeading - range);
  Serial.print("startHeading: ");
  Serial.println(startHeading);
  Serial.print("sweepLeft: ");
  Serial.println(sweepLeft);
  Serial.print("sweepRight: ");
  Serial.println(sweepRight);
  float SPINERROR = 10;   //5 degrees
  float currHeading;
  float ahe;
  while(!found){
    rightSlow();
    currHeading = getHeading();
    found = foundBall(sig);
    if(found)
      return true;
    ahe = adjustedHeadingError(currHeading, sweepRight);
    if (ahe < SPINERROR){
      break;
    }
    Serial.print("currHeading: ");
    Serial.println(currHeading);
  }
  while (!found) {
    leftSlow();
    currHeading = getHeading();
    found = foundBall(sig);
    if (found)
      return true;
    ahe = adjustedHeadingError(currHeading, sweepLeft);
    if (ahe < SPINERROR) {
      break;
    }
    Serial.print("currHeading: ");
    Serial.println(currHeading);
  }
  while (!found) {
    rightSlow();
    currHeading = getHeading();
    found = foundBall(sig);
    if (found)
      return true;
    ahe = adjustedHeadingError(currHeading, startHeading);
    if (ahe < SPINERROR) {
      break;
    }
    Serial.print("currHeading: ");
    Serial.println(currHeading);
  }
  return false;
}

void returnToBase() {
  float initHeading = ballsearch.getInitHeading();
  float targetHeading;
  float currHeading;
  float SPINERROR = 10;
  int32_t HOME_BASE_SIGNITURE = 3;
  double inchesPerSecF = 7.789;
  double timeFmillis;
  float SWEEP_DEGREES = 30;
  bool found;
  float ahe;
  Serial.println("return to base");
  Serial.print("initHeading: ");
  Serial.println(initHeading);
  if (initHeading - 180 < 0)
    targetHeading = initHeading + 180;
  else
    targetHeading = initHeading - 180;
  Serial.print("targetHeading: ");
  Serial.println(targetHeading);
  long t1 = millis();
  while (true) {
    rightSlow();
    currHeading = getHeading();
    Serial.print("currHeading: ");
    Serial.println(currHeading);
    Serial.print("difference: ");
    Serial.println(currHeading - targetHeading);
    ahe = adjustedHeadingError(currHeading, targetHeading);
    if (ahe < SPINERROR)
      break;
  }
  //go forward some
  //check +90° then -90° then back to original heading, checking for home base
  //if found, approach(home_base)
  //else
  //loop back to go forward some
  Serial.println("end turn around");
  found = sweep(SWEEP_DEGREES, HOME_BASE_SIGNITURE);
  //found = foundBall(HOME_BASE_SIGNITURE);
  while (!found) {
    timeFmillis = ballsearch.getMaxR() / inchesPerSecF * 1000;
    long t3 = millis();
    while (true) {
      forward();
      if (millis() - t3 > timeFmillis)
        break;
    }
    //scan
    found = sweep(SWEEP_DEGREES, HOME_BASE_SIGNITURE);
  }
  //found homebase, approach it
  track2(HOME_BASE_SIGNITURE);
}

void grasp()
{
  if (grab == 0)
  {
    servo_grab.write(170);
    grab = 1;
  }
  else
  {
    servo_grab.write(80);
    grab = 0;
  }
}

//
// Setup //
//

void setupPins() {
  // setup Serial, LEDs and Motors
  Serial.begin(115200);
  DEBUG("Started serial.");

  //pinMode(LED_PIN, OUTPUT);    //Pin D0 is LED
  //LED_OFF;                     //Turn off LED
  //DEBUG("Setup LED pin.");

  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  servo_grab.attach(SERVO_GRAB);
  DEBUG("Setup motor pins");
}

bool turnon = 0;

void webSocketEvent(uint8_t id, WStype_t type, uint8_t * payload, size_t length) {

  switch (type) {
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
        drive(180 - payload[1], payload[2]);

    case WStype_TEXT:
      DEBUG("On connection #", id)
      DEBUG("  got text: ", (char *)payload);

      if (payload[0] == '#') {
        if (payload[1] == '#') {
          char instrux;
          for (int i = 2; payload[i] != '@'; i++) {
            instrux = (char) payload[i];
            instruxToDrive(instrux);
          }
          drive(90, 90);
        }
        else if (payload[1] == '!') {
          char instrux;
          String cmd = "";
          for (int i = 2; payload[i] != '@'; i++) {
            instrux = (char) payload[i];
            cmd += instrux;
          }
          if (cmd == "Beg") {
            Serial.println("Beg is: " + cmd);
            ballsearch.setMaxR(40);
            ballsearch.setInitHeading(getHeading());
            while (!scan2()) {
              tuple<double, double> toMove = ballsearch.search("L");
              //Serial.println("out of ballsearch.search");
              String instructions = tupToInstrux(toMove);
              drive(90, 90);
            }
            track2();
            //grasp();
          }
          else if (cmd == "Ret"){
            Serial.println("Ret is: " + cmd);
            returnToBase();
          }
          else if (cmd == "App"){
            Serial.println("App is: " + cmd);
            track2();
          }
          else if (cmd == "Dep"){
            Serial.println("Dep is: " + cmd);
            scan2();
          }
          else if (cmd == "Gra"){
            Serial.println("Gra is: " + cmd);
            grasp();
          }
          else if (cmd == "Can")
            Serial.println("Can is: " + cmd);

          drive(90, 90);
        }
        else if (payload[1] == 'C') {
          //LED_ON;
          wsSend(id, "Hello world!");
        }
        else if (payload[1] == 'F') {
          forward();
          dxn = payload[1];
        }
        else if (payload[1] == 'B') {
          backward();
          dxn = payload[1];
        }
        else if (payload[1] == 'L') {
          left();
          dxn = payload[1];
        }
        else if (payload[1] == 'R') {
          right();
          dxn = payload[1];
        }
        //ecode H = UL, I = UR, J=DL, K=DR
        else if (payload[1] == 'U') {
          if (payload[2] == 'L') {
            servo_left_ctr -= 1;
            dxn = 'H';
            //sendCoords(id);
          }
          else if (payload[2] == 'R') {
            servo_right_ctr += 1;
            dxn = 'I';
            //sendCoords(id);
          }
          char tx[20] = "Zero @ (xxx, xxx)";
          sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
          wsSend(id, tx);
        }
        else if (payload[1] == 'D') {
          if (payload[2] == 'L') {
            servo_left_ctr += 1;
            dxn = 'J';
            //sendCoords(id);
          }
          else if (payload[2] == 'R') {
            servo_right_ctr -= 1;
            dxn = 'K';
            //sendCoords(id);
          }
          char tx[20] = "Zero @ (xxx, xxx)";
          sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
          wsSend(id, tx);
        }
        else {
          dxn = 'X';
          stop();
        }
      }

      break;
  }
}
