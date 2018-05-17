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

//#define    RANGE_SENSORS    //update in lasermag.cpp too
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

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
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
  //printWebApp(String(area));
  sendCoords(0, area);
  //Serial.println("passed sendCoords");
  //Serial.println("finished loop");
  //obstacleAvoid();
}


//
// Movement Functions //
//

void obstacleAvoid()
{
  if (leftSensor > 10 && rightSensor > 10)
    forward();
  else if (leftSensor <= 10 && rightSensor > 10)
  {
    float current = sendCoords(0);
    if(sendCoords(0) > current - 89)
      right();
  }
  else if (rightSensor <= 10 && leftSensor > 10)
  {
    float current = sendCoords(0);
    if(sendCoords(0) > current - 89)
      left();
  }
  else if (leftSensor <= 10 && rightSensor <= 10)
    stop();
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

float sendCoords(uint8_t id, int32_t area) {
  char buff [50];
  Serial.println(" ");
  int16_t* p = scanXY(12, 0, bias[2]);
  //Serial.println("left scanxy");
  //printArr(p, *p);
  double sensX = (double) * (p + 1);
  double sensY = (double) * (p + 2);
  double sensTx = (double) * (p + 3);
  double sensTy = (double) * (p + 4);
  int16_t filX = (int16_t) sensX;
  int16_t filY = (int16_t) sensY;
  int16_t filTx = (int16_t) sensTx;
  int16_t filTy = (int16_t) sensTy;
  findMinMax(sensTx, sensTy);
  /*
  Serial.print("Max x: ");
  Serial.print(maxX);
  Serial.print(" Min x: ");
  Serial.print(minX);
  Serial.print(" Max y: ");
  Serial.print(maxY);
  Serial.print(" Min y: ");
  Serial.print(minY);
 
  Serial.print("x: ");
  Serial.print(sensTx);
  Serial.print(" y: ");
  Serial.print(sensTy);
  */
  float headingRad = headingCalc(filTx, filTy);
  float headingDeg = convertDeg(headingRad);
  #if defined LASER_SENSORS
    Serial.println("headingDeg: ");
    Serial.print(headingDeg);
  #endif
  int16_t conFilX = convertX(filX);
  int16_t conFilY = convertY(filY);
  int16_t area16 = (int16_t) area;
  leftSensor = conFilX;
  rightSensor = conFilY;
  sprintf (buff, "x: %d y: %d h: %f a: %d", conFilX, conFilY, headingDeg, area16);
  //sprintf (buff, "a: %d", area16);
  wsSend(id, buff);

  return headingDeg;
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
  double instruxPerRadius = 5.1;
  double instruxPerDegree = 6.0;
  double inchesPerSecF = 7.789;
  //double degreesPerSecR = 193;
  //double degreesPerSecL = 236;
  double degreesPerSecR = 193;
  double degreesPerSecL = 236;
  double timeFmillis = dRadius/inchesPerSecF*1000;
  double timeLmillis = dHeading/degreesPerSecL*1000;
  double timeRmillis = dHeading/degreesPerSecR*1000;
  int nRad = (int) dRadius/instruxPerRadius;
  int nDeg = (int) dHeading/instruxPerDegree;
  delay(200);
  Serial.println(dHeading);
  Serial.print("timeFmillis: ");
  Serial.println(timeFmillis);
  Serial.print("timeRmillis: ");
  Serial.println(timeRmillis);
  Serial.print("timeLmillis: ");
  Serial.println(timeLmillis);
  if (dHeading > 0){
    long t1 = millis();
    while(true){
      left();
      delay(10);
      instrux += "L";
      if (millis() - t1 > timeLmillis)
        break;
    }
  }
  else if (dHeading < 0){
    long t2 = millis();
    while(true){
      right();
      delay(10);
      instrux += "R";
      if (millis() - t2 > abs(timeRmillis))
        break;
    }
  }
  long t3 = millis();
  while(true){
    forward();
    delay(10);
    instrux += "F";
    if (millis() - t3 > timeFmillis)
      break;
  }
  Serial.println(instrux);
  return instrux;
}

int32_t getArea(){
  int32_t area = scanBlocks();
  double areaA[4] = {area,0,0,0};
  area = (kalman.getFilteredValue(areaA, 'X'))[0];
  return area;
}

bool scan(){
  long t1 = millis();
  int count = 0;
  //double timeR360 = 1.6*1000;
  double timeR360 = 2.4*1000;
  bool found = foundBall();
  while(!found){
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

void track(){
  int X_ERROR_ERROR = 20;
  int Y_ERROR_ERROR = 10;
  long motorX_adjust_millis = 200;
  long motorY_adjust_millis = 200;
  Block ball = foundBall2();
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
      //first center x
      while((abs(x_error) > 20) || (abs(y_error) > 10)){
        Serial.println(" ");
        while(abs(x_error) > 20){
          if(x_error < 0){
            //object is on right side of screen
            long t1 = millis();
            while(millis() - t1 < motorX_adjust_millis)
              bApproach.right();
            bApproach.stopApp();
            if (motorX_adjust_millis > 50)
              motorX_adjust_millis /= 1.13;
            else
              break;
          }
          if(x_error > 0){
            //object is on left side of screen
            long t1 = millis();
            while(millis() - t1 < motorX_adjust_millis)
              bApproach.left();
            bApproach.stopApp();
            if (motorX_adjust_millis > 50)
              motorX_adjust_millis /= 1.13;
            else
              break;
          }
          ball = foundBall2();
          bApproach.updateBlock(ball);
          x_error = bApproach.getX() - CENTER_X;
          printWebApp(String(x_error));
        }
        //now center y
        while(abs(y_error) > 10){
          if(y_error < 0){
            //object is on above the screen
            long t1 = millis();
            while(millis() - t1 < motorY_adjust_millis)
              bApproach.forward();
            bApproach.stopApp();
            if (motorY_adjust_millis > 50)
              motorY_adjust_millis /= 1.13;
            else
              break;
          }
          if(y_error > 0){
            //object is on right side of screen
            long t1 = millis();
            while(millis() - t1 < motorY_adjust_millis)
              bApproach.backward();
            bApproach.stopApp();
            if (motorY_adjust_millis > 50)
              motorY_adjust_millis /= 1.13;
            else
              break;
          }
          ball = foundBall2();
          bApproach.updateBlock(ball);
          y_error = bApproach.getY() - CENTER_Y;
        }
        
        ball = foundBall2();
        bApproach.updateBlock(ball);
        x_error = bApproach.getX() - CENTER_X;
        y_error = bApproach.getY() - CENTER_Y;
        Serial.println("adjusted x and y");
        Serial.print("x_error: ");
        Serial.println(x_error);
        Serial.print("y_error: ");
        Serial.println(y_error);
        String s = "finished loop " + String(x_error);
        printWebApp(s);
        if(motorX_adjust_millis < 50)
          motorX_adjust_millis += 100;
        if(motorY_adjust_millis < 50)
          motorY_adjust_millis += 100;
    }
    Serial.println("done tracking");
    return;
  }
  /*else{
    Serial.println("can't find ball");
  }*/
  return;
}

void track2(){
  int X_ERROR_ERROR = 20;
  int Y_ERROR_ERROR = 10;
  Block ball = foundBall2();
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
        ball = foundBall2();
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
  DEBUG("Setup motor pins");
}

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
          if (cmd == "Beg"){
            Serial.println("Beg is: " + cmd);
            ballsearch.setMaxR(30);
            while(!scan()){
              tuple<double,double> toMove = ballsearch.search("L");
              //Serial.println("out of ballsearch.search");
              String instructions = tupToInstrux(toMove);
              drive(90, 90);
            }
          }
          else if (cmd == "Ret"){
            Serial.println("Ret is: " + cmd);
            scan();
          }
          else if (cmd == "App"){
            Serial.println("App is: " + cmd);
            track2();
          }
          else if (cmd == "Dep")
            Serial.println("Dep is: " + cmd);
          else if (cmd == "Gra")
            Serial.println("Gra is: " + cmd);
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
