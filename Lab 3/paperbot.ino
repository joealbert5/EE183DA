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


const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;
Servo servo_left;
Servo servo_right;
int servo_left_ctr = 90;
int servo_right_ctr = 90;

char dxn = 'X';
double q_processNoise [3] = {.125,.125,.125};
double r_sensorNoise [3] = {6,6,6};
double p_estimateError [3] = {1,1,1};
double x_initVal [3] = {10,10,0};
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

uint16_t convertX(uint16_t x){
  return .0941*x - 3.4884;
}

uint16_t convertY(uint16_t x){
  return -1*.0949*x + 27.95;
}

void sendCoords(uint8_t id){
  char buff [50];
  uint16_t* p = scanXY();
  printArr(p, *p);
  float* theta = printMag();
  double sensX = (double) *(p + 1);
  double sensY = (double) *(p + 2);
  double sensT = (double) *(theta + 2);
  double measurements [3] = {sensX, sensY, sensT};
  //printArr((uint16_t)theta, *theta);
  double* filteredM = kalman.getFilteredValue(measurements, dxn);
  uint16_t filX = (uint16_t) *(filteredM + 0);
  uint16_t filY = (uint16_t) *(filteredM + 1);
  uint16_t filT = (uint16_t) *(filteredM + 2);
  sprintf (buff, "x: %d y: %d theta: %d", filX, filY, filT);
  //sprintf (buff, "x: %d y: %d dxn: %c", filX, filY, dxn);
  
  //Serial.println(buff);
  wsSend(id, buff);
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
                if(payload[1] == 'C') {
                  LED_ON;
                  wsSend(id, "Hello world!");
                }
                else if(payload[1] == 'F'){ 
                  forward();
                  dxn = payload[1];
                  sendCoords(id);
                }
                else if(payload[1] == 'B') {
                  backward();
                  dxn = payload[1];
                  sendCoords(id);
                }
                else if(payload[1] == 'L') {
                  left();
                  dxn = payload[1];
                  sendCoords(id);
                }
                else if(payload[1] == 'R') {
                  right();
                  dxn = payload[1];
                  sendCoords(id);
                }
                else if(payload[1] == 'U') {
                  if(payload[2] == 'L') 
                    servo_left_ctr -= 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr += 1;
                  char tx[20] = "Zero @ (xxx, xxx)";
                  sprintf(tx, "Zero @ (%3d, %3d)", servo_left_ctr, servo_right_ctr);
                  wsSend(id, tx);
                }
                else if(payload[1] == 'D') {
                  if(payload[2] == 'L') 
                    servo_left_ctr += 1;
                  else if(payload[2] == 'R') 
                    servo_right_ctr -= 1;
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
