#include <Pixy.h>
#include <PixyI2C.h>
#include <PixySPI_SS.h>
#include <PixyUART.h>
#include <TPixy.h>
#include <SPI.h>
#include "ballsearch2.h"

uint16_t blocks;
uint16_t old_blocks;
//const int BUTTON = D0;
int buttonState = 0;
int32_t prevArea = 0;

PixySPI_SS pixy(10);
Ballsearch b(make_tuple(0,0));

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //pinMode(BUTTON, INPUT);
  Serial.println("pinmocde");
  //pixy.init();
  Serial.println("pixy init");
}

void loop() {
  //Serial.println("begin loop");
  //buttonState = digitalRead(BUTTON);
  //if(buttonState == HIGH){
  /*
    Serial.println("button pressed");
    uint16_t blocks = pixy.getBlocks();
    if (blocks > 0){
      int32_t w = pixy.blocks[0].width;
      int32_t h = pixy.blocks[0].height;
      int32_t area = w*h;
      int32_t dA = area - prevArea;
      Serial.print("prevArea = ");
      Serial.print(prevArea);
      Serial.print(", area = ");
      Serial.print(area);
      Serial.print(", dA = ");
      Serial.println(dA);
      prevArea = area;
    }
    else
      Serial.println("blocks not > 0");*/
  int count = b.search("L");
  Serial.print("FINISHED.  COUNT IS ");
  Serial.println(count);
  
  delay(200);
  //}
}
