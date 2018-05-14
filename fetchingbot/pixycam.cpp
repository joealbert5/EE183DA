#include <Pixy.h>
#include <PixyI2C.h>
#include <PixySPI_SS.h>
#include <PixyUART.h>
#include <TPixy.h>
#include <SPI.h>
#include "pixycam.h"

uint16_t blocks;
uint16_t old_blocks;
//const int BUTTON = D0;
int buttonState = 0;
int32_t prevArea = 0;
const int MIN_AREA_DETECTION = 50;

PixySPI_SS pixy(10);

bool foundBall(){
  uint16_t blocks = pixy.getBlocks();
  int32_t area;
  if (blocks > 0) {
    for(int i = 0; i < blocks; i++){
      int32_t w = pixy.blocks[i].width;
      int32_t h = pixy.blocks[i].height;
      area = w * h;
      if(area > MIN_AREA_DETECTION){
        Serial.println("******FOUND BALL********");
        delay(30);
        return true;
      }
    }
    Serial.println("Found blocks but not minimum area");
  }
  else{
    Serial.println("blocks not > 0");
  }
    
  delay(30);
  return false;
}

void setupPixy()
{
  pixy.init();
  Serial.println("Pixy Init");
}

int32_t scanBlocks()
{
  uint16_t blocks = pixy.getBlocks();
  int32_t area;
  if (blocks > 0) {
    int32_t w = pixy.blocks[0].width;
    int32_t h = pixy.blocks[0].height;
    area = w * h;
    int32_t dA = area - prevArea;
    Serial.print("prevArea = ");
    Serial.print(prevArea);
    Serial.print(", area = ");
    Serial.print(area);
    Serial.print(", dA = ");
    Serial.println(dA);
    prevArea = area;
  }
  else{
    Serial.println("blocks not > 0");
    area = 0;
  }
    
  delay(30);
  return area;
}

