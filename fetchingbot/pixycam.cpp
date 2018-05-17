#include <Pixy.h>
#include <PixyI2C.h>
#include <PixySPI_SS.h>
#include <PixyUART.h>
#include <TPixy.h>
#include <SPI.h>
#include "pixycam.h"
#include "server.h"

uint16_t blocks;
uint16_t old_blocks;
//const int BUTTON = D0;
int buttonState = 0;
int32_t prevArea = 0;
const int MIN_AREA_DETECTION = 100;

PixySPI_SS pixy(10);

void printWebApp(String s){
  char buff [100];
  s.toCharArray(buff,sizeof(buff));
  wsSend(0,buff);
}

bool foundBall(){
  uint16_t blocks = pixy.getBlocks();
  int32_t area;
  char buff [100];
  if (blocks > 0) {
    for(int i = 0; i < blocks; i++){
      int32_t w = pixy.blocks[i].width;
      int32_t h = pixy.blocks[i].height;
      area = w * h;
      sprintf (buff, "found ball scan: (%d, %d)", i, area);
      Serial.println(buff);
      if(area > MIN_AREA_DETECTION){
        //Serial.println("******FOUND BALL********");
        sprintf (buff, "found ball exit scan: (%d, %d)", i, area);
        Serial.println(buff);
        printWebApp("***FOUND BALL***");
        return true;
      }
    }
    //Serial.println("Found blocks but not minimum area");
    printWebApp("Found blocks but not minimum area");
  }
  else{
    //Serial.println("blocks not > 0");
    printWebApp("blocks not > 0");
  }
  return false;
}

Block foundBall2(){
  int32_t area;
  int count = 0;
  while (count < 20){
    uint16_t blocks = pixy.getBlocks();
    if (blocks > 0) {
      Serial.print("nBlocks: ");
      Serial.println(blocks);
      delay(10);
      for(int i = 0; i < blocks; i++){
        int32_t w = pixy.blocks[i].width;
        int32_t h = pixy.blocks[i].height;
        area = w * h;
        if(area > MIN_AREA_DETECTION){
          //Serial.println("******FOUND BALL********");
          char buff [50];
          sprintf (buff, "found ball: (%d, %d)", i, area);
          Serial.println(buff);
          printWebApp("***FOUND BALL***");
          return pixy.blocks[i];
        }
      }
      //Serial.println("Found blocks but not minimum area");
      printWebApp("Found blocks but not minimum area");
    }
    count++;
  }
  //Serial.println("blocks not > 0, counted 10 times");
  printWebApp("blocks not > 0, counted 10 times");
  uint16_t blocks = pixy.getBlocks();
  delay(30);
  return pixy.blocks[0];
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
    int32_t dA = area - prevArea;/*
    Serial.print("prevArea = ");
    Serial.print(prevArea);
    Serial.print(", area = ");
    Serial.print(area);
    Serial.print(", dA = ");
    Serial.println(dA);*/
    prevArea = area;
  }
  else{
    Serial.println("blocks not > 0");
    area = 0;
  }
    
  delay(30);
  return area;
}

