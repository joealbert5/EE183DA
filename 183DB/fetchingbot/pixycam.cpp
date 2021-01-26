#include <Pixy.h>
#include <PixyI2C.h>
#include <PixySPI_SS.h>
#include <PixyUART.h>
#include <TPixy.h>
#include <SPI.h>
#include "pixycam.h"

uint16_t blocks;
uint16_t old_blocks;
const int BUTTON = D0;
int buttonState = 0;
int32_t prevArea = 0;

PixySPI_SS pixy(10);

void setupPixy()
{
  pixy.init();
  Serial.println("Pixy Init");
}

void scanBlocks()
{
  uint16_t blocks = pixy.getBlocks();
  if (blocks > 0) {
    int32_t w = pixy.blocks[0].width;
    int32_t h = pixy.blocks[0].height;
    int32_t area = w * h;
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
    Serial.println("blocks not > 0");
    
  delay(200);
}

