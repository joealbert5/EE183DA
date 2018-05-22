#ifndef _pixycam_h
#define _pixycam_h

#include <TPixy.h>

void setupPixy();

int32_t scanBlocks();

bool foundBall(int32_t sig = 2);

Block foundBall2(int32_t sig = 2);

void printWebApp(String s);

#endif

