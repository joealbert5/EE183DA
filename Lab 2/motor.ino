#include <Servo.h>

Servo servo;

void setup() {
  servo.attach(16);
  servo.write(0);
  delay(1000);
}

void loop() {
  while(){
   servo.write(90);
   delay(1000);
   servo.write(0);
   delay(1000);
  }
}
//different tempos will be same code with different delays

// this is what i did to get the motor to stop after a set # of iterations 
/*
int i = 0;
void loop() {
  i++;
  if(i < 5){
   servo.write(180);
   delay(1000);
   servo.write(0);
   delay(1000);
  }
  if (i>5){
    servo.write(0);
  }
}*/
