#include <Servo.h>

Servo servo;

void setup() {
  servo.attach(16);
  servo.write(0);
  pinMode(D4, INPUT);
  Serial.begin(115200);
}

int i=0;
void loop() {
 int ctsValue = digitalRead(D4);
 Serial.println(ctsValue);
 if(ctsValue == HIGH){
   i++;
   if (i<2){
    servo.write(90);
    delay(500);
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(250);
    servo.write(0);
    delay(500);
    servo.write(90);
    delay(250);
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(500);
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(500);
  }
  if(i<100){ 
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(250);
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(500);
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(250);
    servo.write(0);
    delay(500);
    servo.write(90);
    delay(250);
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(500);
    servo.write(0);
    delay(250);
    servo.write(90);
    delay(500);
  }
   if(i>99){
    servo.write(90);
   }
 }
 if (ctsValue = LOW){
    servo.write(90);
    delay(100);
    servo.write(0);
    delay(100);
  } 
}




