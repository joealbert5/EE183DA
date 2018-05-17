//ballsearch2.cpp
/* A simplified one dimensional Kalman filter implementation - actually a single variable low pass filter ;-)
   Based on: http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
*/

#ifndef _Ballapproach_h
#define _Ballapproach_h

#include <cstdlib>
#include <Servo.h>

using namespace std;

const int32_t CENTER_X = 100;
const int32_t CENTER_Y = 280; //max is 320

class Ballapproach {
  private:
    /* Ballapproach variables */
  	int32_t m_ballx;
  	int32_t m_bally;
  	int32_t m_ballw;
  	int32_t m_ballh;
  	int32_t m_balls;
  	Servo m_lServo;
  	Servo m_rServo;
  	int32_t area;

    double _normalize(int32_t data, int32_t center){
      double norm = 0;
      int32_t maxY = 320;
      int32_t minY = 0;
      //center is 280 right now
      int32_t tooFarRange = center - minY;
      int32_t tooCloseRange = maxY - center;
      //normalize tooFarRange, scale accordingly for tooClose
      //range from -1 < norm < ~.14?
      //          -280< error < 40
      int32_t error = data - center;
      if (error < 0)
        norm = error/tooFarRange;
      else if (error > 0)
        norm = error/tooCloseRange;
      return norm;
    }

  public:
    Ballapproach(Block block, Servo left, Servo right) {
        this->m_ballx = block.y;
        this->m_bally = block.x;
        this->m_ballw = block.height;
        this->m_ballh = block.width;
        this->m_balls = block.signature;
        this->m_lServo = left;
        this->m_rServo = right;
    }

    int updateBlock(Block block){
        this->m_ballx = block.y;
        this->m_bally = block.x;
        this->m_ballw = block.height;
        this->m_ballh = block.width;
        this->m_balls = block.signature;
    }
    void setArea(int32_t a){this->area = a;}	//SET THIS FROM KALMAN FILTER IN FETCHINGBOT.INO

    int32_t getX(){return m_ballx;}
    int32_t getY(){return m_bally;}
    int32_t getW(){return m_ballw;}
    int32_t getH(){return m_ballh;}

    void drive(int left, int right) {
      m_lServo.write(left);
      m_rServo.write(right);
    }

    void forward() {
      drive(110, 70);
    }

    void backward() {
      DEBUG("backward");
      drive(70, 110);
    }

    void left() {
      drive(70, 70);
    }

    void right() {
      drive(110, 110);
    }

    void stopApp(){
      drive(90,90);
    }
    

    void update2(Block block){
      updateBlock(block);
      int32_t x_error = m_ballx - CENTER_X;
      int32_t y_error = m_bally - CENTER_Y;
      int speed;
      int32_t diff;
      double y_norm = _normalize(y_error, CENTER_Y);
      speed = constrain(y_norm*400, -100, 400);
      diff = (x_error + (x_error * speed))>>6;
      int leftSpeed = constrain(speed - diff, -400, 400);
      int rightSpeed = constrain(speed + diff, -400, 400);
      leftSpeed = map(leftSpeed,-400,400,180,0); // Map to servo output values
      rightSpeed = map(rightSpeed,-400,400,0,180); // Map to servo output values 
      m_lServo.write(leftSpeed);
      m_rServo.write(rightSpeed);
      char buff [100];
      sprintf(buff, "leftSpeed: %d", leftSpeed);
      Serial.println( buff);
      sprintf(buff, "rightSpeed: %d", rightSpeed);
      Serial.println( buff);
    }

    char * printData(){
      char buff [100];
      //sprintf(buff, "center: %d prevError: %d pgain: %d dgain: %d", m_center, m_prevError, m_pgain, m_dgain);
      return buff;
    }


};

#endif
