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
const int32_t CENTER_Y = 300; //max is 320

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
      //DEBUG("forward");
      drive(110, 20);
    }

    void backward() {
      DEBUG("backward");
      drive(20, 110);
    }

    void left() {
      DEBUG("left");
      drive(20, 20);
    }

    void right() {
      DEBUG("right");
      drive(110, 110);
    }

    void stopApp(){
      drive(90,90);
    }
    

    void track(Servo leftServo, Servo rightServo){
      

    }


};

#endif
