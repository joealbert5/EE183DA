//ballsearch2.cpp
/* A simplified one dimensional Kalman filter implementation - actually a single variable low pass filter ;-)
   Based on: http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
*/

#ifndef _Ballsearch_h
#define _Ballsearch_h

#include <vector>
#include <cmath>
#include <String>
#include <cstdlib>  //rand() % 100 + 1  -> in range 1 to 100
//rand() % 30 + 1985 -> in range 1985 to 2014
#include <tuple>
#include <time.h>
//#include <iostream>

using namespace std;

class Ballsearch {
  private:
    /* Ballsearch variables */
    double maxR;
    double XMAX;
    double YMAX;
    vector<tuple<int,int>> pastPoints;
    tuple<int,int> start;
    tuple<int,int> endd;
    int count;
    double prevAngle;
    bool finish;

    int _randint(int low, int high){
      srand(millis());
      int diff = high - low;
      int r = rand() % diff + low;
      return r;
    }

    double _pow(double base, double exp){return (double) pow(base, exp);}

    double _sqrt(double x){return sqrt(x);}

    double _atan2(double y, double x){return atan2(y, x);}

    double _dist(double x, double y){
      double x2 = _pow(x, 2);
      double y2 = _pow(y, 2);
      return _sqrt(x2 + y2);
    }

    double _dist(tuple<int,int> tup){
      int x = _getTup(tup,0);
      int y = _getTup(tup,1);
      double x2 = _pow(x, 2);
      double y2 = _pow(y, 2);
      return _sqrt(x2 + y2);
    }

    tuple<int,int> _subTup(tuple<int,int> tup1, tuple<int,int> tup2){
      int sub1 = get<0>(tup1) - get<0>(tup2);
      int sub2 = get<1>(tup1) - get<1>(tup2);
      return make_tuple(sub1,sub2);
    }

    tuple<int, int> _tuple(int x, int y){return make_tuple(x, y);}

  int _getTup(tuple<int, int> tup, int index){
    if(index == 0) return get<0>(tup);
    else if (index == 1) return get<1>(tup);
  }

  String _printTup(tuple<int,int> tup){
    String e1 = String(get<0>(tup));
    String e2 = String(get<1>(tup));
    return "(" + e1 + ", " + e2 + ")";
  }

  String _printTup(tuple<double,double> tup){
    String e1 = String(get<0>(tup));
    String e2 = String(get<1>(tup));
    return "(" + e1 + ", " + e2 + ")";
  }

    tuple<int, int> _unitDxn(tuple<int,int> tup, int unit){
      int x = _getTup(tup, 0);
      int y = _getTup(tup, 1);
      double mag = _dist(x, y);
      if (mag != 0) {
        int ux = (int) x/mag*unit;
        int uy = (int) y/mag*unit;
        return _tuple(ux, uy);
      }
      else{
        //Serial.println("mag was " + str(mag));
        return _tuple(0,0);
      }
    }

  tuple<int,int> _randCoordUnit(String side){
    tuple<int,int> rand = _tuple(0,0);
    if (side == "L"){
      rand = _tuple(_randint(-1*this->XMAX, 0), _randint(-1*this->YMAX, this->YMAX));
    }
    else if (side == "R"){
      rand = _tuple(_randint(0, this->XMAX), _randint(-1*this->YMAX, this->YMAX));
    }
    else{
      rand = _tuple(_randint(-1*this->XMAX, this->XMAX), _randint(-1*this->YMAX, this->YMAX));
    }
    return _unitDxn(rand, this->maxR);
    }

    tuple<bool, double> _isValidRand(tuple<int,int> endd, tuple<int,int> unitRand, double prevAngle, String side, bool retry){
    if ((side == "L" && _getTup(endd, 0) <= 0 && _getTup(endd, 0) >= -XMAX && abs(_getTup(endd, 1)) <= YMAX) || (side == "R" && _getTup(endd,0) >= 0 && _getTup(endd,0) <= XMAX && abs(_getTup(endd,1)) <= YMAX)){
      tuple<int,int> newDxn = unitRand;
      double newAngle = _atan2(_getTup(newDxn,1), _getTup(newDxn,0));
      if (newAngle < 0)
        newAngle += 2*PI;
      if (!retry)
        return make_tuple((abs(newAngle - prevAngle) <= PI/2), newAngle);
      else
        return make_tuple(retry, newAngle);
    }
    else
      return make_tuple(false, 0);
  }

  bool _isNotNearby(tuple<int,int> curr){
    int cr0 = _getTup(curr,0);
    int cr1 = _getTup(curr,1);
    for(vector<tuple<int,int>>::iterator it = pastPoints.begin(); it != pastPoints.end(); it++){
      //cout << get<0>(*it) << endl;
      int ct0 = get<0>(*it);
      int ct1 = get<1>(*it);
      tuple<int,int> diff = make_tuple((ct0-cr0),(ct1-cr1));
      int d = _dist(diff);
      if(d <= 50)
        return false;
    }
    return true;
  }

    //samples is a vector of tuples, each tuple is a tuple, double
  tuple< tuple<int,int>,double > _getOutOfCorner(double prevAngle, vector< tuple< tuple<int,int>,double > > samples){
    tuple< tuple<int,int>,double > bestSample = samples[0];
    double bestTarget = 0;
    for(vector<tuple<tuple<int,int>,double>>::iterator it = samples.begin(); it != samples.end(); it++){
      tuple<int,int> endd = get<0>(*it);
      double newAngle = get<1>(*it);
      double dAngle = abs(prevAngle - newAngle);
      double target = abs(PI/2 - dAngle);
      if(target < bestTarget){
        bestTarget = target;
        bestSample = make_tuple(endd, newAngle);
      }
    }
    return bestSample;
  }
    
  public:
    Ballsearch(tuple<int,int> start, double maxR = 45, double XMAX = 200, double YMAX = 200) {
        this->maxR = maxR;
        this->YMAX = YMAX;
        this->XMAX = XMAX;
        this->pastPoints.push_back(start);
        this->start = start;
        count = 0;
        prevAngle = 0;
        finish = false;
    }

    void setMaxR(int r){this->maxR = r;}
    void setXMAX(int x){this->XMAX = x;}
    void setYMAX(int x){this->YMAX = x;}
    void addToHistory(tuple<int,int> t){this->pastPoints.push_back(t);}

  String printTup(tuple<int,int> tup){
    String e1 = String(get<0>(tup));
    String e2 = String(get<1>(tup));
    return "(" + e1 + ", " + e2 + ")";
  }

  tuple<double,double> toMove(tuple<int,int> start, tuple<int,int> endd){
      //TODO: implement this
      //Serial.print("start is: ");
      //Serial.println(_printTup(start));
      //Serial.print("end is: ");
      //Serial.println(_printTup(endd));
      tuple<int,int> diff = _subTup(endd, start);
      double dHeading = _atan2(_getTup(diff, 1), _getTup(diff, 0));
      double dRadius = _dist(diff);
      delay(100);
      return make_tuple(dRadius, dHeading);
  }

  tuple<int,int> movee(tuple<int,int>&start, tuple<int,int>&endd){
    //TODO: implement this
    Serial.print("start is: ");
    Serial.println(_printTup(start));
    Serial.print("end is: ");
    Serial.println(_printTup(endd));
    delay(100);
    Serial.print("size is: ");
    Serial.println(this->pastPoints.size());
    return endd;
  }

  bool scan(){
    //TODO: spin 360 scanning for ball
    return false;
  }

  tuple<double,double> search(String side){/*
    tuple<int,int> start = _tuple(0,0);
    tuple<int,int> endd;
    int count = 0;
    double prevAngle = 0;
    bool finish = false;*/
    double newAngle = 0;
    int countToRetry = 0;
    vector< tuple< tuple<int,int>,double > > samples;
    while (true){
      tuple<int,int> unitRand;
      if (count == 0) unitRand = _randCoordUnit(side);
      else unitRand = _randCoordUnit("None");
      //Serial.println(_printTup(unitRand));
      endd = _tuple(_getTup(start,0) + _getTup(unitRand,0), _getTup(start,1) + _getTup(unitRand,1));
      //Serial.println(_printTup(endd));
      tuple<bool, double> res = _isValidRand(endd, unitRand, prevAngle, side, countToRetry >= 1000);
      if (get<0>(res) && _isNotNearby(endd)){
        newAngle = get<1>(res);
        break;
      }
      countToRetry++;
      //Serial.print("countToRetry is ");
      //Serial.println(countToRetry);
      //delay(10);
      if (countToRetry > 90 && get<0>(res)){
        if (countToRetry >= 100 && samples.size() >= 10){
          tuple< tuple<int,int>,double > bSample = _getOutOfCorner(prevAngle, samples);
          newAngle = get<1>(bSample);
          endd = get<0>(bSample);
          //Serial.println(_printTup(endd));
          break;
        }
        //Serial.println(_printTup(endd));
        samples.push_back(make_tuple(endd, get<1>(res)));
      }
    }
    //Serial.println(_printTup(endd));
    tuple<double,double> moveTo = toMove(start,endd);
    Serial.println(_printTup(moveTo));
    start = endd;
    addToHistory(start);
    count++;
    //Serial.print("count is ");
    //Serial.println(count);
    prevAngle = newAngle;
    return moveTo;
  }
};

#endif
