/* A simplified one dimensional Kalman filter implementation - actually a single variable low pass filter ;-)
   Based on: http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
*/

#ifndef _Kalman_h
#define _Kalman_h

#include <vector>
using namespace std;

class Kalman {
  private:
    /* Kalman filter variables */
    double* q; //process noise covariance
    double* r; //measurement noise covariance
    double* x; //value
    double* p; //estimation error covariance
    double k; //kalman gain
    
  public:
    Kalman(double* process_noise, double* sensor_noise, double* estimated_error, double* intial_value) {
      /* The variables are x for the filtered value, q for the process noise, 
         r for the sensor noise, p for the estimated error and k for the Kalman Gain. 
         The state of the filter is defined by the values of these variables.
         
         The initial values for p is not very important since it is adjusted
         during the process. It must be just high enough to narrow down.
         The initial value for the readout is also not very important, since
         it is updated during the process.
         But tweaking the values for the process noise and sensor noise
         is essential to get clear readouts.
         
         For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
         q = 0.125
         r = 32
         p = 1023 //"large enough to narrow down"
         e.g.
         myVar = Kalman(0.125,32,1023,0);
      */
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
        this->x = intial_value; //x will hold the iterated filtered value
    }
    vector<double> dxnToBu(char dxn){
      vector<double> ret;
      double arr [3] = {0,0,0};
      switch(dxn){
                       //{delta-x, delta-y, delta-theta} delta = change per 1/20 seconds
        case 'X': arr[0] = 0; arr[1] = 0; arr[2] = 0;ret.assign(arr, arr + 3); return ret;
        case 'F': arr[0] = 0; arr[1] = -.7; arr[2] = 0; ret.assign(arr, arr + 3); return ret;
        case 'B': arr[0] = 0; arr[1] = .7; arr[2] = 0; ret.assign(arr, arr + 3); return ret;
        case 'L': arr[0] = 0; arr[1] = 0; arr[2] = 4.5; ret.assign(arr, arr + 3); return ret;
        case 'R': arr[0] = 0; arr[1] = 0; arr[2] = -4.5; ret.assign(arr, arr + 3); return ret;
      }
    }

    /*
    double* getFilteredValue(double* measurement, char dxn) {
      vector<double> Bu = dxnToBu(dxn);
      for (int i = 0; i < 4; i++){
        // Updates and gets the current measurement value 
        //prediction update
        //predicted error covariance = previous + process noise
        this->p[i] = this->p[i] + this->q[i];
        //measurement update
        //gain = ratio between how large sensor noise is compared to previous estimated error
        this->k = this->p[i] / (this->p[i] + this->r[i]);
        //current filtered value = previous filtered value + gain*(unfiltered - filtered value)
        this->x[i] = this->x[i] + Bu[i] + this->k * (measurement[i] - this->x[i]);
        //current error = (1 - gain)*previous error
        this->p[i] = (1 - this->k) * this->p[i];
      }
      return this->x;
    }*/

    double* getFilteredValue(double* measurement, char dxn) {
      vector<double> Bu = dxnToBu(dxn);
      for (int i = 0; i < 1; i++){
        // Updates and gets the current measurement value 
        //prediction update
        //predicted error covariance = previous + process noise
        this->p[i] = this->p[i] + this->q[i];
        //measurement update
        //gain = ratio between how large sensor noise is compared to previous estimated error
        this->k = this->p[i] / (this->p[i] + this->r[i]);
        //current filtered value = previous filtered value + gain*(unfiltered - filtered value)
        this->x[i] = this->x[i] + Bu[i] + this->k * (measurement[i] - this->x[i]);
        Serial.print("areaF: ");
        Serial.println(this->x[i]);
        //current error = (1 - gain)*previous error
        this->p[i] = (1 - this->k) * this->p[i];
      }
      return this->x;
    }
    
    void setParameters(double* process_noise, double* sensor_noise, double* estimated_error) {
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
    }

    void setParameters(double* process_noise, double* sensor_noise) {
        this->q = process_noise;
        this->r = sensor_noise;
    }
    
    double* getProcessNoise() {
      return this->q;
    }
    
    double* getSensorNoise() {
      return this->r;
    }
    
    double* getEstimatedError() {
      return this->p;
    }
};

#endif
