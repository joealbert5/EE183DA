/* A simplified one dimensional Kalman filter implementation - actually a single variable low pass filter ;-)
   Based on: http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
*/

#ifndef _Kalman_h
#define _Kalman_h

class Kalman {
  private:
    /* Kalman filter variables */
    uint16_t q; //process noise covariance
    uint16_t r; //measurement noise covariance
    uint16_t x; //value
    uint16_t p; //estimation error covariance
    uint16_t k; //kalman gain
    
  public:
    Kalman(uint16_t process_noise, uint16_t sensor_noise, uint16_t estimated_error, uint16_t intial_value) {
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
    
    uint16_t getFilteredValue(uint16_t measurement) {
      /* Updates and gets the current measurement value */
      //prediction update
      //predicted error covariance = previous + process noise
      this->p = this->p + this->q;
    
      //measurement update
      //gain = ratio between how large sensor noise is compared to previous estimated error
      this->k = this->p / (this->p + this->r);
      //current filtered value = previous filtered value + gain*(unfiltered - filtered value)
      this->x = this->x + this->k * (measurement - this->x);
      //current error = (1 - gain)*previous error
      this->p = (1 - this->k) * this->p;
      
      return this->x;
    }

    
    
    void setParameters(uint16_t process_noise, uint16_t sensor_noise, uint16_t estimated_error) {
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
    }

    void setParameters(uint16_t process_noise, uint16_t sensor_noise) {
        this->q = process_noise;
        this->r = sensor_noise;
    }
    
    uint16_t getProcessNoise() {
      return this->q;
    }
    
    uint16_t getSensorNoise() {
      return this->r;
    }
    
    uint16_t getEstimatedError() {
      return this->p;
    }
};

#endif
