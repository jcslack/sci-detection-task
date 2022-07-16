// ACTUATOR.h
#ifndef actuator_h
#define actuator_h

#include <Arduino.h>

class actuator {    
public:
    actuator(int pin1,int pin2,int pin3,int pin4);
    void forward();
    void backward();
    void upward();
    void downward();
    void stop_movement(); 
private:
    int uPin; //upward pin
    int dPin; //downward pin
    int fPin; //forward pin
    int bPin; //backward pin
    byte uState; //upward state
    byte dState; //downward state
    byte fState; //forward state
    byte bState; //backward state
  
};

#endif
