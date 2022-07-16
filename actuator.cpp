//actuator.cpp
#include <Arduino.h>
#include "actuator.h"

actuator::actuator(int pin1,int pin2,int pin3,int pin4){
  fPin = pin1;
  bPin = pin2;
  uPin = pin3;
  dPin = pin4;
  fState = HIGH;
  bState = HIGH;
  uState = HIGH;
  dState = HIGH;
  pinMode(fPin,OUTPUT);
  pinMode(bPin,OUTPUT);
  pinMode(uPin,OUTPUT);
  pinMode(dPin,OUTPUT);
  digitalWrite(fPin,fState);
  digitalWrite(bPin,bState);
  digitalWrite(uPin,uState);
  digitalWrite(dPin,dState);
}

void actuator::forward() {
  fState = LOW;
  digitalWrite(fPin,fState);
}

void actuator::backward() {
  bState = LOW;
  digitalWrite(bPin,bState);
}

void actuator::upward() {
  uState = LOW;
  digitalWrite(uPin,uState);
}

void actuator::downward() {
  dState = LOW;
  digitalWrite(dPin,dState); 
}

void actuator::stop_movement() {
  if (fState == LOW){
    fState = HIGH;
    digitalWrite(fPin,fState);
  }
  else if (bState == LOW){
    bState = HIGH;
    digitalWrite(bPin,bState);
  }
  else if (uState == LOW){
    uState = HIGH;
    digitalWrite(uPin,uState);
  }
  else if (dState == LOW){
    dState = HIGH;
    digitalWrite(dPin,dState);
  }  
}
