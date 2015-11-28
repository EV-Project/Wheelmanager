#ifndef PEDALS_H 
#define PEDALS_H


#include <inttypes.h>
#include <Arduino.h> // Arduino 1.0
#include <FlexCAN.h>
#include <CANcallbacks.h>

#include <ChallengerEV.h>



class Pedals{
  public:
    Pedals();
    //~Pedals();
    float getThrottle();
    float getBrake();
    bool getReverse();
    bool getSDfront();
    bool getSDrear();
    bool getEstop();
    bool processMessage(CAN_message_t &message);
  private:
    float brakeVal;
    float throttleVal;
  
    bool brakesOK;
    bool throttleOK;
    bool reverse;
    bool SDfront;
    bool SDrear;
    bool Estop;

    uint32_t watchdogTimer = 0;
    uint32_t watchdogPeriod= 100000;   //100mS

    bool checkWatchdog();
    void patWatchdog();
};




#endif
