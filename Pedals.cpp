#include "Pedals.h"

Pedals::Pedals(){
    throttleVal = 0;
    brakeVal = 0;
}
//Pedals::~Pedals(){}
float Pedals::getThrottle(){  	
    checkWatchdog();
    return throttleVal;
}
float Pedals::getBrake(){
    checkWatchdog();
    return brakeVal;
}
bool Pedals::getReverse(){
    checkWatchdog();
    return reverse;
}
bool Pedals::getSDfront(){
    checkWatchdog();
    return SDfront;
}
bool Pedals::getSDrear(){
    checkWatchdog();
    return SDrear;
}
bool Pedals::getEstop(){
    checkWatchdog();
    return Estop;
}
bool Pedals::processMessage(CAN_message_t &message){
    patWatchdog();

    throttleVal = message.buf[0]/255.0;
    brakeVal = message.buf[1]/255.0;

    reverse    = (((1<<reverseSwBit)    & (message.buf[2]))!=0);
    SDfront    = (((1<<frontSDBit)      & (message.buf[2]))!=0);
    SDrear     = (((1<<rearSDBit)       & (message.buf[2]))!=0);
    Estop      = (((1<<estopBit)        & (message.buf[2]))!=0);
    brakesOK   = (((1<<brakeWarnBit)    & (message.buf[2]))!=0);
    throttleOK = (((1<<throttleWarnBit) & (message.buf[2]))!=0);

    return true;
}

void Pedals::patWatchdog(){
    watchdogTimer = micros();   //pat the watchdog
}

bool Pedals::checkWatchdog(){
    if(micros() - watchdogTimer >= watchdogPeriod){
        //we haven't had an update in too long.
        throttleVal = 0;
        brakeVal = 0;
        return false; //timed out
    }
    return true;
}