#include "Pedals.h"

Pedals::Pedals(){
  throttleVal = 0;
  brakeVal = 0;
}
//Pedals::~Pedals(){}
float Pedals::getThrottle(){  	
  return throttleVal;
}
float Pedals::getBrake(){
  return brakeVal;
}
bool Pedals::getReverse(){
  return reverse;
}
bool Pedals::getSDfront(){
  return SDfront;
}
bool Pedals::getSDrear(){
  return SDrear;
}
bool Pedals::getEstop(){
  return Estop;
}
bool Pedals::processMessage(CAN_message_t &message){
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
