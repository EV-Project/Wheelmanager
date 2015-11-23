#include <KellyCAN.h>
#include <FlexCAN.h>
#include <CANcallbacks.h>
#include <ChallengerEV.h>
#include "Pedals.h"

//wheels are indexed 1-4
const int wheelnumber = 3;




const int wheelnum = wheelnumber - 1;

const int brakeDacPin = 20;   
const int throttleDacPin = 21;
const int brSwOut = 14;         
const int thSwOut = 15;
const int reSwOut = 16;

const int redLightInPin = 22;
const int greenLightInPin = 23;


//pull the constants out of ChallengerEV.h
const uint32_t ManagerID  = wheel[wheelnum].managerID;  //messages to the wheel managers
const uint32_t KellyreqID = wheel[wheelnum].motorReqID; //messages to the kelly
const uint32_t KellyresID = wheel[wheelnum].motorResID; //messages from the kelly
const uint32_t dashID = wheel[wheelnum].dashID;  //telemetry messages from the wheel manager


FlexCAN CANbus(1000000);
CANcallbacks canbus(&CANbus);
KellyCAN motor(&canbus, KellyreqID, KellyresID);
Pedals pedals;



//below are the messages defined in the datasheet.  The flash reads look a whole lot like memory offsets.
CAN_message_t known_messages[] = { 
  {KellyreqID,0,3,0,CCP_FLASH_READ,INFO_MODULE_NAME,8,0,0,0,0,0},
  {KellyreqID,0,3,0,CCP_FLASH_READ,INFO_SOFTWARE_VER,2,0,0,0,0,0},
  {KellyreqID,0,3,0,CCP_FLASH_READ,CAL_TPS_DEAD_ZONE_LOW,1,0,0,0,0,0},
  {KellyreqID,0,3,0,CCP_FLASH_READ,CAL_TPS_DEAD_ZONE_HIGH,1,0,0,0,0,0},
  {KellyreqID,0,3,0,CCP_FLASH_READ,CAL_BRAKE_DEAD_ZONE_LOW,1,0,0,0,0,0},
  {KellyreqID,0,3,0,CCP_FLASH_READ,CAL_BRAKE_DEAD_ZONE_HIGH,1,0,0,0,0,0},
  {KellyreqID,0,1,0,CCP_A2D_BATCH_READ1,0,0,0,0,0,0,0},
  {KellyreqID,0,1,0,CCP_A2D_BATCH_READ2,0,0,0,0,0,0,0},
  {KellyreqID,0,1,0,CCP_MONITOR1,0,0,0,0,0,0,0},
  {KellyreqID,0,1,0,CCP_MONITOR2,0,0,0,0,0,0,0},
  {KellyreqID,0,2,0,COM_SW_ACC,COM_READING,0,0,0,0,0,0},
  {KellyreqID,0,2,0,COM_SW_BRK,COM_READING,0,0,0,0,0,0},
  {KellyreqID,0,2,0,COM_SW_REV,COM_READING,0,0,0,0,0,0}
};

bool motorProcessMessage(CAN_message_t &message){
  motor.processMessage(message);
  return true;
}
bool pedalsProcessMessage(CAN_message_t &message){
  pedals.processMessage(message);
  Serial.println("New Pedal Data");
  Serial.print("Brake: ");
  Serial.println(pedals.getBrake());
  
  return true;
}


void setup(){

  pinMode(brakeDacPin, OUTPUT);
  pinMode(throttleDacPin, OUTPUT);
  pinMode(thSwOut, OUTPUT);
  pinMode(brSwOut, OUTPUT);
  pinMode(reSwOut, OUTPUT);

  pinMode(redLightInPin, INPUT);
  pinMode(greenLightInPin, INPUT);
  
  Serial.begin(9600);

  Serial.println("Manager for the kelly motor controller");

  CANbus.begin();

  CAN_filter_t pedalFilter;
  pedalFilter.rtr = 0;
  pedalFilter.ext = 0;
  pedalFilter.id = ManagerID;

  CAN_filter_t KellyReturn;
  KellyReturn.rtr = 0;
  KellyReturn.ext = 0;
  KellyReturn.id = KellyresID;

  //CAN_filter_t vectorHostFilter;
  //vectorHostFilter.rtr = 0;
  //vectorHostFilter.ext = 0;
  //vectorHostFilter.id = DEF_RESPONSE_ID;

  CANbus.setFilter(pedalFilter,0);
  CANbus.setFilter(KellyReturn,1);
  //fill the remaining filters to prevent ack.
  for (int i = 2; i < 8; ++i)
  {
    CANbus.setFilter(pedalFilter,i);
  }

  //set the callback functions
  canbus.set_callback(KellyReturn.id, &motorProcessMessage);
  canbus.set_callback(pedalFilter.id, &pedalsProcessMessage);
}

int messageNumber = 0;


void loop(){
  /**** copy the pots to the DACs ****/
  analogWrite(brakeDacPin, 255*pedals.getBrake());
  analogWrite(throttleDacPin, 255*pedals.getThrottle());
  digitalWrite(brSwOut, pedals.getBrake());
  digitalWrite(reSwOut, pedals.getReverse());
  digitalWrite(thSwOut, pedals.getThrottle());
  
  bool redLightState = digitalRead(redLightInPin);
  bool greenLightState = digitalRead(greenLightInPin);

  /**** Send a new request to the controller ****/
  if(!motor.get_waiting()){
    CAN_message_t outbound = known_messages[messageNumber];
    if(motor.request(outbound)){
      //Serial.print("sent message ");
      //Serial.println(messageNumber);
      //canDump(&outbound);
      if(++messageNumber > 12){
        messageNumber = 0;
        //dumpStats();        
      }
    }
  }

  /**** check for new messages ****/
  CAN_message_t message;
  if(canbus.receive(message)){
    //canDump(&message);

    if(motor.get_intercepted()){
      if(motor.get_process_error()){
        Serial.println("process failed");
      }else{
        //Serial.println("process success");
      }
    }else{
      //Serial.println("No response from Kelly");
    }
  }

  /**** transmit to the dash ****/
  CAN_message_t telem_message = {dashID,0,8, 0, 0,0,0,0,0,0,0,0};

  uint16_t reportRPM = motor.get_mech_rpm();

  uint8_t telemBits = 0;
  if(redLightState) telemBits |= 1<< redLightBit;
  if(greenLightState) telemBits |= 1<< greenLightBit;

  telem_message.buf[0] = ((reportRPM>>8) & 0xFF);   //bit packing for the RPM
  telem_message.buf[1] = (reportRPM & 0xFF);
  telem_message.buf[2] = motor.get_battery_voltage(); //pack voltage (raw from the kelly)
  telem_message.buf[3] = motor.get_current_pc();      //controller current in percent (raw)
  telem_message.buf[4] = 255*pedals.getThrottle();    //throttle repeated from pedals
  telem_message.buf[5] = 255*pedals.getBrake();       //brake repeated from pedals

  telem_message.buf[6] = telemBits;            //bitmap of additional info
  //add some switches and status bits (including the red and green lights)
  telem_message.len = 7;

  canbus.transmit(telem_message);

}



/* the CAN message struct in the FlexCan lib only for reference
typedef struct CAN_message_t {
  uint32_t id; // can identifier
  uint8_t ext; // identifier is extended
  uint8_t len; // length of data
  uint16_t timeout; // milliseconds, zero will disable waiting
  uint8_t buf[8];
} CAN_message_t;
*/



void dumpStats(){
  
  Serial.print("name: ");
  Serial.println(motor.get_module_name());
  
  /*
  Serial.print("version: ");
  printHex(motor.get_module_ver()[0]);
  printHex(motor.get_module_ver()[1]);
  Serial.println("");
  */
  Serial.print("RPM: ");
  Serial.println(motor.get_mech_rpm());
  
  Serial.print("throttle: ");
  Serial.println(motor.get_throttle_pot());
  
}

void printHex(uint8_t val){
  Serial.print("0123456789ABCDEF"[(val>>4)&0x0F]);
  Serial.print("0123456789ABCDEF"[val&0x0F]);
}

void canDump(CAN_message_t &message){
  Serial.print("ID: ");
  Serial.print(message.id);
  //Serial.print("\trtr: ");
  //Serial.print(message.header.rtr);
  Serial.print("\tlen: ");
  Serial.print(message.len);
  Serial.print("\tmessage: "); 
  for(int i=0; i<message.len; i++){
    //Serial.print(message->data[i]);
    //Serial.print((char)message->data[i]);
    printHex(message.buf[i]);    
    Serial.print(" ");
  }
  Serial.println("");  
}
