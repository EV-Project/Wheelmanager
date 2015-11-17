#include <KellyCAN.h>
#include <FlexCAN.h>
#include <CANcallbacks.h>
#include <Pedals.h>

#define PEDAL_BOX_CAN_ID 150

FlexCAN CANbus(1000000);

//static CAN_message_t msg,rxmsg;
//CanBus canbus;

CANcallbacks canbus(&CANbus);
KellyCAN motor(&canbus, 107, 115);
Pedals pedals;


//check these pins, they're all the wrong order
int brakeDacPin = 20;
int throttleDacPin = 21;
int brSwOut = 14;
int reSwOut = 15;
int thSwOut = 16;


/*
typedef struct CAN_message_t {
  uint32_t id; // can identifier
  uint8_t ext; // identifier is extended
  uint8_t len; // length of data
  uint16_t timeout; // milliseconds, zero will disable waiting
  uint8_t buf[8];
} CAN_message_t;
*/

//below are the messages defined in the datasheet.  The flash reads look a whole lot like memory offsets.
CAN_message_t known_messages[] = { 
  {DEF_REQUEST_ID,0,3,0,CCP_FLASH_READ,INFO_MODULE_NAME,8,0,0,0,0,0},
  {DEF_REQUEST_ID,0,3,0,CCP_FLASH_READ,INFO_SOFTWARE_VER,2,0,0,0,0,0},
  {DEF_REQUEST_ID,0,3,0,CCP_FLASH_READ,CAL_TPS_DEAD_ZONE_LOW,1,0,0,0,0,0},
  {DEF_REQUEST_ID,0,3,0,CCP_FLASH_READ,CAL_TPS_DEAD_ZONE_HIGH,1,0,0,0,0,0},
  {DEF_REQUEST_ID,0,3,0,CCP_FLASH_READ,CAL_BRAKE_DEAD_ZONE_LOW,1,0,0,0,0,0},
  {DEF_REQUEST_ID,0,3,0,CCP_FLASH_READ,CAL_BRAKE_DEAD_ZONE_HIGH,1,0,0,0,0,0},
  {DEF_REQUEST_ID,0,1,0,CCP_A2D_BATCH_READ1,0,0,0,0,0,0,0},
  {DEF_REQUEST_ID,0,1,0,CCP_A2D_BATCH_READ2,0,0,0,0,0,0,0},
  {DEF_REQUEST_ID,0,1,0,CCP_MONITOR1,0,0,0,0,0,0,0},
  {DEF_REQUEST_ID,0,1,0,CCP_MONITOR2,0,0,0,0,0,0,0},
  {DEF_REQUEST_ID,0,2,0,COM_SW_ACC,COM_READING,0,0,0,0,0,0},
  {DEF_REQUEST_ID,0,2,0,COM_SW_BRK,COM_READING,0,0,0,0,0,0},
  {DEF_REQUEST_ID,0,2,0,COM_SW_REV,COM_READING,0,0,0,0,0,0}
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
  Serial.begin(9600);

  Serial.println("Manager for the kelly motor controller");

  CANbus.begin();

  CAN_filter_t pedalFilter;
  pedalFilter.rtr = 0;
  pedalFilter.ext = 0;
  pedalFilter.id = PEDAL_BOX_CAN_ID;

  CAN_filter_t KellyReturn;
  KellyReturn.rtr = 0;
  KellyReturn.ext = 0;
  KellyReturn.id = DEF_RESPONSE_ID;

  //CAN_filter_t vectorHostFilter;
  //vectorHostFilter.rtr = 0;
  //vectorHostFilter.ext = 0;
  //vectorHostFilter.id = DEF_RESPONSE_ID;

  CANbus.setFilter(pedalFilter,0);
  CANbus.setFilter(KellyReturn,1);
  for (int i = 2; i < 8; ++i)
  {
    CANbus.setFilter(pedalFilter,i);
  }


  canbus.set_callback(DEF_RESPONSE_ID, &motorProcessMessage);
  canbus.set_callback(PEDAL_BOX_CAN_ID, &pedalsProcessMessage);
}

int messageNumber = 0;


void loop(){
  /**** copy the pots to the DACs ****/
  analogWrite(brakeDacPin, 255*pedals.getBrake());
  analogWrite(throttleDacPin, 255*pedals.getThrottle());
  digitalWrite(brSwOut, pedals.getBrake());
  digitalWrite(reSwOut, pedals.getReverse());
  digitalWrite(thSwOut, pedals.getThrottle());
  

  
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

}






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
