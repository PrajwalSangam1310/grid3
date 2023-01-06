
#include<SPI.h>
#include "printf.h"
#include "RF24.h"
#include<ros.h>
#include<grid3/botCommand.h>


//ros part
//ros node

ros::NodeHandle nh;
grid3::botCommand rosCommand;

uint16_t tr_payload[4] = {0,16384,32768,49152};
//struct command_payloads{
//  uint16_t bot1;
//  uint16_t bot2;
//  uint16_t bot3;
//  uint16_t bot4;
//}


void cmdSubCb(const grid3::botCommand& temp){
//  nh.loginfo("in cmd callback");
  rosCommand.botId= temp.botId;
  rosCommand.instruction = temp.instruction;
  rosCommand.forwardSpeed = temp.forwardSpeed;
  rosCommand.rotationalSpeed = temp.rotationalSpeed;
//  if(temp.instruction == 0){
//    nh.loginfo("0");
//  }
//  if(temp.instruction == 1){
//    nh.loginfo("1");
//  }
//  if(temp.instruction == 4){
//    nh.loginfo("4");
//  }
//  if(temp.instruction == 5){
//    nh.loginfo("5");
//  }
//  if(temp.instruction == 7){
//    nh.loginfo("7");
//  }
  encode_command(temp.botId);
}
ros::Subscriber<grid3::botCommand> rosCmdSub("grid3/controller/cmdBroadcast", &cmdSubCb);

//ros::Publisher imuPub("grid3/imu", &tempImu);


// Configurations
uint8_t address[6] = "0Node";
uint16_t bot_id_mask = 0xc000;


// Variables used for radio transmission
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

void setup() {
  ros_init();
  radio_init();
}

void loop() {
//   nh.loginfo("in loop");
   ros_loop();
   transmit_data();
   radio.flush_tx();
}

void radio_init() {
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    nh.loginfo("radio hardware is not responding!!");
    while (1) {} // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(tr_payload));
  radio.openWritingPipe(address);
  radio.stopListening();
  nh.loginfo("Radio: Initiated");
  radio.setAutoAck(false);
  
}

void ros_init(){
  nh.initNode();
  nh.subscribe(rosCmdSub);
//  nh.subscribe(sub);
}  

void encode_command(int bot_id){
        uint16_t encodded_cmd = 0;
        encodded_cmd = encodded_cmd | rosCommand.botId;
        
        encodded_cmd = encodded_cmd << 3; 
        encodded_cmd = encodded_cmd | rosCommand.instruction;
        
        encodded_cmd = encodded_cmd << 5; 
        encodded_cmd = encodded_cmd | rosCommand.forwardSpeed;

        encodded_cmd = encodded_cmd << 5; 
        encodded_cmd = encodded_cmd | rosCommand.rotationalSpeed;

        encodded_cmd = encodded_cmd << 1; 
        tr_payload[bot_id] = encodded_cmd;
}

bool transmit_data(){
//   unsigned long start_timer = micros();                         // start the timer
  bool report = radio.write(&tr_payload, sizeof(tr_payload));   // transmit & save the report
//  unsigned long end_timer = micros();                           // end the timer

  if (report) {
//    tr_payload.counter = tr_payload.counter + 1;
//    nh.loginfo("transmission successful");
    return true;
  }
  else {
//    nh.loginfo("Radio: Transmission failed or timed out");      // payload was not delivered
    return false;
  }
}

void ros_loop(){
  nh.spinOnce();
  delay(50);
}
