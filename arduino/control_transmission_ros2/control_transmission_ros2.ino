
#include<SPI.h>
#include "printf.h"
#include "RF24.h"
#include<ros.h>
#include<grid3/simpleImu.h>
#include<grid3/botCommand.h>
#include<grid3/droppingCompleted.h>
#include <std_msgs/Empty.h>

//ros part
//ros node

ros::NodeHandle nh;
uint8_t priority[4] = {0,0,0,0};
//msgs
grid3::botCommand rosCommand;
//grid3::simpleImu tempImu;
// grid3::commenceDropping commenceDropping;
grid3::droppingCompleted droppingCompleted;


void cmdSubCb(const grid3::botCommand& temp){
  nh.loginfo("in cmd callback");
  rosCommand.botId= temp.botId;
  rosCommand.instruction = temp.instruction;
  rosCommand.forwardSpeed = temp.forwardSpeed;
  rosCommand.rotationalSpeed = temp.rotationalSpeed;
  if(temp.instruction == 1){
    nh.loginfo("1");
  }
  if(temp.instruction == 4){
    nh.loginfo("4");
  }
  if(temp.instruction == 5){
    nh.loginfo("5");
  }
  if(temp.instruction == 7){
    nh.loginfo("7");
  }

//  update_acknowledgement_payloads();
}
ros::Subscriber<grid3::botCommand> rosCmdSub("grid3/controller/cmd2", &cmdSubCb);

void messageCb( const std_msgs::Empty& toggle_msg){
  nh.loginfo("in led callback");
//  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );


//ros::Publisher imuPub("grid3/imu", &tempImu);
ros::Publisher droppingCompletedPub("grid3/droppingCompleted", &droppingCompleted);


// Configurations
uint8_t address[][6] = {"1Node","2Node","3Node","4Node"};
uint16_t bot_id_mask = 0xc000;


// Variables used for radio transmission
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
struct IMU_PayloadStruct{
  int accelX;
  int accelY;
  int accelZ;
  int gyroX;
  int gyroY;
  int gyroZ;
  bool dropped;
};
IMU_PayloadStruct received_payload;
uint16_t ctrl_payload;
uint8_t pipe; 

//temp variables

char stringCmd[16];

void setup() {
  ros_init();
  radio_init();
  radio.flush_tx();
  radio.flush_tx();
}

void loop() {
    nh.loginfo("in loop");
   receive_data();
   ros_loop();
  uint16_t encodded_cmd = 0;
//     radio.flush_tx();
     if(radio.writeAckPayload(4, &encodded_cmd, sizeof(uint16_t))){
      nh.loginfo("d1");
//      radio.flush_tx();
     }
     else{
//      nh.loginfo("n1");
     }

     if(radio.writeAckPayload(3, &encodded_cmd, sizeof(uint16_t))){
      nh.loginfo("d2");
//      radio.flush_tx();
     }
     else{
//      nh.loginfo("n2");
     }

     
     if(radio.writeAckPayload(2, &encodded_cmd, sizeof(uint16_t))){
      nh.loginfo("d3");
     }
     else{
//      nh.loginfo("n3");
     }

     
     if(radio.writeAckPayload(1, &encodded_cmd, sizeof(uint16_t))){
      nh.loginfo("d4");
     }
     else{
//      nh.loginfo("n4");
     }

}

void radio_init() {
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    nh.loginfo("radio hardware is not responding!!");
    while (1) {} // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  for(int i=0;i<=3;i++) {
    radio.openReadingPipe(i+1, address[i]);
  }
  radio.startListening();
  radio.flush_tx();
  // Uncomment following lines to view details about the connection
//  printf_begin();
//  radio.printPrettyDetails();
  nh.loginfo("Radio: Initiated");

  //debugging
  
}

void ros_init(){
  nh.initNode();
  nh.subscribe(rosCmdSub);
  nh.subscribe(sub);
  
//  nh.advertise(imuPub);
  nh.advertise(droppingCompletedPub);
  
  for(int i=0;i<4;i++){
    droppingCompleted.status[i] = false;
  }
}

void receive_data(){
  if (radio.available(&pipe)) {                    // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
    radio.read(&received_payload, bytes);                  // fetch payload from FIFO
    nh.loginfo("Radio: Received ");
    if (pipe==0) {nh.loginfo("0");}
    if (pipe==1) {nh.loginfo("1");}
    if (pipe==2) {nh.loginfo("2");}
    if (pipe==3) {nh.loginfo("3");}
    if (pipe==4) {nh.loginfo("4");}
    if (pipe==5) {nh.loginfo("5");}
//    nh.loginfo(bytes);                           // print the size of the payload
//    nh.loginfo(" bytes on pipe ");
//    nh.loginfo(String('pipe'));                            // print the pipe number
//    nh.loginfo(" : ");
//    nh.loginfo(received_payload.accelX);
//    nh.loginfo(received_payload.accelY);
//    nh.loginfo(received_payload.accelZ);
//    nh.loginfo(received_payload.gyroX);
//    nh.loginfo(received_payload.gyroY);
//    nh.loginfo(received_payload.gyroZ);
//    nh.loginfo("dropped:");
//    nh.loginfo(droppingCompleted);
  }
}

void update_acknowledgement_payloads(){
        uint16_t encodded_cmd = 0;
        encodded_cmd = encodded_cmd | rosCommand.botId;
        
        encodded_cmd = encodded_cmd << 3; 
        encodded_cmd = encodded_cmd | rosCommand.instruction;
        
        encodded_cmd = encodded_cmd << 5; 
        encodded_cmd = encodded_cmd | rosCommand.forwardSpeed;

        encodded_cmd = encodded_cmd << 5; 
        encodded_cmd = encodded_cmd | rosCommand.rotationalSpeed;

        encodded_cmd = encodded_cmd << 1; 
//        nh.loginfo("Radio: Sending control signal to bot ");
//        radio.writeAckPayload(rosCommand.botId+1, &encodded_cmd, sizeof(encodded_cmd));
//        nh.loginfo("Radio: Sending control signal to bot ");
//        printCmd(encodded_cmd);
}

void printCmd(uint16_t encodded_cmd){
  uint16_t bit_mask = 1;
  bit_mask = bit_mask << 15;
  nh.loginfo("in printCmd");
  for(int i = 0;i<16;i++){
    if(encodded_cmd & bit_mask){
      nh.loginfo("1");
    }
    else{
      nh.loginfo("0"); 
    }
    bit_mask = bit_mask >> 1;
  }
}

void ros_loop(){
//  tempImu.botId = pipe;
//  tempImu.wx = received_payload.gyroX;
//  tempImu.wy = received_payload.gyroY;
//  tempImu.wz = received_payload.gyroZ;
//  tempImu.ax = received_payload.accelX;
//  tempImu.ay = received_payload.accelY;
//  tempImu.az = received_payload.accelZ;
//  imuPub.publish(&tempImu);
  
  droppingCompleted.status[pipe]=received_payload.dropped;
  droppingCompletedPub.publish(&droppingCompleted);
  nh.spinOnce();
  
}
