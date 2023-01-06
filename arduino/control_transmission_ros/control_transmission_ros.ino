
#include<SPI.h>
#include "printf.h"
#include "RF24.h"
#include<ros.h>
#include<grid3/simpleImu.h>
#include<grid3/botCommand.h>

//ros part
//ros node

ros::NodeHandle nh;

//msgs
grid3::botCommand rosCommand;
grid3::simpleImu tempImu;


void cmdSubCb(const grid3::botCommand& temp){
  rosCommand.botId = temp.botId;
  rosCommand.instruction = temp.instruction;
  rosCommand.forwardSpeed = temp.forwardSpeed;
  rosCommand.rotationalSpeed = temp.rotationalSpeed;
  update_acknowledgement_payloads();

}


ros::Subscriber<grid3::botCommand> rosCmdSub("grid3/botCommand", cmdSubCb);
ros::Publisher imuPub("arduino/imu", &tempImu);


// Configurations
uint8_t address[][6] = {"1Node","2Node","3Node","4Node"};
uint16_t bot_id_mask = 0xc000;


// Variables used for radio transmission
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin
struct IMU_PayloadStruct {
  int accelX;
  int accelY;
  int accelZ;
  int gyroX;
  int gyroY;
  int gyroZ;
};
IMU_PayloadStruct received_payload;
uint16_t ctrl_payload;
uint8_t pipe; 

//temp variables

char stringCmd[16];

void setup() {
  ros_init();
  radio_init();
}


void loop() {
   receive_data();
   ros_loop();
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
  // Uncomment following lines to view details about the connection
  //printf_begin();
  //radio.printPrettyDetails();
  nh.loginfo("Radio: Initiated");
}

void ros_init(){
  nh.initNode();
  nh.subscribe(rosCmdSub);
  nh.advertise(imuPub);
}

void receive_data(){
  if (radio.available(&pipe)) {                    // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
    radio.read(&received_payload, bytes);                  // fetch payload from FIFO
    nh.loginfo("Radio: Received ");
    nh.loginfo(bytes);                           // print the size of the payload
    nh.loginfo(" bytes on pipe ");
    nh.loginfo(pipe);                            // print the pipe number
    nh.loginfo(" : ");
    nh.loginfo(received_payload.accelX);
    nh.loginfo(received_payload.accelY);
    nh.loginfo(received_payload.accelZ);
    nh.loginfo(received_payload.gyroX);
    nh.loginfo(received_payload.gyroY);
    nh.loginfo(received_payload.gyroZ);
  }
}

void update_acknowledgement_payloads() {
    nh.loginfo("Serial: Recevied command: ");
    uint16_t encodded_cmd = 0;
    encodded_cmd = encodded_cmd | rosCommand.botId;
    
    encodded_cmd = encodded_cmd << 3; 
    encodded_cmd = encodded_cmd | rosCommand.instruction;
    
    encodded_cmd = encodded_cmd << 5; 
    encodded_cmd = encodded_cmd | rosCommand.forwardSpeed;

    encodded_cmd = encodded_cmd << 5; 
    encodded_cmd = encodded_cmd | rosCommand.rotationalSpeed;

    encodded_cmd = encodded_cmd << 1; 
    nh.loginfo("Radio: Sending control signal to bot ");
    radio.writeAckPayload(rosCommand.botId, &encodded_cmd, sizeof(encodded_cmd));
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
  tempImu.botId = pipe;
  tempImu.wx = received_payload.gyroX;
  tempImu.wy = received_payload.gyroY;
  tempImu.wz = received_payload.gyroZ;
  tempImu.ax = received_payload.accelX;
  tempImu.ay = received_payload.accelY;
  tempImu.az = received_payload.accelZ;
  imuPub.publish(&tempImu);
  nh.spinOnce();
}
