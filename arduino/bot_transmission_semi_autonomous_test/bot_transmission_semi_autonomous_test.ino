#include <MPU6050.h>
#include <Servo.h>
#include<SPI.h>
#include "printf.h"
#include "RF24.h"

// Configurations
int bot_id = 3;
uint8_t broadcastAddress[6] = "0Node";
uint8_t manualAddress[6] = "3Node";

uint16_t control_instruction_mask = 0x3800;
uint16_t linear_speed_mask = 0x7c0;
uint16_t rotational_speed_mask = 0x3e;
uint8_t max_linear_speed = 255;
uint8_t max_rotational_speed = 100;
unsigned long delay_per_loop = 500;

bool listenBroadcaster = true;
uint8_t pipe;
// Variables used for radio transmission
RF24 radio(7, 8);                 // using pin 7 for the CE  printf_begin();
//  radio.printPrettyDetails(); pin, and pin 8 for the CSN pin
uint16_t ctrl_payload[4] = {0,16384,32768,49152};
int old_ctrl_payload[4] = {0,16384,32768,49152};

// Variables used for Servo Motor
Servo myservo;
// Pins for motors
const int ENA = 3;
const int ENB = 5;
const int IN1 = 4;
const int IN2 = 6;
const int IN3 = 9;
const int IN4 = 10;


// Variables used for IMU sensor
//MPU6050 mpu;

//temp variables

unsigned long old_time = 0;
void setup() {
  serial_init();  // Serial Connection
  servo_init();   // Motors
  radio_init();   // radio Transreceiver
  uint16_t temp = old_ctrl_payload[bot_id-1];
  Serial.println(temp);
}

void loop() {

    if (millis() >= old_time + delay_per_loop){
  receive_data();
//  Serial.println();
  old_time = millis();
    }
//    decode_control_message(); 
}
void receive_data(){
//   uint16_t temp = old_ctrl_payload[3];
//   Serial.println(temp);
   if(radio.available(&pipe)){
    Serial.println(pipe);
    if(!listenBroadcaster && pipe == 1){
      Serial.println("message from broadcaster, not listen to broadcaster");
      radio.flush_rx();
      return;
    }
   
    Serial.println(F("radio: Received Control Command"));
//      Serial.println("before radio.read");
      radio.read(&ctrl_payload,sizeof(ctrl_payload));
      
//      Serial.println("after radio.read");
//      if(temp != ctrl_payload[bot_id -1]){
//        Serial.println("new cmd");
//        Serial.println(ctrl_payload[bot_id-1]);
        decode_control_message(); 
//      }
//       else{
//        Serial.println("same command");
//      }
   }

  

}
void serial_init() {
  Serial.begin(9600);
  while (!Serial) {} // some boards need to wait to ensure access to serial over USB
  Serial.println(F("Serial: Initiated"));
}

void radio_init() {
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(ctrl_payload));
  radio.openReadingPipe(1,broadcastAddress);
  radio.openReadingPipe(2,manualAddress);
  radio.startListening();
  // Uncomment following lines to view details about the connection
  printf_begin();
  radio.printPrettyDetails();
  Serial.println(F("radio: Initiated"));
 radio.setAutoAck(false);
}

void servo_init() {
  myservo.attach(2);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println(F("Servo: Initiated"));
}

void move_forward(int motor_speed) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print(F("Servo: Moving forwards with speed "));
  Serial.println(motor_speed);
}

void move_backward(int motor_speed) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.print(F("Servo: Moving backwards with speed "));
  Serial.println(motor_speed);
}

void halt() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println(F("Servo: Stopping"));
}

void move_right(int motor_speed) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.print(F("Servo: Moving right with speed "));
  Serial.println(motor_speed);
}
void move_left(int motor_speed) {
  analogWrite(ENA, motor_speed);
  analogWrite(ENB, motor_speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print(F("Servo: Moving left with speed "));
  Serial.println(motor_speed);
}
int drop_package() {
  // TODO: Implement functionality to drop the package

  // Send confirmation of package drop
  Serial.println("in drop package, dropping initiated");
  halt();
                for (int pos = 90; pos >= 0; pos -= 1) { // goes from 0 degrees to 180 degrees
                  // in steps of 1 degree
                  myservo.write(pos);              // tell servo to go to position in variable 'pos'
                  delay(15);                       // waits 15 ms for the servo to reach the position
                  Serial.println(pos);
                }
                radio.flush_rx();
                delay(1000);
                for (int pos = 0; pos <= 90; pos += 1) { // goes from 180 degrees to 0 degrees
                  myservo.write(pos);   // tell servo to go to position in variable 'pos'
                  radio.flush_rx();
                  delay(15);                       // waits 15 ms for the servo to reach the position
                  Serial.println(pos);
                }
  Serial.println("dropping completed");
  return 1;
}

void decode_control_message() {
  // 3rd,4th,5th bits are instructions
  uint16_t temp = ctrl_payload[bot_id -1];
  old_ctrl_payload[bot_id-1] = temp;
  uint8_t control_instruction = (control_instruction_mask & temp) >> 11;
  uint8_t linear_speed = (int) ((linear_speed_mask & temp) >> 6) * (max_linear_speed) / (31);
  uint8_t rotational_speed = (int) ((rotational_speed_mask & temp) >> 1) * (max_rotational_speed) / (31);

  if (control_instruction == 0) {
    // Halt
    halt();
  }
  else if (control_instruction == 1) {
    // Drop package
    drop_package();
  }
  else if (control_instruction == 4) {
    // Move forwards
    move_forward(linear_speed);
  }
  else if (control_instruction == 5) {
    // Move right
    move_right(rotational_speed);
  }
  else if (control_instruction == 6) {
    // Move backwards
    move_backward(linear_speed);
  }
  else if (control_instruction == 7) {
    // Move left
    move_left(rotational_speed);
  }
  else if (control_instruction == 2){
    listenBroadcaster = false;
    radio.closeReadingPipe(1);
    Serial.println("listening to manual");
  }
  else if(control_instruction == 3){
    listenBroadcaster = true;
  radio.openReadingPipe(1,broadcastAddress);
    Serial.println("listening to broadcaster");
  }

}
