#include <MPU6050.h>
#include <Servo.h>
#include<SPI.h>
#include "printf.h"
#include "RF24.h"

// Configurations
int bot_id = 1;
uint8_t address[6] = "1Node";
uint16_t control_instruction_mask = 0x3800;
uint16_t linear_speed_mask = 0x7c0;
uint16_t rotational_speed_mask = 0x3e;
uint8_t max_linear_speed = 255;
uint8_t max_rotational_speed = 100;
unsigned long delay_per_loop = 100;

// Variables used for radio transmission
RF24 radio(7, 8);                 // using pin 7 for the CE  printf_begin();
//  radio.printPrettyDetails(); pin, and pin 8 for the CSN pin
uint16_t ctrl_payload = 0;

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
  radio_init();   // Radio Transreceiver
}

void loop() {

  if (millis() >= old_time + delay_per_loop) {
    receive_data();
    old_time = millis();
  }
}
void receive_data() {
  if (radio.available()) {
    radio.read(&ctrl_payload, sizeof(ctrl_payload));
    Serial.print(F("Radio: Received Control Command: "));
    Serial.println(ctrl_payload);
    decode_control_message();
    radio.flush_rx();
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
    Serial.println(F("Radio: Radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(ctrl_payload));
  radio.openReadingPipe(1, address);
  radio.startListening();
  // Uncomment following lines to view details about the connection
  printf_begin();
  radio.printPrettyDetails();
  Serial.println(F("Radio: Initiated"));
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
  halt();
  Serial.println(F("Servo: Dropping Package..."));
  for (int pos = 90; pos >= 0; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }
  delay(1000);
  for (int pos = 0; pos <= 90; pos += 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15 ms for the servo to reach the position
  }
  Serial.println("Servo: Dropping completed!");
  return 1;
}

void decode_control_message() {
  // 3rd,4th,5th bits are instructions
  uint8_t temp_id = ctrl_payload >> 14;
  uint8_t control_instruction = (control_instruction_mask & ctrl_payload) >> 11;
  uint8_t linear_speed = (int) ((linear_speed_mask & ctrl_payload) >> 6) * (max_linear_speed) / (31);
  uint8_t rotational_speed = (int) ((rotational_speed_mask & ctrl_payload) >> 1) * (max_rotational_speed) / (31);

  if (bot_id - 1 != temp_id) {
    return;
  }
  Serial.println(F("Servo: Executing Command..."));
  if (control_instruction == 0) {
    // Halt
    halt();
  }
  else if (control_instruction == 1) {
    // Drop package
    drop_package();
  }
  else if (control_instruction == 4) {
    // Move forwardsflush_rx
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

}
