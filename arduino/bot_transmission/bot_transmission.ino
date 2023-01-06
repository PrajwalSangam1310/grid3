#include <MPU6050.h>
#include <Servo.h>
#include<SPI.h>
#include "printf.h"
#include "RF24.h"

// Configurations
int bot_id = 4;
uint8_t address[][6] = {"1Node", "2Node", "3Node", "4Node"};
uint16_t control_instruction_mask = 0x3800;
uint16_t linear_speed_mask = 0x7c0;
uint16_t rotational_speed_mask = 0x3e;
uint8_t max_linear_speed = 255;
uint8_t max_rotational_speed = 100;
unsigned long delay_per_loop = 100;

// Variables used for radio transmission
RF24 radio(7, 8);                 // using pin 7 for the CE  printf_begin();
//  radio.printPrettyDetails(); pin, and pin 8 for the CSN pin
struct IMU_PayloadStruct {
  int accelX;
  int accelY;
  int accelZ;
  int gyroX;
  int gyroY;
  int gyroZ;
  bool dropped;
};
IMU_PayloadStruct tr_payload = {0, 0, 0, 0, 0, 0, false};
uint16_t ctrl_payload;
uint8_t pipe;


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
MPU6050 mpu;

//temp variables

unsigned long old_time = 0;
void setup() {
  serial_init();  // Serial Connection
  servo_init();   // Motors
 imu_init();     // IMU sensor
  radio_init();   // Radio Transreceiver

  //  memcpy(tr_payload.message, "Hello", 6);
}

void loop() {

    if (millis() >= old_time + delay_per_loop){
  orientation_readings();
  bool transmission_result = transmit_data();
  if (transmission_result) {
    if (check_ack_payload()) {
      decode_control_message();
    }
  }
  Serial.println();
  old_time = millis();
    }
}

void serial_init() {
  Serial.begin(115200);
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
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openWritingPipe(address[bot_id - 1]);
  radio.stopListening();
  // Uncomment following lines to view details about the connection
  printf_begin();
  radio.printPrettyDetails();
  Serial.println(F("Radio: Initiated"));
}

bool transmit_data() {
  unsigned long start_timer = micros();                         // start the timer
  bool report = radio.write(&tr_payload, sizeof(tr_payload));   // transmit & save the report
  unsigned long end_timer = micros();                           // end the timer

  if (report) {
    Serial.print(F("Radio: Transmission successful! "));        // payload was delivered
    Serial.print(F("Time to transmit = "));
    Serial.print(end_timer - start_timer);                      // print the timer result
    Serial.println(F(" us."));
    Serial.print(F("Radio: Sent "));
    Serial.print(tr_payload.accelX);
    Serial.print(tr_payload.accelY);
    Serial.print(tr_payload.accelZ);
    Serial.print(tr_payload.gyroX);
    Serial.print(tr_payload.gyroY);
    Serial.println(tr_payload.gyroZ);
    //    tr_payload.counter = tr_payload.counter + 1;
    return true;
  }
  else {
    Serial.println(F("Radio: Transmission failed or timed out"));      // payload was not delivered
    return false;
  }
}

bool check_ack_payload() {
  if (radio.available(&pipe)) {
    radio.read(&ctrl_payload, sizeof(ctrl_payload));  // get incoming ACK payload
    Serial.print(F("Radio: Recieved "));
    Serial.print(radio.getDynamicPayloadSize());      // print incoming payload size
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);                               // print pipe number that received the ACK
    Serial.print(F(": "));
    Serial.println(ctrl_payload);                       // print incoming message
    return true;
  }
  else {
    Serial.println(F("Radio: Recieved: empty ACK packet")); // empty ACK packet received
    return false;
  }
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
                delay(1000);
                for (int pos = 0; pos <= 90; pos += 1) { // goes from 180 degrees to 0 degrees
                  myservo.write(pos);              // tell servo to go to position in variable 'pos'
                  delay(15);                       // waits 15 ms for the servo to reach the position
                  Serial.println(pos);
                }
  Serial.println("dropping completed");
  return 1;
}

void decode_control_message() {
  // 3rd,4th,5th bits are instructions
  uint8_t control_instruction = (control_instruction_mask & ctrl_payload) >> 11;
  uint8_t linear_speed = (int) ((linear_speed_mask & ctrl_payload) >> 6) * (max_linear_speed) / (31);
  uint8_t rotational_speed = (int) ((rotational_speed_mask & ctrl_payload) >> 1) * (max_rotational_speed) / (31);
  tr_payload.dropped = false;

  if (control_instruction == 0) {
    // Halt
    halt();
  }
  else if (control_instruction == 1) {
    // Drop package
    int temp = drop_package();
    if(temp == 1){
      tr_payload.dropped = true;
    }
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
}

void imu_init() {
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
}

void orientation_readings() {
  Vector normAccel = mpu.readNormalizeAccel();
  Vector normGyro = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll
  // int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  // int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  // int yaw = (atan2(normAccel.ZAxis, sqrt(normAccel.XAxis*normAccel.XAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;

  tr_payload.accelX = normAccel.XAxis;
  tr_payload.accelY = normAccel.YAxis;
  tr_payload.accelZ = normAccel.ZAxis;


  tr_payload.gyroX = normGyro.XAxis;
  tr_payload.gyroY = normGyro.YAxis;
  tr_payload.gyroZ = normGyro.ZAxis;
}
