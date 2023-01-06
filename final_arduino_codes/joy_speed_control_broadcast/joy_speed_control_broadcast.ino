//this works with broadcast message


#include <SPI.h>
#include <RF24.h>
#include "printf.h"


RF24 radio(7, 8);
const byte address[6] = "1Node";

int VRx = A0;
int VRy = A1;
int SW = 2;

int xPosition = 0;
int yPosition = 0;
int servo = 0;
int mapX = 0;
int mapY = 0;
int message;
int broadCastMessage[4] = {0,16384,32768,49152};


int rotational_speed = 15;
int linear_speed = 15;
int bot_id = 1;
int instruction = 0;

uint16_t rotateRightCommand;
uint16_t rotateLeftCommand;
uint16_t moveForwardCommand;
uint16_t haltCommand;
uint16_t droppingCommand;

const int buttonPin = 4;
int button;

void setup() {
  Serial.begin(9600);

  pinMode(VRx, INPUT);
  pinMode(VRy, INPUT);
  pinMode(SW, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  radio_init();

  initiate_commands(bot_id, linear_speed, rotational_speed);

}

void loop() {
  xPosition = analogRead(VRx);
  yPosition = analogRead(VRy);
  servo = digitalRead(SW);
  mapX = map(xPosition, 0, 1023, -512, 512);
  mapY = map(yPosition, 0, 1023, -512, 512);


  Serial.println(button);
  button = digitalRead(buttonPin);
  
  if (button == 0) {
    Serial.println("button pressed");
    if (mapX > 400) {
      Serial.println("linear speed increasing");
      linear_speed = linear_speed + 1;
      if (linear_speed > 31) {
        Serial.println("Reached Maximum Linear Speed");
        linear_speed = 31;
      }
      delay(1000);
    }
    if (mapX < -400) {
      Serial.println("linear speed decreasing");
      linear_speed = linear_speed - 1;
      if (linear_speed <= 0) {
        Serial.println("Reached Minimum Linear Speed");
        linear_speed = 0;
      }
      delay(1000);
    }
    if (mapY > 400) {
      Serial.println("angular speed increasing");
      rotational_speed = rotational_speed + 1;
      if (rotational_speed > 31) {
        Serial.println("Reached Maximum Angular Speed");
        rotational_speed = 31;
      }
      delay(1000);
    }
    if (mapY < -400) {
      Serial.println("angular speed decreasing");
      rotational_speed = rotational_speed - 1;
      if (rotational_speed <= 0) {
        Serial.println("Reached Minimum Angular Speed");
        rotational_speed = 0;
      }
      delay(1000);
    }
  }
  else {
    if (mapX < -400) {
      instruction = 6; //backward
      Serial.println("Backward Command");
    }
    else if (mapX > 400) {
      instruction = 4; //forward
      Serial.println("Forward Command");
    }
    else if (mapY < -400) {
      instruction = 7;  //left
      Serial.println("Rotate Left Command");
    }
    else if (mapY > 400) {
      instruction = 5; //right
      Serial.println("Rotate Right Command");
    }
    else if (servo == 0) {
      instruction = 1; //servo
      Serial.println("Dropping Command");
    }
    else {
      instruction = 0; //stop
      Serial.println("Stoping command");
    };
  }

  message = encode_command(bot_id - 1, instruction, linear_speed, rotational_speed);
  transmit_data();
  Serial.print(message);
  Serial.println();
  delay(500);
}

void initiate_commands(uint8_t bot_id, uint16_t fw_speed, uint16_t rt_speed) {
  rotateRightCommand = encode_command(bot_id, 5, fw_speed, rt_speed);
  rotateLeftCommand = encode_command(bot_id, 7, fw_speed, rt_speed);
  moveForwardCommand = encode_command(bot_id, 4, fw_speed, rt_speed);
  haltCommand = encode_command(bot_id, 0, fw_speed, rt_speed);
  droppingCommand = encode_command(bot_id-1, 1, fw_speed, rt_speed);
}


int encode_command(uint8_t bot_id, uint8_t instruction, uint16_t fw_speed, uint16_t rt_speed) {
  uint16_t encodded_cmd = 0;
  encodded_cmd = encodded_cmd | bot_id;

  encodded_cmd = encodded_cmd << 3;
  encodded_cmd = encodded_cmd | instruction;

  encodded_cmd = encodded_cmd << 5;
  encodded_cmd = encodded_cmd | fw_speed;

  encodded_cmd = encodded_cmd << 5;
  encodded_cmd = encodded_cmd | rt_speed;

  encodded_cmd = encodded_cmd << 1;
  return encodded_cmd;
}


bool transmit_data(){
  broadCastMessage[bot_id-1] = message;
  bool report = radio.write(&broadCastMessage, sizeof(broadCastMessage));   // transmit & save the report

  if (report) {
    Serial.println("[controller 2]Radio: transmission successful");
    return true;
  }
  else {
    Serial.println( "[controller 2]Radio: Transmission failed or timed out");      // payload was not delivered
    return false;
  }
}


void radio_init() {
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println("[controller 2]radio hardware is not responding!!");
    while (1) {} // hold in infinite loop
  }
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(sizeof(broadCastMessage));
  radio.openWritingPipe(address);
  radio.stopListening();
   printf_begin();
  radio.printPrettyDetails();
  Serial.println("Radio: Initiated");
}
