
#include<SPI.h>
#include "printf.h"
#include "RF24.h"

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


void setup() {
  serial_init();
  radio_init();
}

void loop() {
   receive_data();
   update_acknowledgement_payloads();
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
  for(int i=0;i<=3;i++) {
    radio.openReadingPipe(i+1, address[i]);
  }
  radio.startListening();
  // Uncomment following lines to view details about the connection
  //printf_begin();
  //radio.printPrettyDetails();
  Serial.println(F("Radio: Initiated"));
}

void receive_data() {
  if (radio.available(&pipe)) {                    // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
    radio.read(&received_payload, bytes);                  // fetch payload from FIFO
    Serial.print(F("Radio: Received "));
    Serial.print(bytes);                           // print the size of the payload
    Serial.print(F(" bytes on pipe "));
    Serial.print(pipe);                            // print the pipe number
    Serial.print(F(" : "));
    Serial.print(received_payload.accelX);
    Serial.print(received_payload.accelY);
    Serial.print(received_payload.accelZ);
    Serial.print(received_payload.gyroX);
    Serial.print(received_payload.gyroY);
    Serial.println(received_payload.gyroZ);
  }
}
void update_acknowledgement_payloads() {
  if (Serial.available()) {
    ctrl_payload = Serial.parseInt();
    
    Serial.print(F("Serial: Recevied command: "));
    Serial.println(ctrl_payload);
    uint8_t bot_id = ((ctrl_payload & bot_id_mask) >> 14) + 1;
    Serial.print(F("Radio: Sending control signal to bot "));
    Serial.println(bot_id);
    radio.writeAckPayload(bot_id, &ctrl_payload, sizeof(ctrl_payload));
 }
}
