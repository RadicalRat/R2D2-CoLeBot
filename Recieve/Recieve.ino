#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const byte address[6] = "64479";
RF24 radio(9, 8);

struct DataPacket {

int JoyLY = 0; // Left Joystick Y value
int JoyLX = 0; // Left Joystick X value
int JoyRY = 0; // Right Joystick Y value
int JoyRX = 0; // Right Joystick X value

bool JoyLBT = 0; // Left Joystick Button Toggle State BOOL
bool JoyRBT = 0; // Right Joystick Button Toggle State BOOL
bool B1T = 0; // Button 1 Toggle State BOOL
bool B2T = 0; // Button 2 Toggle State BOOL

};
DataPacket inputdata;
void setup() {
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, address);
  radio.startListening();
  Serial.begin(9600);
}

void loop() {
if (radio.available()) {//if a signal is available
    radio.read(&inputdata, sizeof(DataPacket));
  }
  SerialPrint();
  delay(50);
}

void SerialPrint() {
  Serial.print(inputdata.JoyLY);
  Serial.print("  ");
  Serial.print(inputdata.JoyLX);
  Serial.print("  ");
  Serial.print(inputdata.JoyRY);
  Serial.print("  ");
  Serial.print(inputdata.JoyRX);
  Serial.print("  ");
  Serial.print(inputdata.JoyLBT);
  Serial.print("  ");
  Serial.print(inputdata.JoyRBT);
  Serial.print("  ");
  Serial.print(inputdata.B1T);
  Serial.print("  ");
  Serial.print(inputdata.B2T);
  Serial.println("  ");
}
