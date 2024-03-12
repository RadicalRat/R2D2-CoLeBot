#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define JoyLYPin A5
#define JoyLXPin A4
#define JoyLBPin 2 
#define JoyRYPin A3
#define JoyRXPin A2
#define JoyRBPin 3
#define B1Pin 5
#define B2Pin 6


struct DataPacket {
/*
  Data Structure for transmitting the input variables to the robot, called using inputdata
*/
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

int JoyLY = 0; // Left Joystick Y value
int JoyLX = 0; // Left Joystick X value
int JoyRY = 0; // Right Joystick Y value
int JoyRX = 0; // Right Joystick X value

bool JoyLBP = 0; // Left Joystick Button Previous State BOOL
bool JoyLBC = 0; // Left Joystick Button Current State BOOL
bool JoyLBT = 0; // Left Joystick Button Toggle State BOOL

bool JoyRBP = 0; // Right Joystick Button Previous State BOOL
bool JoyRBC = 0; // Right Joystick Button Current State BOOL
bool JoyRBT = 0; // Right Joystick Button Toggle State BOOL

bool B1P = 0; // Button 1 Previous State BOOL
bool B1C = 0; // Button 1 Current State BOOL
bool B1T = 0; // Button 1 Toggle State BOOL

bool B2P = 0; // Button 2 Previous State BOOL
bool B2C = 0; // Button 2 Current State BOOL
bool B2T = 0; // Button 2 Toggle State BOOL

const byte address[6] = "64479"; // Set address to 64479 (Team's unique signal code)
RF24 radio(9, 8); // initialize radio on pins 9 and 8

void setup() {
  // Radio Setup
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(address);
  radio.stopListening();

  // Open serial port
  Serial.begin(9600);

  // Initializes digital inputs
  pinMode(JoyLBPin,INPUT_PULLUP);
  pinMode(JoyRBPin,INPUT_PULLUP);
  pinMode(B1Pin,INPUT_PULLUP);
  pinMode(B2Pin,INPUT_PULLUP);
}



void loop() {
  // Update joystick values with analog read
  JoyLY = analogRead(JoyLYPin);
  JoyLX = analogRead(JoyLXPin);
  JoyRY = analogRead(JoyRYPin);
  JoyRX = analogRead(JoyRXPin);


  //  Joystick Left button True Toggle
  JoyLBP = JoyLBC;
  JoyLBC = digitalRead(JoyLBPin);
  if (JoyLBP > JoyLBC){
    JoyLBT = !JoyLBT;
    delay(150);
  }

  //Joystick Right button True Toggle
  JoyRBP = JoyRBC;
  JoyRBC = digitalRead(JoyRBPin);
  if (JoyRBP > JoyRBC){
    JoyRBT = !JoyRBT;
    delay(150);
  }

  //Button 1 True Toggle
  B1P = B1C;
  B1C = digitalRead(B1Pin);
  if (B1P > B1C){
    B1T = !B1T;
    delay(150);
  }

  //Button 2 True Toggle
  B2P = B2C;
  B2C = digitalRead(B2Pin);
  if (B2P > B2C){
    B2T = !B2T;
    delay(150);
  }

  // Print inputs into Serial Monitor
  SerialPrint();

  // Updates data in the Datapacket datastructure
  DataUpdate();

  // Transmits the Datapacket
  Transmit();
}


// Function to print out the Input variables in the serial monitor
void SerialPrint() {
  Serial.print(JoyLY);
  Serial.print("  ");
  Serial.print(JoyLX);
  Serial.print("  ");
  Serial.print(JoyRY);
  Serial.print("  ");
  Serial.print(JoyRX);
  Serial.print("  ");
  Serial.print(JoyLBT);
  Serial.print("  ");
  Serial.print(JoyRBT);
  Serial.print("  ");
  Serial.print(B1T);
  Serial.print("  ");
  Serial.print(B2T);
  Serial.println("  ");
}

// Function to update the datapacket
void DataUpdate() {
  // Update Joystick variables into DataPacket
  inputdata.JoyLY = JoyLY;
  inputdata.JoyLX = JoyLX;
  inputdata.JoyRY = JoyRY;
  inputdata.JoyRX = JoyRX;

  //Update Button Toggle variables into DataPacket
  inputdata.JoyLBT = JoyLBT;
  inputdata.JoyRBT = JoyRBT;
  inputdata.B1T = B1T;
  inputdata.B2T = B2T;

}

// Function to send the datapacket to the reciever module
void Transmit() {
  radio.write(&inputdata, sizeof(DataPacket)); //send Data 
  delay(50);
}