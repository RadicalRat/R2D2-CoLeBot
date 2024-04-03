#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Initialize Variables
#define JoyLYPin A4
#define JoyLXPin A5
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
int Armstate = 0; // Position state for the arm
bool JoyRBT = 0; // Right Joystick Button Toggle State BOOL
bool B2T = 0; // Button 2 Toggle State BOOL
};

DataPacket inputdata;

int JoyLY = 0; // Left Joystick Y value
int JoyLX = 0; // Left Joystick X value
int JoyRY = 0; // Right Joystick Y value
int JoyRX = 0; // Right Joystick X value

bool JoyLBP = 0; // Left Joystick Button Previous State BOOL
bool JoyLBC = 0; // Left Joystick Button Current State BOOL
int Armstate = 0; // Controller Variable for the state of the Arm

bool JoyRBP = 0; // Right Joystick Button Previous State BOOL
bool JoyRBC = 0; // Right Joystick Button Current State BOOL
bool JoyRBT = 0; // Right Joystick Button Toggle State BOOL

bool B1P = 0; // Button 1 Previous State BOOL
bool B1C = 0; // Button 1 Current State BOOL

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
  JoyLY = (1023-analogRead(JoyLYPin));
  JoyLX = analogRead(JoyLXPin);
  JoyRY = (1023-analogRead(JoyRYPin));
  JoyRX = analogRead(JoyRXPin);


  //  Armstate 3 State Toggle; Looks at Left Joystick button and home button
  //  This button will act as a counter that does not exceed 2

  JoyLBP = JoyLBC;
  JoyLBC = digitalRead(JoyLBPin);
  B1P = B1C;
  B1C = digitalRead(B1Pin);
  if (JoyLBP > JoyLBC){
    Armstate++;   //  Increment the Button value by 1 when it is pressed
    delay(100);
  }
  if (Armstate > 2) {  // If the button value exceeds 2, then it will be reset back to 0
    Armstate = 0;
  }
  if (B1P > B1C){ // If Button 1 is pressed (Home Button) -> will set Armstate to 2
    Armstate = 2;
    delay(150);
  }



  //Joystick Right button True Toggle -> Claw Control
  JoyRBP = JoyRBC;
  JoyRBC = digitalRead(JoyRBPin);
  if (JoyRBP > JoyRBC){
    JoyRBT = !JoyRBT;
    delay(150);
  }

  //Button 2 True Toggle -> Killswitch
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
  Serial.print(Armstate);
  Serial.print("  ");
  Serial.print(JoyRBT);
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
  inputdata.Armstate = Armstate;
  inputdata.JoyRBT = JoyRBT;
  inputdata.B2T = B2T;

}

// Function to send the datapacket to the reciever module
void Transmit() {
  radio.write(&inputdata, sizeof(DataPacket)); //send Data 
  delay(50);
}