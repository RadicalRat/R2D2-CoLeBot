#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
const byte address[6] = "64479"; // Set address to 64479 (Team's unique signal code)
RF24 radio(9, 8); // initialize radio on pins 9 and 8

// DC motor pins
const int motor1 = 6
const int motor2 = 5
const int motor1_direction1 = A3   //Forward
const int motor1_direction2 = A2    //Reverse
const int motor2_direction1 = A5  //Forward
const int motor2_direction2 = A4   //Reverse


//Motor initializations
int motor1_pwm = 0;
int motor2_pwm = 0;
bool motor1_state1 = LOW;   //Next 4 lines are motor direction variables
bool motor1_state2 = LOW;
bool motor2_state1 = LOW;
bool motor2_state2 = LOW;

int servo_arm_angle = 0;
int servo_claw_angle = 0;

struct DataPacket {
/*                                         
  Data Structure for recieving the input variables to the robot, called using inputdata
*/
int JoyLY = 0; // Left Joystick Y value
int JoyLX = 0; // Left Joystick X value
int JoyRY = 0; // Right Joystick Y value
int JoyRX = 0; // Right Joystick X value

int JoyLBT = 0; // Left Joystick Button Toggle State INT
bool JoyRBT = 0; // Right Joystick Button Toggle State BOOL
bool B1T = 0; // Button 1 Toggle State BOOL
bool B2T = 0; // Button 2 Toggle State BOOL

};

DataPacket inputdata;

void setup() {
  // Radio Setup
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, address);
  radio.startListening();

  //Servos
  armservo.attach(3); // Servo signal wire on pin 3 and 5
  clawservo.attach(4);
  armservo.write(servo_arm_angle); // write the desired servo angle (servoangle) to the servo motor
  clawservo.write(servo_claw_angle); // write the desired servo angle (servoangle) to the servo motor
  delay(20);  // every servoname.write(angle); function call needs a 20 ms delay or timer to update servo position

  // Open serial port
  Serial.begin(9600);
}

void loop() {

if (radio.available()) { //if a signal is available
    radio.read(&inputdata, sizeof(DataPacket)); // Read incoming data and store in datapacket datastructure
  }
  
  // print input variables read from the reciever
  SerialPrint();

  // wait 50 ms to avoid too many updates happening too quickly
  delay(50);
}


// Function to directly print the data in the Datapacket Datastructure
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
// Arm Servo Function
void servo_arm_control() {
  JoyLBT++;  // Change condition

  if (JoyLBT > 2) {
    JoyLBT = 0;
  }
  if (JoyLBT == 0) {    //Position 0
    servo_arm_angle = 0;
  }
  else if (JoyLBT == 1) {    //Position 1
    servo_arm_angle = 90;
  }
  else if (JoyLBT == 2) {    //Position 2
    servo_arm_angle = 180;
  }
  armservo.write(servo_arm_angle);  // Change servo angle
  delay(20);
}

// Claw Servo Function
void servo_claw_control() {
  JoyRBT = !JoyRBT;    // Change condition

  if (JoyRBT == 0) {
    servo_claw_angle = 0;   //Open Position
  }
  if (JoyRBT == 1) {
    servo_claw_angle = 90;    //Closed Position
  }

  clawservo.write(servo_claw_angle);   // Change servo angle
  delay(20);
}
