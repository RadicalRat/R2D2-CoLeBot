#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
const byte address[6] = "64479"; // Set address to 64479 (Team's unique signal code)
RF24 radio(9, 8); // initialize radio on pins 9 and 8

// DC motor pins -> Motor 1 = Left ; Motor 2 = Right
#define motor1 6
#define motor2 5
#define motor1_direction1 A3  //Forward
#define motor1_direction2 A2  //Reverse
#define motor2_direction1 A5  //Forward
#define motor2_direction2 A4  //Reverse


// Servo Pins
#define ServoClawPin 4
#define ServoArmPin 3

// Buzzer Pins
#define Buzzer 

//Motor initializations
Servo armservo;
Servo clawservo;

int motor1_pwm = 0;
int motor2_pwm = 0;
bool motor1_state1 = LOW;   //Next 4 lines are motor direction variables
bool motor1_state2 = LOW;
bool motor2_state1 = LOW;
bool motor2_state2 = LOW;

int servo_arm_angle = 45;
int servo_claw_angle = 90;

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

void setup() {    //still need to set up the dc motors and buttons in here
  // Radio Setup
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, address);
  radio.startListening();

  //Servos
  armservo.attach(ServoArmPin); // Servo signal wire on pin 3 and 5
  clawservo.attach(ServoClawPin);
  armservo.write(servo_arm_angle); // write the desired servo angle (servoangle) to the servo motor
  clawservo.write(servo_claw_angle); // write the desired servo angle (servoangle) to the servo motor
  delay(20);  // every servoname.write(angle); function call needs a 20 ms delay or timer to update servo position

  //Dc motors
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor1_direction1, OUTPUT);
  pinMode(motor1_direction2, OUTPUT);
  pinMode(motor2_direction1, OUTPUT);
  pinMode(motor2_direction2, OUTPUT);

  // Open serial port
  Serial.begin(9600);
}

void loop() {

if (radio.available()) { //if a signal is available
    radio.read(&inputdata, sizeof(DataPacket)); // Read incoming data and store in datapacket datastructure
  }
  int JoyLY = inputdata.JoyLY;
  int JoyLX = inputdata.JoyLX;
  int JoyRX = inputdata.JoyRX;
  int JoyRY = inputdata.JoyRY;

  int JoyLBT = inputdata.JoyLBT;
  bool JoyRBT = inputdata.JoyRBT;
  bool B1T = inputdata.B1T;
  bool B2T = inputdata.B2T;

  // print input variables read from the reciever
  SerialPrint();

  if (B2T == 0) {   //This is statement will act as our kill switch (If B2T = 0 then we will not be able to control the robot)

    // Servo claw
    servo_claw_control(JoyRBT);

    //Servo Arm
    servo_arm_control(JoyLBT);

    //Motor 1
    dc_motor_control(JoyLY, motor1_state1, motor1_state2, motor1_direction1, motor1_direction2, motor1);
    //Motor 2 - Reverse placement of state and direction variables so that this motor will spin in opposite direction than motor one with the same input
    dc_motor_control(JoyRY, motor2_state1, motor2_state2, motor2_direction1, motor2_direction2, motor2);
  }


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
void servo_arm_control(int JoyLBT) {

  if (JoyLBT == 0) {    //Position 0
    servo_arm_angle = 120;
  }
  else if (JoyLBT == 1) {    //Position 1
    servo_arm_angle = 35;
  }
  else if (JoyLBT == 2) {    //Position 2
    servo_arm_angle = 140;
  }
  armservo.write(servo_arm_angle);  // Change servo angle
  delay(20);
}

// <DONE> Claw Servo Function 

int MaxClaw = 121;
int MinClaw = 70;
void servo_claw_control(bool JoyRBT) {
  if (JoyRBT == 0) {
    servo_claw_angle = MinClaw;   //Open Position
  }
  else if (JoyRBT == 1) {
    servo_claw_angle = servo_claw_angle + 2;
  }
  servo_claw_angle = constrain(servo_claw_angle,MinClaw,MaxClaw);
  clawservo.write(servo_claw_angle);   // Change servo angle
  delay(20);
}

// Home Button Function
/*int home_button(bool B1T, int JoyLBT) {
  if (B1T == 1){
    JoyLBT = 0;
    servo_arm_angle = 0;
    armservo.write(servo_arm_angle);
    B1T = 0;
  }
}*/

// <DONE> DC motor control function
void dc_motor_control(int joystick_value, bool state1, bool state2, const int dir1_pin, const int dir2_pin, const int motor_pin) {
  if (joystick_value > 600) {   //Forward
    int spin_speed = map(joystick_value, 600, 1023, 0, 255);
    state1 = HIGH;
    state2 = LOW;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
  else if (joystick_value < 400) {    //Reverse
    int spin_speed = map(joystick_value, 400, 0, 0, 255);  //Need to invert the value so that the speed increases as the joystick value decreases
    state1 = LOW;
    state2 = HIGH;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
  else if (joystick_value > 400 && joystick_value < 600) {
    int spin_speed = 0;
    state1 = LOW;
    state2 = LOW;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
}


