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

int motor1_pwm = 0;     //Motor PWM values
int motor2_pwm = 0;
bool motor1_state1 = LOW;   //Next 4 lines are motor direction variables
bool motor1_state2 = LOW;
bool motor2_state1 = LOW;
bool motor2_state2 = LOW;

int servo_arm_angle = 45;
int servo_claw_angle = 90;

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

void setup() {    //still need to set up the dc motors and buttons in here
  // Radio Setup
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, address);
  radio.startListening();

  //Servos Setup
  armservo.attach(ServoArmPin);
  clawservo.attach(ServoClawPin);
  armservo.write(servo_arm_angle); // write the desired servo angle (servoangle) to the servo motor
  clawservo.write(servo_claw_angle); // write the desired servo angle (servoangle) to the servo motor
  delay(20);

  //Dc Motors Setup
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
  // Setting transmitted input values to variables for later use
  int JoyLY = inputdata.JoyLY;
  int JoyLX = inputdata.JoyLX;
  int JoyRX = inputdata.JoyRX;
  int JoyRY = inputdata.JoyRY;

  int Armstate = inputdata.Armstate;
  bool JoyRBT = inputdata.JoyRBT;
  bool B2T = inputdata.B2T;

  // print input variables read from the reciever
  //SerialPrint();

  if (B2T == 0) {   //This is statement will act as our kill switch (If B2T = 0 then we will not be able to control the robot)

    // Servo claw Control
    servo_claw_control(JoyRBT);

    //Servo Arm Control
    servo_arm_control(Armstate);

    //Motor 1 Control Fast
    dc_motor_control(JoyLY, motor1_state1, motor1_state2, motor1_direction1, motor1_direction2, motor1);
    //Motor 2 Control Fast
    dc_motor_control(JoyRY, motor2_state1, motor2_state2, motor2_direction1, motor2_direction2, motor2);



  }
  else {    //This will make sure the motors are off if the kill switch button is pressed
    motor1_pwm = 0;   //Set spin speed to 0
    motor2_pwm = 0;
    motor1_state1 = LOW;   //Set all Motor States to 0
    motor1_state2 = LOW;
    motor2_state1 = LOW;
    motor2_state2 = LOW;
    //Motor 1
    digitalWrite(motor1_direction1, motor1_state1);
    digitalWrite(motor1_direction2, motor1_state2);
    analogWrite(motor1, motor1_pwm);
    //Motor 2
    digitalWrite(motor2_direction1, motor2_state1);
    digitalWrite(motor2_direction2, motor2_state2);
    analogWrite(motor2, motor2_pwm);

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
  Serial.print(inputdata.Armstate);
  Serial.print("  ");
  Serial.print(inputdata.JoyRBT);
  Serial.print("  ");
  Serial.print(inputdata.B2T);
  Serial.println("  ");
}

// Arm Servo Function
void servo_arm_control(int Armstate) {

  if (Armstate == 0) {    //Position 0
    servo_arm_angle = 120;
  }
  else if (Armstate == 1) {    //Position 1
    servo_arm_angle = 35;
  }
  else if (Armstate == 2) {    //Position 2
    servo_arm_angle = 140;
  }
  armservo.write(servo_arm_angle);  // Change servo angle
  delay(20);
}

// <DONE> Claw Servo Function 

int MaxClaw = 127;     //Claw angle constraints
int MinClaw = 70;
void servo_claw_control(bool JoyRBT) {
  if (JoyRBT == 0) {
    servo_claw_angle = MinClaw;   //Open Position
  }
  else if (JoyRBT == 1) {
    servo_claw_angle = servo_claw_angle + 2;    //Increments until max claw angle is reached
  }
  servo_claw_angle = constrain(servo_claw_angle,MinClaw,MaxClaw);    //Makes sure the min and max claw angles are not exceeded
  clawservo.write(servo_claw_angle);   // Change servo angle
  delay(20);
}


// <DONE> DC motor control function
void dc_motor_control(int joystick_value, bool state1, bool state2, const int dir1_pin, const int dir2_pin, const int motor_pin) {
  if (joystick_value < 450) {   //Forward
    int spin_speed = map(joystick_value, 450, 0, 0, 255);   //Note: Joysticks are upside-down in the housing
    state1 = HIGH;
    state2 = LOW;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
  else if (joystick_value > 550) {    //Reverse
    int spin_speed = map(joystick_value, 550, 1023, 0, 255);  //Note: Joysticks are upside-down in the housing
    state1 = LOW;
    state2 = HIGH;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
  else if (joystick_value > 450 && joystick_value < 550) {   //Stationary
    int spin_speed = 0;
    state1 = LOW;
    state2 = LOW;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
}

// <DONE> DC motor control slow
void dc_motor_control_slow(int joystick_value, bool state1, bool state2, const int dir1_pin, const int dir2_pin, const int motor_pin) {
  if (joystick_value < 400) {   //Forward
    int spin_speed = map(joystick_value, 450, 0, 0, 128);   //Note: Joysticks are upside-down in the housing
    state1 = HIGH;
    state2 = LOW;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
  else if (joystick_value > 600) {    //Reverse
    int spin_speed = map(joystick_value, 550, 1023, 0, 128);  //Note: Joysticks are upside-down in the housing
    state1 = LOW;
    state2 = HIGH;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
  else if (joystick_value > 400 && joystick_value < 600) {   //Stationary
    int spin_speed = 0;
    state1 = LOW;
    state2 = LOW;
    digitalWrite(dir1_pin, state1);
    digitalWrite(dir2_pin, state2);
    analogWrite(motor_pin, spin_speed);
  }
}

