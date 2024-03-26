#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
//heelo
const byte address[6] = "64479"; // Set address to 64479 (Team's unique signal code)
RF24 radio(9, 8); // initialize radio on pins 9 and 8

//Motor initializations
int motor_pwm = 0;
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
  clawservo.attach(5);
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

void servo_arm_control() {
  JoyLBT++;
  if (JoyLBT > 2) {
    JoyLBT = 0;
  }
  if (JoyLBT == 0) {
    servo_arm_angle = 0;
  }
  else if (JoyLBT == 1) {
    servo_arm_angle = 90;
  }
  else if (JoyLBT == 2) {
    servo_arm_angle = 180;
  }
  armservo.write(servo_arm_angle);
}
