/* OWI Robotic Arm Control

This sketch is modified from the original instructables code, which may be
found at this URL:
http://www.instructables.com/files/orig/FMK/LNCG/GUPCC2S9/FMKLNCGGUPCC2S9.zip?utm_source=pdf&utm_campaign=files

This version of the code first upgrades the code to match the Adafruit
Motor Shield v2 library interfaces (classes/methods), and re-organizes it
a bit to better allow the user to tune the potentiometers, by running the
motors one at a time.

You can also use/modify this code base to help you correlate the arm
joint zero-positions you've chosen with the potentiometer values that
result from how you've mounted them on your arm.
*/

#include <Adafruit_MotorShield.h>

// Instantiate the motor shield board and all the motors.  Note that the
// motor numbering depends upon how you've wired your motor leads into
// the Adafruit motor shield board.
Adafruit_MotorShield shield = Adafruit_MotorShield();
Adafruit_DCMotor* elbow2_motor   = shield.getMotor(1);
Adafruit_DCMotor* elbow1_motor   = shield.getMotor(2);
Adafruit_DCMotor* shoulder_motor = shield.getMotor(3);
Adafruit_DCMotor* turret_motor   = shield.getMotor(4);

// These are experimentally determined values for the analog input
// values read from the potentiometers.
const int elbow2_pos0   =  17; // analog in 4
const int elbow1_pos0   = 114; // analog in 3
const int shoulder_pos0 =  60; // analog in 2
const int turret_pos0   = 508; // analog in 1;

// how accurately do we need to match the potentiometer position?
const int epsilon = 5;

// variables to hold the current values of the analog sensor input
// read from the potentiometers; note that the initial values are
// meaningless here, as the first thing we do is read the analog
// inputs
int turret_pos = 0;
int shoulder_pos = 0;
int elbow1_pos = 0;
int elbow2_pos = 0;

const uint8_t defaultSpeed = 100; // default motor speed (range: 0-255)
const int motorRunDurationMs = 500; // run each motor for this many milliseconds

uint8_t lastDirection = RELEASE; // direction to run the motors
uint8_t currentMotor = 0; // 0 is all motors, 1-4 are the motor{1,2,3,4} variables

// Take input from the IDE serial console -- input terminates in a
// carriage return.  This sets the direction for the motors to run, and
// return from this function indicates that the user wants to run the
// next iteration of the loop.
void input()
{
  int incomingByte = 0;	// for incoming serial data
  Serial.println("Enter next command: ");
  
  while (incomingByte != 13) { // 13 is the ASCII code for "enter" (aka carriage return)
    if (Serial.available() > 0) {   
		  // read the incoming byte:
		  incomingByte = Serial.read();

		  // say what you got -- turn on for debug output
		  //Serial.print("I received: ");
		  //Serial.println(incomingByte, DEC);

      // it's not a carriage return -- so determine if it's a character we're
      // using to specify motor selection or direction
      switch (incomingByte) {
        case 48: // ascii "0"
          currentMotor = 0; // means run all motors
          Serial.println("Setting motor selection to all motors.");
          break;
        case 49: // ascii "1"
          currentMotor = 1;
          Serial.println("Setting motor selection to elbow2.");
          break;
        case 50: // ascii "2"
          currentMotor = 2;
          Serial.println("Setting motor selection to elbow1.");
          break;
        case 51: // ascii "3"
          currentMotor = 3;
          Serial.println("Setting motor selection to shoulder.");
          break;
        case 52: // ascii "4"
          currentMotor = 4;
          Serial.println("Setting motor selection to turret.");
          break;
        case 70:  // ascii "F"
        case 102: // ascii "f"
          lastDirection = FORWARD;
          Serial.println("Setting direction forward.");
          break;
        case 66: // ascii "B"
        case 98: // ascii "b"
          lastDirection = BACKWARD;
          Serial.println("Setting direction backward.");
          break;
        case 13: // enter [carriage return]
          // do nothing here -- we'll exit the read loop
          break;
        default:
          Serial.println("Unknown command.");
      }
    }
  }
}

void setup() {
  // initialize the serial port
  Serial.begin(9600); // set up Serial library at 9600 bps
  Serial.println("Robot arm test!");
  Serial.println("Enter 0 for all motors, 1 for elbow2 motor, 2 for elbow1 motor, 3 for shoulder and 4 for turret.");
  Serial.println("Enter F for Forward, B for Backward.");
  Serial.println("Then hit return for next motion.");
  Serial.println("Example:   1F<enter>");
  
  // initialize the motor shield board
  shield.begin(); // initialize the motor shield board
  stopMotors(); // make sure all motors are stopped
}

// Displays analog sensor (potentiometer) values to the user
// via the serial console.
void displaySensorValues() {
  // read the values
  turret_pos = analogRead(0);    // read the input pin 0
  shoulder_pos = analogRead(1);  // read the input pin 1
  elbow1_pos = analogRead(2);    // read the input pin 2
  elbow2_pos = analogRead(3);    // read the input pin 3
  // display them
  Serial.print("Turret = ");
  Serial.print(turret_pos);
  Serial.print(" Shoulder = ");
  Serial.print(shoulder_pos);
  Serial.print(" Elbow1 = ");
  Serial.print(elbow1_pos);
  Serial.print(" Elbow2 = ");
  Serial.println(elbow2_pos);
}

// turn the specified motors on and run it/them
void runMotors()
{
  if (currentMotor == 0) { // run all motors simultaneously
    // set the speed on all motors to defaultSpeed (range: 0-255)
    elbow2_motor->setSpeed(defaultSpeed);     
    elbow1_motor->setSpeed(defaultSpeed);
    shoulder_motor->setSpeed(defaultSpeed);
    turret_motor->setSpeed(defaultSpeed);
    elbow2_motor->run(lastDirection);
    elbow1_motor->run(lastDirection);
    shoulder_motor->run(lastDirection);
    turret_motor->run(lastDirection);
  } else if (currentMotor == 1) {
    elbow2_motor->setSpeed(defaultSpeed);
    elbow2_motor->run(lastDirection);
  } else if (currentMotor == 2) {
    elbow1_motor->setSpeed(defaultSpeed);
    elbow1_motor->run(lastDirection);
  } else if (currentMotor == 3) {
    shoulder_motor->setSpeed(defaultSpeed);
    shoulder_motor->run(lastDirection);
  } else if (currentMotor == 4) {
    turret_motor->setSpeed(defaultSpeed);
    turret_motor->run(lastDirection);
  }
}

void stopMotors() {
  // turn off motors
  elbow2_motor->setSpeed(0);
  elbow1_motor->setSpeed(0);   
  shoulder_motor->setSpeed(0);    
  turret_motor->setSpeed(0);
  elbow2_motor->run(RELEASE);
  elbow1_motor->run(RELEASE);
  shoulder_motor->run(RELEASE);
  turret_motor->run(RELEASE);
}

void loop() {
  // Start by displaying all the current sensor values
  displaySensorValues();
  delay(100);

  // Read the next user input (should be forward or
  // backward directional input)
  input();  // wait for user to hit return
  
  // Now we're going to move all the motors a bit so we can
  // see how the motor direction and sensor direction correlate
  runMotors();
  delay(motorRunDurationMs);
  stopMotors();
}

