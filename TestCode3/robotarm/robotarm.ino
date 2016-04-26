/* OWI Robotic Arm Control

This sketch is modified from the original instructables code, which may be
found at this URL:
http://www.instructables.com/files/orig/FMK/LNCG/GUPCC2S9/FMKLNCGGUPCC2S9.zip?utm_source=pdf&utm_campaign=files

This version of the code test driving an individua arm joint to a
particular position, based on potentiometer feedback.
*/

#include <Adafruit_MotorShield.h>

// NOTE: I've tried to add YOURS_NEEDS_UPDATE tags in this code
// to indicate where you will/may have to change values to match
// your particular set-up -- but no guarantees I haven't missed
// some. :)

// Note: We're going to use arrays of values, as it makes it easier
// in the code to match up motors and sensor values.  Here, we
// define indexes for the arrays, so we don't have all these 0-3
// values running around the code, getting confused with the 1-4
// indexing of the motors/analog-reads from the motor shield board.
const int ELBOW2_INDEX   = 0;
const int ELBOW1_INDEX   = 1;
const int SHOULDER_INDEX = 2;
const int TURRET_INDEX   = 3;
// All arrays of values in the subsequent code here must be defined
// and accessed such that the order of their elements aligns with
// these *_INDEX variables.

// Instantiate the motor shield board and all the motors into an
// array of Adafruit_DCMotor objects.
Adafruit_MotorShield shield = Adafruit_MotorShield();
// YOURS_NEEDS_UPDATE: Note that the numbering used to fetch each
// motor from the shield depends upon how you've wired your motor
// leads into the Adafruit motor shield board.  You will need to
// check and perhaps change the 1-4 values used here.
Adafruit_DCMotor* motorArray[4] = {
  shield.getMotor(1), // elbow2
  shield.getMotor(2), // elbow1
  shield.getMotor(3), // shoulder
  shield.getMotor(4)  // turret
};

// YOURS_NEEDS_UPDATE: These are experimentally determined values for
// the analog input values read from the potentiometers which correspond
// to the zero-position values for the joints.  Yours will be different.
const int jointPositions_0[4] = { 17, 114, 60, 530 };

// YOURS_NEEDS_UPDATE: These are the experimentally determined values for
// the potentiometer reads corresponding to various position of the arm --
// aka, where we want to drive the joint positions to in order to be ready
// to pick up the brick, put down a brick, etc.
const int jointPositions_desired[3][4] = {
  {  17, 114,  60, 530 }, // A: zero position
  { 118, 166, 366, 530 }, // B: pick up brick position (all) & put down #1
  { 118, 166, 366, 550 }, // C: put down #2
  { 118, 166, 366, 750 }, // D: put down #3
};

// how accurately do we need to match the potentiometer position?
// that is, how far off can we be and still call it good?
// YOURS_NEEDS_UPDATE: This value experimentally determined by from the
// potentiometers I'm using -- depending upon what you're using, you may
// need to change this.
const int epsilon = 5;

// YOURS_NEEDS_UPDATE: These are sufficient for me.  It is unlikely that
// you will need to change them, since you're running with the same OWI
// arm, but you can experiment here.
const uint8_t defaultSpeed = 100; // default motor speed (range: 0-255)
const uint8_t slowSpeed    =  50;
const int motorRunDurationMs = 500; // run each motor for this many milliseconds

// major index to use in the jointPositions_desired selection
int currentDesiredJointsIndex = 0; 
// 9 is all motors; 0-3 are the *_INDEX values as defined above.
uint8_t currentSelectedMotorIndex = 0; 

// variables to hold the current values of the analog sensor input
// read from the potentiometers; note that the initial values are
// meaningless here, as the first thing we do is read the analog
// inputs
int jointPositions_current[4] = { 0, 0, 0, 0 };

int readFeedback(int motorIdx) {
  // YOURS_NEEDS_UPDATE: which pin to call analogRead on here (in order to
  // get the read-value for the potentiometer position) will depend upon how
  // you've wired the potentiometers to your motor shield board.
  int returnVal = 0; // bad-index value
  switch (motorIdx) {
    case ELBOW2_INDEX:
      jointPositions_current[ELBOW2_INDEX]   = analogRead(3);  // read the input pin 3
      returnVal = jointPositions_current[ELBOW2_INDEX];
      break;
    case ELBOW1_INDEX:
      jointPositions_current[ELBOW1_INDEX]   = analogRead(2);  // read the input pin 2
      returnVal = jointPositions_current[ELBOW1_INDEX];
      break;
    case SHOULDER_INDEX:
      jointPositions_current[SHOULDER_INDEX] = analogRead(1);  // read the input pin 1
      returnVal = jointPositions_current[SHOULDER_INDEX];
      break;
    case TURRET_INDEX:
      jointPositions_current[TURRET_INDEX]   = analogRead(0);  // read the input pin 0
      returnVal = jointPositions_current[TURRET_INDEX];
      break;
    default:
      break; // unknown -- do nothing
  }
  return returnVal;
}

uint8_t determineMotorDirection(int motorIdx, int currJointPos, int desrdJointPos)
{
  // calculate how much off we are, and if we're within epsilon, call it good
  int diff = desrdJointPos - currJointPos;
  if (((diff >= 0) && (diff <=  epsilon)) ||
      ((diff <  0) && (diff >= -epsilon))) {
    return RELEASE;
  }

  // which direction we're loop-closing on depends upon which motor we have
  // YOURS_NEEDS_UPDATE: Depending upon how you have defined positive and
  // negative directions on your potentiometers, you may need to swap these
  // around.
  switch (motorIdx) {
    case ELBOW2_INDEX: // if elbow2, forward is positive
    case ELBOW1_INDEX: // if elbow1, forward is positive
    case SHOULDER_INDEX: // if shoulder, forward is positive
      if (diff > 0) return FORWARD;
      else return BACKWARD;
      break;
    case TURRET_INDEX: // if turret, forward (right) is negative
      if (diff > 0) return BACKWARD;
      else return FORWARD;
      break;
    default:
      return RELEASE; // unknown -- default to do nothing
  }
}

// motoridx is one of the *_INDEX values defined at the start
// of this file; desiredPosArray is the major index in the 
// jointPositions_desired array, used to determine which set
// of joint position values we're driving towards
void driveMotorToPos(int motorIdx, int desiredPosArray) {
  // get the motor we want to drive
  Adafruit_DCMotor* currMotor = motorArray[motorIdx];
  
  // get where we want to drive this joint to
  int desrdJointPos = jointPositions_desired[desiredPosArray][motorIdx];

  // drive to that joint -- but don't go on forever if we're having
  // trouble
  int loopCount = 0;
  while (loopCount < 5) {
    loopCount = loopCount + 1;
    int currJointPos = readFeedback(motorIdx);
    uint8_t direction = determineMotorDirection(motorIdx, currJointPos, desrdJointPos);
    currMotor->setSpeed(slowSpeed);
    currMotor->run(direction);
    delay(100);
  }
}

// Take input from the IDE serial console -- input terminates in a
// carriage return.  This sets the direction for the motors to run, and
// return from this function indicates that the user wants to run the
// next iteration of the loop.
void input()
{
  int incomingByte = 0;	// for incoming serial data
  Serial.println(""); // output a blank line -- makes it easier to read
  Serial.println("Enter next command: ");
  
  while (incomingByte != 13) { // 13 is the ASCII code for "enter" (aka carriage return)
    if (Serial.available() > 0) {   
		  // read the incoming byte:
		  incomingByte = Serial.read();

		  // say what you got -- turn on for debug output
		  //Serial.print("I received: ");
		  //Serial.println(incomingByte, DEC);

      // determine if it's a character we're using to specify motor selection or direction
      switch (incomingByte) {
        case 57: // ascii "9"
          currentSelectedMotorIndex = 9; // means run all motors
          Serial.println("Setting motor selection to all motors.");
          break;
        case 48: // ascii "0"
          currentSelectedMotorIndex = ELBOW2_INDEX;
          Serial.println("Setting motor selection to elbow2.");
          break;
        case 49: // ascii "1"
          currentSelectedMotorIndex = ELBOW1_INDEX;
          Serial.println("Setting motor selection to elbow1.");
          break;
        case 50: // ascii "2"
          currentSelectedMotorIndex = SHOULDER_INDEX;
          Serial.println("Setting motor selection to shoulder.");
          break;
        case 51: // ascii "3"
          currentSelectedMotorIndex = TURRET_INDEX;
          Serial.println("Setting motor selection to turret.");
          break;
        case 65: // ascii "A"
        case 97: // ascii "a"
          currentDesiredJointsIndex = 0;
          Serial.println("Setting joint selection to zero set.");
          break;
        case 66: // ascii "B"
        case 98: // ascii "b"
          currentDesiredJointsIndex = 1;
          Serial.println("Setting joint selection to one set.");
          break;
        case 67: // ascii "C"
        case 99: // ascii "c"
          currentDesiredJointsIndex = 2;
          Serial.println("Setting joint selection to two set.");
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
  Serial.println("Enter 9 for all motors, 0 for elbow2 motor, 1 for elbow1 motor, 2 for shoulder and 3 for turret.");
  Serial.println("Enter A for the first (joint-zero position), B for second (pick up brick), and C for the third (put down brick).");
  Serial.println("Then hit return for next motion.");
  Serial.println("Example:   0A<enter>");
  
  // handle setting the initial state of the motor shield board
  shield.begin(); // initialize the motor shield board
  stopMotors(); // make sure all motors are stopped
}

// Displays analog sensor (potentiometer) values to the user
// via the serial console.
void displaySensorValues() {
  // read the values
  readFeedback(TURRET_INDEX);
  readFeedback(SHOULDER_INDEX);
  readFeedback(ELBOW1_INDEX);
  readFeedback(ELBOW2_INDEX);
  // display them
  Serial.print("Turret = ");
  Serial.print(jointPositions_current[TURRET_INDEX]);
  Serial.print(" Shoulder = ");
  Serial.print(jointPositions_current[SHOULDER_INDEX]);
  Serial.print(" Elbow1 = ");
  Serial.print(jointPositions_current[ELBOW1_INDEX]);
  Serial.print(" Elbow2 = ");
  Serial.println(jointPositions_current[ELBOW2_INDEX]);
}

void stopMotors() {
  // turn off motors
  motorArray[ELBOW2_INDEX]->setSpeed(0);
  motorArray[ELBOW1_INDEX]->setSpeed(0);   
  motorArray[SHOULDER_INDEX]->setSpeed(0); 
  motorArray[TURRET_INDEX]->setSpeed(0);
  motorArray[ELBOW2_INDEX]->run(RELEASE);
  motorArray[ELBOW1_INDEX]->run(RELEASE);
  motorArray[SHOULDER_INDEX]->run(RELEASE);
  motorArray[TURRET_INDEX]->run(RELEASE);
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
  //runMotors();
  //delay(motorRunDurationMs);
  driveMotorToPos(currentSelectedMotorIndex, currentDesiredJointsIndex);
  stopMotors();
}

