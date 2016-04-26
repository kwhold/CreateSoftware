/* OWI Robotic Arm Control

This sketch is modified from the original instructables code, which may be
found at this URL:
http://www.instructables.com/files/orig/FMK/LNCG/GUPCC2S9/FMKLNCGGUPCC2S9.zip?utm_source=pdf&utm_campaign=files

This version of the code merely upgrades the code to match the Adafruit
Motor Shield v2 library interfaces (classes/methods).  It keeps the same
functionality as the original demo code from the Instructables.  It is
useful for verifying that your hardware is working correctly, while you
continue to develop the programmatic arm control in a different rev of
the code base.

You can also use/modify this code base to help you correlate the arm
joint zero-positions you've chosen with the potentiometer values that
result from how you've mounted them on your arm.

Note: Depending upon your actual hardware, you may get only some of the
motors running on this, due to current overdraw (if you're running off
USB power for the Arduino/motor shield).  Suggest going to the
one-motor-at-a-time version of this test code.
*/

#include <Adafruit_MotorShield.h>

Adafruit_MotorShield shield = Adafruit_MotorShield();
Adafruit_DCMotor* motor1 = shield.getMotor(1);  // Instantiate all the motors
Adafruit_DCMotor* motor2 = shield.getMotor(2);
Adafruit_DCMotor* motor3 = shield.getMotor(3);
Adafruit_DCMotor* motor4 = shield.getMotor(4);
int val0 = 0;           // variable to store the Arm 1 sensor
int val1 = 0;           // variable to store the Arm 2 sensor
int val2 = 0;           // variable to store the Arm 3 sensor
int val3 = 0;           // variable to store the Arm 4 sensor


void input() {   // this is the subroutine that waits for the user to hit enter
  int incomingByte = 0;	// for incoming serial data
  motor1->setSpeed(0);     // stop the motor
  Serial.println("Hit return for next motion");   
  while (incomingByte != 13) { // 13 is the ASCII code for "enter"
    if (Serial.available() > 0) {   
		  // read the incoming byte:
		  incomingByte = Serial.read();

		  // say what you got:
		  Serial.print("I received: ");
		  Serial.println(incomingByte, DEC);
	  }
  }
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Robot arm test!");
  shield.begin();
  motor1->setSpeed(100);     // set the speed to 100/255
  motor2->setSpeed(100);     // do the same for the others...
  motor3->setSpeed(100);    
  motor4->setSpeed(100);
}

void loop() {
  start:  // Start by displaying all the current sensor values
  Serial.println("Start!");
  val0 = analogRead(0);    // read the input pin 0
  val1 = analogRead(1);    // read the input pin 0
  val2 = analogRead(2);    // read the input pin 0
  val3 = analogRead(3);    // read the input pin 0
        
  Serial.print("1 = ");
  Serial.print(val0);             // debug value
  Serial.print("  2 = ");
  Serial.print(val1);             // debug value
  Serial.print("  3 = ");
  Serial.print(val2);             // debug value
  Serial.print("  4 = ");
  Serial.println(val3);             // debug value
  delay(100);
  
  if (val0 > 500) {  // If base is not at end, move to end
    while (val0 > 500) {
      val0 = analogRead(0);    // read the input pin 0
      Serial.print("Value = ");
      Serial.println(val0);             // debug value
      motor1->setSpeed(100);     // set the speed to 100/255
      motor1->run(FORWARD);
      delay(100);
    }
  }
  
  input();  // wait for user to hit return
  
  // Now we're going to move all the motors forward a bit so we can
  // see how the motor direction and sensor direction correlate
  
  motor1->setSpeed(100);     // set the speed to 100/255
  motor1->run(FORWARD);
  motor2->setSpeed(100);     // set the speed to 100/255
  motor2->run(FORWARD);
  motor3->setSpeed(100);     // set the speed to 100/255
  motor3->run(FORWARD);
  motor4->setSpeed(100);     // set the speed to 100/255
  motor4->run(FORWARD);
  delay (1000);
  motor1->setSpeed(0);       // turn off motors
  motor2->setSpeed(0);   
  motor3->setSpeed(0);    
  motor4->setSpeed(0);   
  
  // Now read the sensors to see how they changed

  val0 = analogRead(0);    // read the input pin 0
  val1 = analogRead(1);    // read the input pin 1
  val2 = analogRead(2);    // read the input pin 2
  val3 = analogRead(3);    // read the input pin 3
        
  Serial.print("1 = ");    // report the new readings
  Serial.print(val0);             
  Serial.print("  2 = ");
  Serial.print(val1);             
  Serial.print("  3 = ");
  Serial.print(val2);            
  Serial.print("  4 = ");
  Serial.println(val3);          
  delay(100);

  input();  // wait for user to hit return
  
    // Now we're going to move all the motors back to where they started
  
  motor1->setSpeed(100);     // set the speed to 100/255
  motor1->run(BACKWARD);
  motor2->setSpeed(100);     // set the speed to 100/255
  motor2->run(BACKWARD);
  motor3->setSpeed(100);     // set the speed to 100/255
  motor3->run(BACKWARD);
  motor4->setSpeed(100);     // set the speed to 100/255
  motor4->run(BACKWARD);
  delay (1000);
  motor1->setSpeed(0);       // turn off motors
  motor2->setSpeed(0);   
  motor3->setSpeed(0);    
  motor4->setSpeed(0);   
  
  // Now read the sensors to see how they changed

  val0 = analogRead(0);    // read the input pin 0
  val1 = analogRead(1);    // read the input pin 1
  val2 = analogRead(2);    // read the input pin 2
  val3 = analogRead(3);    // read the input pin 3
        
  Serial.print("1 = ");    // report the new readings
  Serial.print(val0);             
  Serial.print("  2 = ");
  Serial.print(val1);             
  Serial.print("  3 = ");
  Serial.print(val2);            
  Serial.print("  4 = ");
  Serial.println(val3);          
  delay(100);

  input();  // wait for user to hit return

  goto start;
}

