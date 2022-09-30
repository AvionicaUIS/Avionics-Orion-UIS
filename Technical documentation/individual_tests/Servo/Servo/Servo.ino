/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
  Written by BARRAGAN and modified by Scott Fitzgerald
*********/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(27);  // attaches the servo on pin 13 to the servo object
}

void loop() {
  delay(5000);
//  (pos = 0; pos <= 90; pos += 1)  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    pos = 90;
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
  
}
