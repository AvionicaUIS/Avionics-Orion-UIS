/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#define pyro1 13
#define pyro2 12
#define BUZ 26

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(pyro1, OUTPUT);
  pinMode(pyro2, OUTPUT);
  pinMode(BUZ, OUTPUT);
  delay(5000);
  digitalWrite(BUZ,HIGH);
  delay(1000);
  digitalWrite(BUZ,LOW);
  delay(10000);

  digitalWrite(BUZ,HIGH);
  digitalWrite(pyro1, HIGH);
  delay(8000);
  digitalWrite(pyro1,LOW);
  digitalWrite(BUZ,LOW);
  delay(3000);
  digitalWrite(BUZ,HIGH);
  digitalWrite(pyro2, HIGH);
  delay(8000);
  digitalWrite(pyro2, LOW);
  digitalWrite(BUZ,LOW);
}

// the loop function runs over and over again forever
void loop() {
}
