#include <Servo.h>

Servo myServo;
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  myServo.attach(D4);  // WEMOS D1 mini: D4 = GPIO2
  myServo.write(20);
  Serial.println("Servo controller ready. Send angle 0-180.");
}

void loop() {
  if (stringComplete) {
    int angle = inputString.toInt();
    angle = constrain(angle, 0, 180);
    myServo.write(angle);
    Serial.print("Moved to: ");
    Serial.println(angle);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
