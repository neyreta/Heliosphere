// Include the servo library:
#include <Servo.h>
// Create a new servo object:
Servo myservo;
Servo myservo2;


// Define the servo pin:
#define servoPin 9
#define servoPin2 6
#define potPin A0
#define potPin2 A1
// Create a variable to store the servo position:
float angle = 0;
float angle2 = 0;

void setup() {
 // Attach the Servo variable to a pin:
 myservo.attach(servoPin);
 myservo2.attach(servoPin2);
 Serial.begin(9600);  // Initialisons la communication serial
}
void loop() {
 // Tell the servo to go to a particular angle: 
angle = analogRead(potPin)*180./1024.;
angle2 = analogRead(potPin2)*180./1024.;
Serial.println(angle);
Serial.println(angle2);
 // Sweep from 0 to 180 degrees:

myservo.write(angle);
myservo2.write(angle2);
delay(15);
}
