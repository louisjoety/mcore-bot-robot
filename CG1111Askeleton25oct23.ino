#include "MeMCore.h"
#define TURNING_TIME_MS 330 // The time duration (ms) for turning

#define TIMEOUT 2000 // Max microseconds to wait; choose according to max distance of wall
#define SPEED_OF_SOUND 340 // Update according to your own experiment
#define ULTRASONIC 12
// If you are using Port 1 of mCore, the ultrasonic sensor uses digital pin 12
// If you are using Port 2 of mCore, the ultrasonic sensor uses digital pin 10

#define LIGHTSENSOR A6 // internally connected to analog pin A6 in mCore

MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2

int status = 0; // global status; 0 = do nothing, 1 = mBot runs

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeBuzzer buzzer; // create the buzzer object
MeRGBLed led(0,30); // Based on hardware connections on mCore; cannot change

uint8_t motorSpeed = 255;
  // Setting motor speed to an integer between 1 and 255
  // The larger the number, the faster the speed

void celebrate() {
// Each of the following "function calls" plays a single tone.
// The numbers in the bracket specify the frequency and the duration (ms) buzzer.tone(392, 200);
buzzer.tone(523, 200);
buzzer.tone(659, 200);
buzzer.tone(784, 200);
buzzer.tone(659, 150);
buzzer.tone(784, 400);
buzzer.noTone();
}

void setup()
{
  // Any setup code here runs only once:
  delay(5000); // Do nothing for 10000 ms = 10 seconds
  led.setpin(13);
  Serial.begin(9600); // to initialize the serial monitor
}

void loop()
{
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC, INPUT);
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  float distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;
  Serial.println(distance);
  
  int sensorState = lineFinder.readSensors(); // read the line sensor's state

  if (sensorState == S1_IN_S2_IN) {
    leftMotor.run(0);
    rightMotor.run(0);
  } else if (distance < 12) {
    leftMotor.run(-255); // Left wheel go forward
    rightMotor.run(180); // Right wheel stops
  } else if (distance > 14) {
    leftMotor.run(-180); // Left wheel go forward
    rightMotor.run(255); // Right wheel stops
  } else {
    leftMotor.run(-255);
    rightMotor.run(255);
  }

}


