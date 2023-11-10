/*
 * CG1111A A-maze-ing Race Project
 * Team B02-S2-T4
 * Main code for mBot
 */
#include "MeMCore.h"
#include <PID_v1.h>

#define TIMEOUT 2000
#define SPEED_OF_SOUND 340 // for calculating distance based on ultrasonic sensor data
#define ULTRASONIC 12 // port 1

// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 200 // in milliseconds

// Define time delay before taking another LDR reading
#define LDRWait 10 // in milliseconds

#define LDR A0 // LDR sensor pin at A0
#define BUTTON 7

MeLineFollower lineFinder(PORT_2); // assigning lineFinder (black line detector) to RJ25 port 2

MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning rightMotor to port M2
MeBuzzer buzzer; // create the buzzer object
MeRGBLed led(0,30); // internal RGB LED to display colour detected

// define PID parameters
double setpoint = 10.0; // Desired distance from the wall in centimeters

// input of PID: distance from wall measured by ultrasonic sensor
// output of PID: value to add ot subtract from base motor speed for centreing
double input, output;

// PID tuning parameters
double kp = 150;
double ki = 0;
double kd = 0.2;

double baseSpeed = 220; // base speed of motor, PID will add or subtract from this

// variables to hold motor speed values for left and right motors
float motorSpeedL;
float motorSpeedR;

// initialise PID
PID PID(&input,&output,&setpoint,kp,ki,kd,DIRECT);

// Define colour sensor LED pins
int ledArray[] = {A1,A2,A3}; 

// placeholder variables for colour detected
int r = 0;
int g = 0;
int b = 0;

// floats to hold colour arrays
float colourArray[] = {0,0,0};
float blackArray[] = {900.00,966.00,906.00};
float greyDiff[] = {92.00,48.00,95.00};

// for displaying colour values on serial monitor for debugging
char colourStr[3][5] = {"R = ", "G = ", "B = "};

int status = 0; // global status for button

// tune to play at end of maze
void celebrate() {
  // The numbers in the bracket specify the frequency and the duration (ms) buzzer.tone(392, 200);
  // Super Mario Bros Level Complete Jingle
  buzzer.tone(196.00, 200); // G3
  buzzer.tone(261.63, 200); // C4
  buzzer.tone(329.63, 200); // E4
  buzzer.tone(392.00, 200); // G4
  buzzer.tone(523.25, 200); // C5
  buzzer.tone(659.25, 200); // E5
  buzzer.tone(783.99, 600); // G5
  buzzer.tone(659.25, 600); // E5
  buzzer.tone(207.65, 200); // G#3
  buzzer.tone(261.63, 200); // C4
  buzzer.tone(311.13, 200); // Eb4
  buzzer.tone(415.30, 200); // G#4
  buzzer.tone(523.25, 200); // C5
  buzzer.tone(622.25, 200); // Eb5
  buzzer.tone(830.61, 600); // G#5
  buzzer.tone(622.25, 600); // Eb5
  buzzer.tone(233.08, 200); // Bb3
  buzzer.tone(293.66, 200); // D4
  buzzer.tone(349.23, 200); // F4
  buzzer.tone(466.16, 200); // Bb4
  buzzer.tone(587.33, 200); // D5
  buzzer.tone(698.46, 200); // F5
  buzzer.tone(932.33, 600); // Bb5
  buzzer.tone(932.33, 200); // Bb5
  buzzer.tone(932.33, 200); // Bb5
  buzzer.tone(932.33, 200); // Bb5
  buzzer.tone(1046.50, 1200); // C6
  buzzer.noTone();
}

void setup() {
  led.setpin(13); // internal RGB LED is at Pin 13
  pinMode(BUTTON,INPUT); // set button to input
  Serial.begin(9600);
  for (int c = 0; c <= 2; c++) { // set LED pins to output
    pinMode(ledArray[c],OUTPUT);
  }

  // configure PID
  PID.SetMode(AUTOMATIC);
  PID.SetOutputLimits(-250,250);
  PID.SetSampleTime(30);

  // indicate robot is ready to run maze
  led.setColor(255,255,255);
  led.show();
  buzzer.tone(1000,500);
}

int getAvgReading(int times) { // find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  // take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  // calculate the average and return it
  return total/times;
}

// functions to turn the robot based off colour

// turn left once
void turnLeft() {
  leftMotor.run(255); // left motor backwards
  rightMotor.run(255); // right motor forwards
  delay(310); // turn duration, to adjust

  leftMotor.run(-255); // go straight for short while
  rightMotor.run(255);
  delay(100);
}

// turn right once
void turnRight() {
  leftMotor.run(-255); // left motor forwards
  rightMotor.run(-255); // right motor backwards
  delay(300); // turn duration, to adjust

  leftMotor.run(-255); // go straight for short while
  rightMotor.run(255);
  delay(100);
}

// make a u-turn
void uTurn() {
  leftMotor.run(-255); // left motor forwards
  rightMotor.run(-255); // right motor backwards 
  delay(550); // turn duration, to adjust

  leftMotor.run(-255); // go straight for short while
  rightMotor.run(255);
  delay(100);
}

// turn right twice
void turnRightTwice() {
  leftMotor.run(-255); // left motor forwards - first right turn
  rightMotor.run(-255); // right motor backwards
  delay(320); // turn duration, to adjust

  leftMotor.run(-255); // go straight
  rightMotor.run(255);
  delay(700); // straight duration, to adjust

  leftMotor.run(-255); // left motor forwards - second right turn
  rightMotor.run(-255); // right motor backwards
  delay(350); // turn duration, to adjust

  leftMotor.run(-255); // go straight for short while
  rightMotor.run(255);
  delay(100);
}

// turn left twice
void turnLeftTwice() {
  leftMotor.run(255); // left motor backwards  - first left turn
  rightMotor.run(255); // right motor forwards
  delay(320); // turn duration, to adjust

  leftMotor.run(-255); // go straight
  rightMotor.run(255);
  delay(700); // straight duration, to adjust

  leftMotor.run(255); // left motor backwards - second left turn
  rightMotor.run(255); // right motor forwards
  delay(340); // turn duration, to adjust

  leftMotor.run(-255); // go straight for short while
  rightMotor.run(255);
  delay(100);
}

void loop() {
  if (analogRead(BUTTON) < 100) { // maze navigation begins when button is pressed
    status = 1 - status;
    delay(500);
  }

  // ultrasonic sensor read distance to left wall
  pinMode(ULTRASONIC, OUTPUT);
  digitalWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC, INPUT);

  // calculate distance based on interval between sending & receiving sound waves, set to PID input
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  input = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;
  
  int sensorState = lineFinder.readSensors(); // read the line sensor's state

  if (status == 1) { // start when button is pressed
    PID.Compute(); // continually compute PID each loop

    if (sensorState == S1_IN_S2_IN) { // stop at black line
      // stop motors
      leftMotor.run(0);
      rightMotor.run(0);

      // activate colour sensor
      for (int c = 0; c <= 2; c++) {    
        analogWrite(ledArray[c],255); // turn ON LED, red, green & blue, one colour at a time.
        delay(RGBWait);
        colourArray[c] = getAvgReading(5); // get average of 5 readings from LDR

        // the average reading returned minus the lowest value divided by the maximum possible range, 
        // multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
        colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
        analogWrite(ledArray[c],0);  // turn off the current LED colour

        // ensure colour values are within range of 0-255
        if (colourArray[c] > 255) {
          colourArray[c] = 255;
        }
        if (colourArray[c] < 0) {
          colourArray[c] = 0;
        }

        delay(RGBWait);
        Serial.print(colourStr[c]);
        Serial.println(int(colourArray[c])); // print colour values for debugging
      }

      // set colour values of r, g and b to be checked
      r = colourArray[0];
      g = colourArray[1];
      b = colourArray[2];

      // mario coin collecting sound
      // buzzer.tone(987.77, 100); // B4
      // buzzer.tone(1318.51, 300); // E5

      // check r, g & b values if they fall within range of one of the colours
      if (r >= 203 && g >= 86 && g <= 130 && b >= 84 && b <= 127) { // red -- turn left once
        led.setColor(255,0,0); // show red colour
        led.show();
        turnLeft();

      } else if (r >= 86 && r <= 130 && g >= 147 && g <= 220 && b >= 99 && b <= 149) { // green -- turn right once
        led.setColor(0,200,0); // show green colour
        led.show();
        turnRight();

      } else if (r >= 204 && g >= 143 && g <= 214 && b >= 90 && b <= 145) { // orange -- u-turn
        led.setColor(255,51,0); // show orange colour
        led.show();
        uTurn();

      } else if (r >= 113 && r <= 169 && g >= 180 && b >= 195) { // blue -- turn right twice
        led.setColor(10,194,245); // show blue colour
        led.show();
        turnRightTwice();

      } else if (r >= 141 && r <= 212 && g >= 135 && g <= 203 && b >= 170) { // purple - turn left twice
        led.setColor(153,51,255); // show purple colour
        led.show();
        turnLeftTwice();

      } else if (r >= 204 && g >= 204 && b >= 204) { // white -- end of maze
        led.setColor(255,255,255); // show white colour
        led.show();

        leftMotor.stop(); // stop motors
        rightMotor.stop();
        delay(1000);

        celebrate(); // play tune
        delay(10000);

      } else { // if colour detected does not fall into any ranges above, should not happen in normal run
        led.setColor(0,0,0);
        led.show();
      }
    
    // if there is a missing wall, keep moving forward
    } else if (input == 0) { 
      leftMotor.run(-255);
      rightMotor.run(255);

    // move forward with PID control to keep in centre of maze
    } else {
      motorSpeedL = 220 + output/2.2;
      motorSpeedR = 220 - output/2.2;
      delay(20);
      leftMotor.run(-motorSpeedL);
      rightMotor.run(motorSpeedR);
    }
  }
}


