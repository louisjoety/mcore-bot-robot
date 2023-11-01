#include "MeMCore.h"

#define TIMEOUT 2000
#define SPEED_OF_SOUND 340 // For calculating distance based on ultrasonic sensor data
#define ULTRASONIC 12 // Port 1

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
MeRGBLed led(0,30); // Internal RGB LED to check colour

// Define colour sensor LED pins
int ledArray[] = {A1,A2,A3}; // check pins
// placeholders variables for colour detected
int red = 0;
int green = 0;
int blue = 0;
// floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {0,0,0};
float blackArray[] = {0,0,0};
float greyDiff[] = {0,0,0};
// for displaying colour values on serial monitor for debugging
char colourStr[3][5] = {"R = ", "G = ", "B = "};

int status = 0; // global status for button

void celebrate() { // Tune to play at end of maze
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
  buzzer.tone(932.33, 550); // Bb5
  buzzer.tone(0,50); // rest
  buzzer.tone(932.33, 150); // Bb5
  buzzer.tone(0,50); // rest
  buzzer.tone(932.33, 150); // Bb5
  buzzer.tone(0,50); // rest
  buzzer.tone(932.33, 150); // Bb5
  buzzer.tone(0,50); // rest
  buzzer.tone(1046.50, 1200); // C6
  buzzer.noTone();
}

void setup() {
  celebrate(); // testing tune, remove later
  led.setpin(13); // internal RGB LED is at Pin 13
  pinMode(BUTTON,INPUT); // set button to input
  Serial.begin(9600);
  for (int c = 0; c <= 2; c++) { // set LED pins to output
    pinMode(ledArray[c],OUTPUT);
  }
  led.setColor(255,255,255); // Check Indicator -- ON before Calibration
  led.show();
  setBalance(); // calibration
  led.setColor(255,255,255); // Check Indicator -- ON when ready to begin maze
  led.show();
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

void setBalance() { // calibrate white balance of colour sensor
  Serial.println("Put White Sample For Calibration ...");
  for (int i = 0; i < 5; i++) { // beeps once a second for 5s before scanning begins
  buzzer.tone(1000, 500);
  delay(500);
  }

  led.setColor(0,0,0); // Check Indicator -- OFF during Calibration
  led.show();
  // scan the white sample.
  // go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
  for (int i = 0; i <= 2; i++) {
    analogWrite(ledArray[i],255);
    delay(RGBWait);
    whiteArray[i] = getAvgReading(5); // scan 5 times and return the average, 
    analogWrite(ledArray[i],0);
    delay(RGBWait);
  }

  led.setColor(255,255,255); // Check Indicator -- ON after white calibration
  led.show();
  buzzer.tone(1000,1000); // beep to signal end of white calibration
  // calibrate black balance
  delay(1000);
  Serial.println("Put Black Sample For Calibration ...");
  for (int i = 0; i < 5; i++) { // beeps once a second for 5s before scanning begins
    buzzer.tone(1000, 500);
    delay(500);
  }

  led.setColor(0,0,0); // Check Indicator -- OFF during calibration
  led.show();
  // go through one colour at a time, set the minimum reading for red, green and blue to the black array
  for (int i = 0; i <= 2; i++) {
    analogWrite(ledArray[i],255);
    delay(RGBWait);
    blackArray[i] = getAvgReading(5);
    analogWrite(ledArray[i],0);
    delay(RGBWait);
  // the differnce between the maximum and the minimum gives the range
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }

  Serial.println("Colour Sensor Is Ready.");
  buzzer.tone(1000,1000); // beep to signal end of black calibration
}

void loop() {
  if (analogRead(BUTTON) < 100) { // mazepathing begins when button is pressed after calibration
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
  // calculate distance based on interval between sending & receiving sound waves
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  float distance = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;
  Serial.println(distance);
  
  int sensorState = lineFinder.readSensors(); // read the line sensor's state

  if (status == 1) { // start when button is pressed
    if (sensorState == S1_IN_S2_IN) { // stop at black line
      // stop motors
      leftMotor.run(0);
      rightMotor.run(0);

      // activate colour sensor
      for (int c = 0; c <= 2; c++) {    
        Serial.print(colourStr[c]);
        analogWrite(ledArray[c],255); // turn ON LED, red, green & blue, one colour at a time.
        delay(RGBWait);
        // get the average of 5 consecutive readings for the current colour and return an average 
        colourArray[c] = getAvgReading(5);
        // the average reading returned minus the lowest value divided by the maximum possible range, 
        // multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
        colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
        analogWrite(ledArray[c],0);  // turn off the current LED colour
        delay(RGBWait);
        Serial.println(int(colourArray[c])); // show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
      }

      led.setColor(colourArray[0],colourArray[1],colourArray[2]);
      led.show(); // show detected colour on internal RGB LEDs
      delay(500);
      // conditions for turning robot based on colour goes here

    } else if (distance < 12) { // centreing: when robot is too far left
      leftMotor.run(-255);
      rightMotor.run(180); // reduce speed to right motor, robot veers right

    } else if (distance > 14) { // centreing: when robot is too far right
      leftMotor.run(-180); // reduce speed to left motor, robot veers left
      rightMotor.run(255);
      
    } else { // move forward
      leftMotor.run(-255);
      rightMotor.run(255);
    }
  }
}


