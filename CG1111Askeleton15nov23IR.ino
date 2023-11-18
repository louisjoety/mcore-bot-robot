/*
 * CG1111A A-maze-ing Race Project
 * Team B02-S2-T4
 * Main code for mBot
 */

#include <MeMCore.h>
#include <PID_v1.h>

/**
 * Definitions of important variables
 *
 * These include variables for measuring distance from ultrasonic sensor to wall,
 * the ultrasonic sensor port,
 * time delays for LDR in colour sensor, and
 * arduino pins to non-mBot electronics (LDR, IR receiver, 2-to-4 decoder)
 */
#define TIMEOUT 2000 // timeout value for ultrasonic sensor pulse
#define SPEED_OF_SOUND 340 // for calculating distance based on ultrasonic sensor data
#define ULTRASONIC 12 // port 1

// time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 150 // in milliseconds

// time delay between LDR readings
#define LDRWait 10 // in milliseconds

// initialise RJ25 adapter pins to peripherals
#define LDR A2 // LDR sensor pin
#define IR A3 // IR receiver pin

#define D1 A0 // pin 1A of 2-to-4 decoder
#define D2 A1 // pin 1B of 2-to-4 decoder

#define BUTTON 7 // button to start mazepathing

/**
 * Assigning mBot ports to external mBot peripherals (motors, line sensor)
 * and initialising internal mBot peripherals (internal RGB LEDs, buzzer)
 */
MeLineFollower lineFinder(PORT_2); // black line detector

MeDCMotor leftMotor(M1); // left motor
MeDCMotor rightMotor(M2); // right motor
MeBuzzer buzzer; // create the buzzer object
MeRGBLed led(0,30); // internal RGB LED to display colour detected

/**
 * Defining parameters for PID guidance
 * PID system ensures smooth movement of robot
 * while ensuring it stays in centre of maze without hitting walls
 *
 * Parameters include:
 * setpoint, desired distnce from wall in cm
 * input (ultrasonic sensor reading)
 * output (motor speed adjustment based off of deviation from setpoint) of PID calculations
 * kp, ki and kd, PID tuning parameters to optimise movement of robot
 * baseSpeed, the base speed of motors that PID can add or subtract from
 *
 * Also includes turnSpeed, the speed of motors when making a turn,
 * motorSpeedL and motorSpeedR, variables to hold the adjusted motor speed values,
 * and the initial PID setup for the PID library
 */
double setpoint = 10.0;

double input, output;

double kp = 150;
double ki = 0;
double kd = 0.05;

double baseSpeed = 230;
double turnSpeed = 255;

float motorSpeedL;
float motorSpeedR;

PID PID(&input,&output,&setpoint,kp,ki,kd,DIRECT);

/**
 * Defining parameters for colour sensor
 * This includes:
 *
 * ledPins, array of values to set the 2-to-4 decoder inputs to switch each colour LED (and IR emitter) on and off
 * r, g and b, placeholder variables to set the detected colour values
 * colourArray, whiteArray, blackArray and greyDiff, arrays to hold colour values and the calibration values
 * colourStr, an array of texts for printing the colours to serial monitor for debugging
 */
int ledPins[4][2] = {
  {0, 0}, // red LED
  {255, 0}, // green LED
  {0, 255}, // blue LED
  {255, 255} // IR emitter
};

int r = 0;
int g = 0;
int b = 0;

float colourArray[] = {0,0,0};
float whiteArray[] = {0,0,0}; // only used for calibration, not in actual run
float blackArray[] = {762.00,943.00,877.00}; // values are set to what is printed on serial monitor after calibration
float greyDiff[] = {214.00,68.00,122.00};

char colourStr[3][5] = {"R = ", "G = ", "B = "};

/** 
 * Defining button variable
 * This enables the robot to start only when button is pressed
 */
int status = 0; 

/**
 * Function to play a tune at the end of the maze
 * Numbers in brackets specify frequency of tone and the duration in ms
 *
 * Tune is the Level Complete Jingle from Super Mario Bros.
 */
void celebrate() {
  buzzer.tone(196.00, 150); // G3
  buzzer.tone(261.63, 150); // C4
  buzzer.tone(329.63, 150); // E4
  buzzer.tone(392.00, 150); // G4
  buzzer.tone(523.25, 150); // C5
  buzzer.tone(659.25, 150); // E5
  buzzer.tone(783.99, 450); // G5
  buzzer.tone(659.25, 450); // E5
  buzzer.tone(207.65, 150); // G#3
  buzzer.tone(261.63, 150); // C4
  buzzer.tone(311.13, 150); // Eb4
  buzzer.tone(415.30, 150); // G#4
  buzzer.tone(523.25, 150); // C5
  buzzer.tone(622.25, 150); // Eb5
  buzzer.tone(830.61, 450); // G#5
  buzzer.tone(622.25, 450); // Eb5
  buzzer.tone(233.08, 150); // Bb3
  buzzer.tone(293.66, 150); // D4
  buzzer.tone(349.23, 150); // F4
  buzzer.tone(466.16, 150); // Bb4
  buzzer.tone(587.33, 150); // D5
  buzzer.tone(698.46, 150); // F5
  buzzer.tone(932.33, 450); // Bb5
  buzzer.tone(932.33, 100); // Bb5
  delay(50);
  buzzer.tone(932.33, 100); // Bb5
  delay(50);
  buzzer.tone(932.33, 100); // Bb5
  delay(50);
  buzzer.tone(1046.50, 900); // C6
  buzzer.noTone();
}

/**
 * Initial setup of peripherals
 * This sets pins to input and output based off their function
 * and configures PID parameters to correct outputs
 *
 * Once setup is complete, robot lights up internal LEDs to white and plays tune to signal ready to begin mazepathing
 * Tune is the first few notes from Overworld Theme, Super Mario Bros.
 */
void setup() {
  led.setpin(13); // internal RGB LED is at Pin 13
  pinMode(BUTTON,INPUT); // set button to input
  Serial.begin(9600);

  // set 1A and 1B 2-to-4 decoder pins to output
  pinMode(D1,OUTPUT);
  pinMode(D2,OUTPUT);

  // configure PID
  PID.SetMode(AUTOMATIC);
  PID.SetOutputLimits(-250,250);
  PID.SetSampleTime(30);

  // light up internal LEDs to signal ready
  led.setColor(255,255,255);
  led.show();

  // setBalance(); // calibrate colour sensor when needed

  // play tune to signal ready
  buzzer.tone(659.25,100); // E5
  delay(50);
  buzzer.tone(659.25,100); // E5
  delay(200);
  buzzer.tone(659.25,100); // E5
  delay(200);
  buzzer.tone(523.25,100); // C5
  delay(50);
  buzzer.tone(659.25,100); // E5
  delay(200);
  buzzer.tone(783.99,100); // G5
  delay(500);
  buzzer.tone(392,100); // G4
  buzzer.noTone();
}

/**
 * Function to find the average reading for the requested number of times of scanning LDR
 * Takes 5 consecutive readings from the LDR,
 * then returns the average of the 5 readings
 */
int getAvgReading(int times) { 
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

/**
 * Function to calibrate the minimum value and maximum range of colour sensor
 * Only activated during setup when required
 *
 * Takes reading from LDR on a white and a black sample,
 * finds the maximum range by subtracting black values from white values,
 * then prints out the minimum values and range values to serial monitor
 */
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
    analogWrite(D1,ledPins[i][0]);
    analogWrite(D2,ledPins[i][1]);
    delay(RGBWait);
    whiteArray[i] = getAvgReading(5); // scan 5 times and return the average, 
    analogWrite(D1,ledPins[i][0]);
    analogWrite(D2,ledPins[i][1]);
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
    analogWrite(D1,ledPins[i][0]);
    analogWrite(D2,ledPins[i][1]);
    delay(RGBWait);
    blackArray[i] = getAvgReading(5); // scan 5 times and return the average, 
    analogWrite(D1,ledPins[i][0]);
    analogWrite(D2,ledPins[i][1]);
    delay(RGBWait);
  // the differnce between the maximum and the minimum gives the range
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }

  // print out the values for black array and grey diff
  Serial.println(blackArray[0]);
  Serial.println(blackArray[1]);
  Serial.println(blackArray[2]);
  Serial.println(greyDiff[0]);
  Serial.println(greyDiff[1]);
  Serial.println(greyDiff[2]);

  buzzer.tone(1000,1000); // beep to signal end of black calibration
}

/** 
 * Function to take 5 readings from IR and return the average
 * When called, this turns the IR emitter off and reads the IR receiver for ambient value,
 * then turns IR emitter on and reads the recevier again for the measured value,
 * then subtracts ambient value from measured value to get a reading without ambient interference
 *
 * This is repeated 5 times consecutively, and average of the readings is returned
 */
float readIR() {
  float ambient;
  float reading;
  float total = 0;
  for (int i = 0; i < 5; i++) {
    // turn off IR emitter (switching decoder to red LED)
    analogWrite(D1,ledPins[0][0]);
    analogWrite(D2,ledPins[0][1]);
    delay(10);
    // take reading from IR receiver
    ambient = analogRead(IR);

    // turn on IR emitter
    analogWrite(D1,ledPins[3][0]);
    analogWrite(D2,ledPins[3][1]);
    delay(10);
    // take reading from IR receiver
    reading = analogRead(IR);
    total += reading - ambient;
  }
  Serial.println(total/5);
  return total/5;
}

/**
 * Functions to make turns at waypoints
 * Based off the colour, the main loop function calls these functions
 * which will set speed of motors to make the robot turn
 *
 * Duration of turns and straight portions (for blue and purple colours) can be adjusted here
 */
// turn left once
void turnLeft() {
  leftMotor.run(turnSpeed); // left motor backwards
  rightMotor.run(turnSpeed); // right motor forwards
  delay(305); // turn duration, to adjust

  leftMotor.run(-turnSpeed); // go straight for short while
  rightMotor.run(turnSpeed);
  delay(100);
}

// turn right once
void turnRight() {
  leftMotor.run(-turnSpeed); // left motor forwards
  rightMotor.run(-turnSpeed); // right motor backwards
  delay(305); // turn duration, to adjust

  leftMotor.run(-turnSpeed); // go straight for short while
  rightMotor.run(turnSpeed);
  delay(100);
}

// make a u-turn
void uTurn() {
  leftMotor.run(turnSpeed); // left motor forwards
  rightMotor.run(turnSpeed); // right motor backwards 
  delay(550); // turn duration, to adjust

  leftMotor.run(-turnSpeed); // go straight for short while
  rightMotor.run(turnSpeed);
  delay(100);
}

// turn right twice
void turnRightTwice() {
  leftMotor.run(-turnSpeed); // left motor forwards - first right turn
  rightMotor.run(-turnSpeed); // right motor backwards
  delay(300); // turn duration, to adjust

  leftMotor.run(-turnSpeed); // go straight
  rightMotor.run(turnSpeed);
  delay(610); // straight duration, to adjust

  leftMotor.run(-turnSpeed); // left motor forwards - second right turn
  rightMotor.run(-turnSpeed); // right motor backwards
  delay(350); // turn duration, to adjust

  leftMotor.run(-turnSpeed); // go straight for short while
  rightMotor.run(turnSpeed);
  delay(100);
}

// turn left twice
void turnLeftTwice() {
  leftMotor.run(turnSpeed); // left motor backwards  - first left turn
  rightMotor.run(turnSpeed); // right motor forwards
  delay(300); // turn duration, to adjust

  leftMotor.run(-turnSpeed); // go straight
  rightMotor.run(turnSpeed);
  delay(645); // straight duration, to adjust

  leftMotor.run(turnSpeed); // left motor backwards - second left turn
  rightMotor.run(turnSpeed); // right motor forwards
  delay(325); // turn duration, to adjust

  leftMotor.run(-turnSpeed); // go straight for short while
  rightMotor.run(turnSpeed);
  delay(100);
}

/**
 * The main looping function containing the main functions of robot when mazepathing
 * 
 * This includes:
 * Checking if button is pressed before starting mazepathing
 * Reading the ultrasonic sensor's distance from left wall
 * Calculating the PID output values based off ultrasonic sensor reading
 * Reading the IR sensor's distance from right wall
 * Reading the line sensor to check for black line
 * Reading the colour sensor when black line is reached
 * Performing tasks based off the detected colour
 * Adjusting motor speeds to veer left when IR sensor is too close to right wall
 * Setting motor speeds based off PID to stay in centre of maze
 */
void loop() {
  if (analogRead(BUTTON) < 100) { // mazepathing begins when button is pressed
    status = 1 - status;
    delay(500);
  }

  // ultrasonic sensor read distance to left wall
  pinMode(ULTRASONIC, OUTPUT);
  analogWrite(ULTRASONIC, LOW);
  delayMicroseconds(2);
  analogWrite(ULTRASONIC, HIGH);
  delayMicroseconds(10);
  analogWrite(ULTRASONIC, LOW);
  pinMode(ULTRASONIC, INPUT);

  // calculate distance based on interval between sending & receiving sound waves, set to PID input
  long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT);
  input = duration / 2.0 / 1000000 * SPEED_OF_SOUND * 100;
  
  int sensorState = lineFinder.readSensors(); // read the line sensor's state

  if (status == 1) { // start when button is pressed
    PID.Compute(); // continually compute PID each loop
    readIR(); // take a reading from IR sensor each loop

    // detect black line at a turn and stop to read colour
    if (sensorState == S1_IN_S2_IN) {
      // stop motors
      leftMotor.run(0);
      rightMotor.run(0);

      // activate colour sensor
      for (int c = 0; c <= 2; c++) {
        // turn on RGB LEDs, one at a time
        analogWrite(D1, ledPins[c][0]);
        analogWrite(D2, ledPins[c][1]);
        delay(RGBWait);
        colourArray[c] = getAvgReading(5); // get average of 5 readings from LDR

        // the average reading returned minus the lowest value divided by the maximum possible range, 
        // multiplied by 255 will give a value between 0-255, representing the value for the current reflectivity (i.e. the colour LDR is exposed to)
        colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
        
        // turn off the current LED colour (switching decoder to IR emitter)
        analogWrite(D1,ledPins[3][0]);
        analogWrite(D2,ledPins[3][1]);

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

      // mario coin collecting sound after colour is read
      buzzer.tone(987.77, 25); // B4
      buzzer.tone(1318.51, 100); // E5

      // check r, g & b values if they fall within range of one of the colours

      // red -- turn left once
      if (r >= 220 && g >= 95 && g <= 160 && b >= 84 && b <= 165) {
        led.setColor(255,0,0); // show red colour on top
        led.show();
        turnLeft();

      // green -- turn right once
      } else if (r >= 100 && r <= 170 && g >= 160 && g <= 225 && b >= 99 && b <= 170) {
        led.setColor(0,200,0); // show green colour on top
        led.show();
        turnRight();

      // orange -- u-turn
      } else if (r >= 215 && g >= 160 && g <= 220 && b >= 100 && b <= 180) { 
        led.setColor(255,51,0); // show orange colour on top
        led.show();
        uTurn();

      // blue -- turn right twice
      } else if (r >= 155 && r <= 200 && g >= 205 && b >= 220) { 
        led.setColor(10,194,245); // show blue colour on top
        led.show();
        turnRightTwice();

      // purple -- turn left twice
      } else if (r >= 185 && r <= 230 && g >= 170 && g <= 204 && b >= 180) { 
        led.setColor(153,51,255); // show purple colour on top
        led.show();
        turnLeftTwice();

      // white -- end of maze, stop and play tune
      } else if (r >= 204 && g >= 204 && b >= 204) {
        led.setColor(255,255,255); // show white colour on top
        led.show();

        leftMotor.stop(); // stop motors
        rightMotor.stop();
        delay(100);

        celebrate(); // play tune
        delay(10000);

      // if colour detected does not fall into any ranges above, should not happen in normal run
      } else {
        led.setColor(0,0,0);
        led.show();
      }

    // if IR sensor detects within 4cm of right wall, veer to left
    } else if (IR < 395) { 
      leftMotor.run(-baseSpeed + 50);
      rightMotor.run(baseSpeed);

    // if there is a missing wall on the left, keep moving forward
    } else if (input == 0 || input > 20) { 
      leftMotor.run(-baseSpeed);
      rightMotor.run(baseSpeed);

    // move forward with PID control to keep in centre of maze
    } else {
      motorSpeedL = baseSpeed + output/2.2;
      motorSpeedR = baseSpeed - output/2.2;
      delay(20); // small delay shifts PID timing to correct values
      leftMotor.run(-motorSpeedL);
      rightMotor.run(motorSpeedR);
    }
  }
}