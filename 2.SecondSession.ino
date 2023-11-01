#include "MeMCore.h"

#define TIMEOUT 2000
#define SPEED_OF_SOUND 340
#define ULTRASONIC 12
#define RGBWait 200 
#define LDRWait 10
#define LDR A0 
#define BUTTON 7

MeLineFollower lineFinder(PORT_2);
MeDCMotor leftMotor(M1); 
MeDCMotor rightMotor(M2); 
MeBuzzer buzzer; 
MeRGBLed led(0,30); 

int ledArray[] = {A1,A2,A3}; 
int red = 0;
int green = 0;
int blue = 0;
float colourArray[] = {0,0,0};
float whiteArray[] = {0,0,0};
float blackArray[] = {0,0,0};
float greyDiff[] = {0,0,0};
char colourStr[3][5] = {"R = ", "G = ", "B = "};
int status = 0; 

void celebrate() {
buzzer.tone(523, 200);
buzzer.tone(659, 200);
buzzer.tone(784, 200);
buzzer.tone(659, 150);
buzzer.tone(784, 400);
buzzer.noTone();
}

void setup() {
  delay(5000);
  led.setpin(13);
  pinMode(BUTTON,INPUT);
  Serial.begin(9600);
  for (int c = 0; c <= 2; c++) { 
    pinMode(ledArray[c],OUTPUT);
  }
  led.setColor(0,0,0); 
  led.show();
  setBalance(); 
  led.setColor(255,255,255);
  led.show();
}

int getAvgReading(int times){      
  int reading;
  int total = 0;
  for(int i = 0; i < times; i++) {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  return total/times;
}

void setBalance() {
  Serial.println("Put White Sample For Calibration ...");
  for (int i = 0; i < 5; i++) { // 5s to put on sample
  buzzer.tone(1000, 500);
  delay(500);
  }
  led.setColor(0,0,0);
  led.show();
  for (int i = 0; i <= 2; i++) {
    analogWrite(ledArray[i],255);
    delay(RGBWait);
    whiteArray[i] = getAvgReading(5);       
    analogWrite(ledArray[i],0);
    delay(RGBWait);
  }
  buzzer.tone(1000,1000);
  delay(1000);
  Serial.println("Put Black Sample For Calibration ...");
  for (int i = 0; i < 5; i++) { 
    buzzer.tone(1000, 500);
    delay(500);
  }
  for (int i = 0; i <= 2; i++) {
     analogWrite(ledArray[i],255);
     delay(RGBWait);
     blackArray[i] = getAvgReading(5);
     analogWrite(ledArray[i],0);
     delay(RGBWait);
     greyDiff[i] = whiteArray[i] - blackArray[i];
  }
  Serial.println("Colour Sensor Is Ready.");
  buzzer.tone(1000,1000);
  }

void loop() {
  if (analogRead(BUTTON) < 100) {
    status = 1 - status;
    delay(500);
  }
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
  
  int sensorState = lineFinder.readSensors(); 

  if (status == 1) {
    if (sensorState == S1_IN_S2_IN) { 
      leftMotor.run(0);
      rightMotor.run(0);
      for (int c = 0; c <= 2; c++) {    
        Serial.print(colourStr[c]);
        analogWrite(ledArray[c],255); 
        delay(RGBWait);
        colourArray[c] = getAvgReading(5);
        colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
        analogWrite(ledArray[c],0);  
        delay(RGBWait);
        Serial.println(int(colourArray[c]));
      }
      led.setColor(colourArray[0],colourArray[1],colourArray[2]);
      led.show();
      delay(500);
    } 
    else if (distance < 12) { 
      leftMotor.run(-255);
      rightMotor.run(180);
    } 
    else if (distance > 14) {
      leftMotor.run(-180);
      rightMotor.run(255);
    } 
    else { 
      leftMotor.run(-255);
      rightMotor.run(255);
    }
  }
}


