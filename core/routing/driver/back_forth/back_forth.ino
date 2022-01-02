/* Firgelli Automations
 * Limited or no support: we do not have the resources for Arduino code support
 * 
 * Program enables momentary direction control of actuator using push button
 */
 
#include <elapsedMillis.h>
elapsedMillis timeElapsed;

int PIN_EXTEND = 9; 
int PIN_RETRACT = 10;
int sensorPin = A0;

#define RETRACT -1
#define STOP 0
#define EXTEND 1

// retracted is max analog value when blue is ground and white is +

int sensorVal;
// speed range 0-255
int Speed=255;
float strokeLength = 12.0;
float extensionLength;

int maxAnalogReading;
int minAnalogReading;

void setup() {
   pinMode(PIN_EXTEND, OUTPUT);
   pinMode(PIN_RETRACT, OUTPUT);
   pinMode(sensorPin, INPUT);
   Serial.begin(19200);
   minAnalogReading = moveToLimit(EXTEND);
   Serial.print("Extended value: ");
   Serial.println(minAnalogReading);
   maxAnalogReading = moveToLimit(RETRACT);
   Serial.print("Retracted value: ");
   Serial.println(maxAnalogReading);
   driveActuator(STOP, 0);
}

void loop(){
   int center = (minAnalogReading + maxAnalogReading) / 2;
   sensorVal = analogRead(sensorPin);
   Serial.println(center);
   Serial.println(sensorVal);
   Serial.println("Extending to center");
   int ctr = 0;
   while (sensorVal > center) {
      driveActuator(EXTEND, 64);
      delay(20);
      sensorVal = analogRead(sensorPin);
      if (++ctr == 8) {
         Serial.println(sensorVal);
         ctr = 0;
      }
   }
   driveActuator(STOP, 0);
   delay(1000);
   Serial.println(sensorVal);
   Serial.println("Retracting to center");
   ctr = 0;
   while (sensorVal < center) {
      driveActuator(RETRACT, 64);
      delay(20);
      sensorVal = analogRead(sensorPin);
      if (++ctr == 8) {
         Serial.println(sensorVal);
         ctr = 0;
      }
   }
   driveActuator(STOP, 0);
   Serial.println(sensorVal);
   Serial.println("should be centered");
   delay(3000);
//   if (ctr == 0) {
//  Serial.println("Extending...");
//  sensorVal = analogRead(sensorPin);
//  while(sensorVal > minAnalogReading){
//    driveActuator(EXTEND, Speed);
//    displayOutput();  
//    delay(20);
//  }
//  driveActuator(0, Speed);
//  delay(3000);
//  
//  Serial.println("Retracting...");
//  sensorVal = analogRead(sensorPin);
//  while(sensorVal < maxAnalogReading){
//    driveActuator(RETRACT, Speed);
//    displayOutput();  
//    delay(20);
//  }
//  driveActuator(STOP, Speed);
//  delay(3000);
}

int moveToLimit(int Direction){
  int prevReading=0;
  int currReading=0;
  do{
    prevReading = currReading;
    driveActuator(Direction, Speed);
    timeElapsed = 0;
    while(timeElapsed < 200){ delay(1);}           //keep moving until analog reading remains the same for 200ms
    currReading = analogRead(sensorPin);
  }while(prevReading != currReading);
  Serial.print("Limit for direction ");
  Serial.print(Direction);
  Serial.print(" is ");
  Serial.println(currReading);
  return currReading;
}

float mapfloat(float x, float inputMin, float inputMax, float outputMin, float outputMax){
 return outputMax - (x-inputMin) * (outputMax-outputMin) / (inputMax-inputMin);
// return (x-inputMin) * (outputMax-outputMin) / (inputMax-inputMin) + outputMin;
}

void displayOutput(){
  sensorVal = analogRead(sensorPin);
    extensionLength = mapfloat(sensorVal, float(minAnalogReading), float(maxAnalogReading), 0.0, strokeLength);
    Serial.print("Analog Reading: ");
    Serial.print(sensorVal);
    Serial.print("\tActuator extension length: ");
    Serial.print(extensionLength);
    Serial.println(" inches");  
}

void driveActuator(int Direction, int Speed){
  switch(Direction){
    case 1:       //extension
      analogWrite(PIN_EXTEND, Speed);
      analogWrite(PIN_RETRACT, 0);
      break;
   
    case STOP:       //stopping
      analogWrite(PIN_EXTEND, 0);
      analogWrite(PIN_RETRACT, 0);
      break;

    case RETRACT:      //retraction
      analogWrite(PIN_EXTEND, 0);
      analogWrite(PIN_RETRACT, Speed);
      break;
  }
}
