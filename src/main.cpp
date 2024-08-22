#include <Arduino.h>
#include "functions.h"

int springCylinder = 51;
int spindleCylinder = 53;
int cylinderButton = 45;
int spindleButton = 43;
int startButton = 41;
int spindleLeakButton = 39;

int cylinderButtonState = 0;
int spindleButtonState = 0;
int startButtonState = 0;
bool springState = false;
bool spindleState = false;

int big_stepPin = 4;
int big_dirPin = 2;
int big_enPin = 3;
int setpoint = 500;
float pressure = 0;
float postPressure = 0;
float error = 0;

int small_stepPin = 6;
int small_dirPin = 7;
int small_enPin = 8;

int TESTING = 9;
int FAIL = 11;
int PASS = 10;

int inputRelay = 49;
int outputRelay = 47;

bool spindleCheck = false;
bool springCheck = false;
bool startProgram = false;
bool pressureSetCheck = false;
bool shutOffCheck = false;
bool cleanCheck = false;
bool decayCheck = false;
bool firstPulseCheck = false;
bool secondPulseCheck = false;
bool pressureHold = false;
bool springPressureCheck = false;
bool spindleLeak = false;
bool springChecking = false;

bool fail = false;
bool pass = true;

void reset(bool status) {
  if (status == true) {
    Serial.println("Testing Complete ---> Valve passed all tests");
    digitalWrite(FAIL, LOW);
    digitalWrite(PASS, HIGH);
    digitalWrite(TESTING, LOW);
    digitalWrite(springCylinder, HIGH);
    digitalWrite(spindleCylinder, HIGH);
    digitalWrite(inputRelay, LOW);
    digitalWrite(outputRelay, HIGH);
    delay(600);
    digitalWrite(outputRelay, LOW);
    pressureSetCheck = false;  //good
    spindleCheck = false;  //good
    springCheck = false;   //good
    startProgram = false;  // good
  } else if (status == false) {
    digitalWrite(FAIL, HIGH);
    digitalWrite(PASS, LOW);
    digitalWrite(TESTING, LOW);
    digitalWrite(springCylinder, HIGH);
    digitalWrite(spindleCylinder, HIGH);
    digitalWrite(inputRelay, LOW);
    digitalWrite(outputRelay, HIGH);
    delay(600);
    digitalWrite(outputRelay, LOW);
    pressureSetCheck = false;
    spindleCheck = false;
    springCheck = false;
    startProgram = false;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(springCylinder, OUTPUT);
  pinMode(spindleCylinder, OUTPUT);
  pinMode(cylinderButton, INPUT);
  pinMode(spindleButton, INPUT);
  pinMode(startButton, INPUT);
  pinMode(spindleLeakButton, INPUT);
  //pinMode(spindleUpButton, INPUT);
  //pinMode(spindleDownButton, INPUT);

  pinMode(big_stepPin, OUTPUT);
  pinMode(big_dirPin, OUTPUT);
  pinMode(big_enPin, LOW);

  pinMode(small_stepPin, OUTPUT);
  pinMode(small_dirPin, OUTPUT);
  pinMode(small_enPin, LOW);

  pinMode(TESTING, OUTPUT);
  pinMode(PASS, OUTPUT);
  pinMode(FAIL, OUTPUT);

  pinMode(inputRelay, OUTPUT);
  pinMode(outputRelay, OUTPUT);

  digitalWrite(springCylinder, HIGH);
  digitalWrite(spindleCylinder, HIGH);
}

void loop() {


  cylinderButtonState = digitalRead(cylinderButton);
  spindleButtonState = digitalRead(spindleButton);
  startButtonState = digitalRead(startButton);


  if (cylinderButtonState == HIGH) {
    springState = !springState;
    digitalWrite(springCylinder, springState);
    if (springState == false) {
      springCheck = true;
    }
    delay(500);
  }

  if (spindleButtonState == HIGH) {
    spindleState = !spindleState;
    digitalWrite(spindleCylinder, spindleState);
    if (spindleState == false) {
      spindleCheck = true;
    }
    delay(500);
  }

  if (springCheck == true && spindleCheck == true && startButtonState == HIGH) {
    startProgram = true;
    digitalWrite(PASS, LOW);
    digitalWrite(FAIL, LOW);
  }

  if (startProgram == true) {
    Serial.println("Entered main program function");

    digitalWrite(inputRelay, HIGH);
    digitalWrite(outputRelay, HIGH);
    delay(500);

    digitalWrite(TESTING, HIGH);
    shutOffCheck = shutOffFunction(small_dirPin,small_stepPin,inputRelay,outputRelay);
    digitalWrite(TESTING, LOW);

    if (shutOffCheck == true) {
      digitalWrite(PASS, HIGH);
    } else if (shutOffCheck == false) {
      digitalWrite(fail, HIGH);
    }

    if (shutOffCheck == true) {

      digitalWrite(TESTING, HIGH);
      spindleLeak = spindleReversal(small_dirPin,small_stepPin,inputRelay,outputRelay,spindleButton,cylinderButton,spindleLeakButton,startButton);
      digitalWrite(TESTING, LOW);
      digitalWrite(inputRelay, LOW);
      digitalWrite(outputRelay, HIGH);


      if (spindleLeak == true) {
        pressure = inletPressureSensor();
        Serial.print("Current inlet pressure : ");
        Serial.println(pressure);
        postPressure = pressureSensor();
        Serial.print("Current outlet pressure : ");
        Serial.println(postPressure);

        if (abs(pressure - postPressure) > 350) {
          Serial.println("Test 1 : Output pressure much lower than input pressure ----> pressure is held");
          springPressureCheck = true;
        } else {
          Serial.println("Test 1 : Output pressure similar to the input pressure ----> pressure is not held");
        }

        springChecking = springPressureHold(big_dirPin,big_stepPin,inputRelay,outputRelay);

        digitalWrite(TESTING, HIGH);
        pressureHold = pressureDecay(inputRelay,outputRelay);
        digitalWrite(TESTING, LOW);

        if (pressureHold == true) {
          digitalWrite(PASS, HIGH);
        } else {
          digitalWrite(FAIL, HIGH);
        }


        if (pressureHold == true && springPressureCheck == true && springChecking == true) {
          Serial.println("Successfully holding pressure");

          while (pressureSetCheck == false) {
            digitalWrite(inputRelay, HIGH);
            digitalWrite(outputRelay, LOW);

            pressure = pressureSensor();

            Serial.print("Setpoint : ");
            Serial.print(setpoint);
            Serial.print("  ");
            Serial.print("Current pressure : ");
            Serial.print(pressure);
            Serial.println("  ");

            if ((abs(pressure - setpoint)) < 30) {
              pressureSetCheck = true;
              Serial.println("Set pressure Achieved");
            } else {
              error = pressure - setpoint;
              pressureAdjustment(error,big_dirPin,big_stepPin,inputRelay,outputRelay,setpoint);
            }
            delay(10);
          }

          digitalWrite(inputRelay, HIGH);
          digitalWrite(outputRelay, LOW);
          delay(250);
          pressure = pressureSensor();
          Serial.print("Pressure test for the pulsing : ");
          Serial.println(pressure);
          secondPulseCheck = secondPulse(pressure,inputRelay,outputRelay);

          digitalWrite(inputRelay, HIGH);
          digitalWrite(outputRelay, LOW);
          delay(250);
          pressure = pressureSensor();
          Serial.print("Checking for pressure creep after setting the pressure. Pressure set to ");
          Serial.println(pressure);
          digitalWrite(TESTING, HIGH);
          springPressureCheck = springPressure(pressure);
          digitalWrite(TESTING, LOW);

          if (springPressureCheck == false) {
            digitalWrite(FAIL, HIGH);
          } else {
            digitalWrite(PASS, HIGH);
          }

          if (secondPulseCheck == true && springPressureCheck == true) {
            Serial.print("Pressure holding at setpoint of ");
            Serial.println(setpoint);

            reset(pass);

          } else {
            Serial.println("Pulsing failed or pressure not holding to the setpoint");
            reset(fail);
          }
        } else {
          Serial.println("Spring issue, pressure on the output not valid");
          reset(fail);
        }
      } else {
        Serial.println("Leak from the spindle based on operators observation");
        reset(fail);
      }
    } else {
      Serial.println("Shut off failed");
      reset(fail);
    }
  }
}
