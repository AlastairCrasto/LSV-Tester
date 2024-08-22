#include <Arduino.h>
#include "functions.h"

float pressureSensor() {
  float pressure = 0;
  float sensorValue = analogRead(A0);  //reading anaolgue value at pin A0
  // pressure = 1.1769 * sensorValue + 5.4893;  // analogue reading to kPA conversion
  pressure = 1.1843 * sensorValue - 2.5289;

  if (pressure < 6) {  // too low of a pressure to capture - assuming 0 due to conversion error
    return 0;
  } else {
    return pressure;
  }
}

float inletPressureSensor() {
  float pressureSend = 0;

  float sensorValue = analogRead(A1);
  pressureSend = sensorValue;

  if (sensorValue < 1) {  // too low of a pressure to capture - assuming 0 due to conversion error
    return 0;
  } else {
    return pressureSend;
  }
}

void pressureAdjustment(float errorValue,int springDirPin,int springStepPin,int input,int output,int setpoint) {
  Serial.println(" Entered automatic adjustment");
  Serial.print("errorValue : ");
  Serial.println(errorValue);
  if (errorValue > 0) {  // Setting the direction for the motor
    digitalWrite(springDirPin, LOW);
    digitalWrite(input, LOW);
    digitalWrite(output, HIGH);
    delay(200);
    Serial.println("Reversal");
    for (int i = 0; i <= 800; i++) {
      digitalWrite(springStepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(springStepPin, LOW);
      delayMicroseconds(500);
    }
    delay(500);
  } else {
    Serial.println("Tightening");
    digitalWrite(springDirPin, HIGH);
    while (errorValue < -1) {
      digitalWrite(springStepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(springStepPin, LOW);
      delayMicroseconds(500);

      float pressure_check = pressureSensor();
      errorValue = pressure_check - setpoint;
    }
  }

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  //digitalWrite(springCylinder,HIGH);
  delay(500);

  Serial.println("Exiting adjustment --> back to loop function");
}

bool shutOffFunction(int spindleDirPin,int spindleStepPin,int input, int output) {

  bool coarse_tighten = true;

  if (coarse_tighten == true) {
    digitalWrite(spindleDirPin, HIGH);
    digitalWrite(input, LOW);
    digitalWrite(output, HIGH);
    delay(600);
    for (int j = 0; j < 9; j++) {
      for (int i = 0; i <= 400; i++) {
        digitalWrite(spindleStepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(spindleStepPin, LOW);
        delayMicroseconds(500);
      }
    }
    Serial.println("Coarse tightening completed");
    coarse_tighten = false;
  }

  //digitalWrite(outputRelay,LOW);
  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  delay(500);

  float preSample = 0;
  float postSample = 0;

  preSample = pressureSensor();
  Serial.print("The outlet pressure after shut off (pre) : ");
  Serial.println(preSample);

  Serial.println("Checking for pressure creep after shut off");
  delay(10000);
  Serial.print("The outlet pressure after shut off (post) : ");
  postSample = pressureSensor();
  Serial.println(postSample);
  Serial.print("Difference : ");
  Serial.println(abs(preSample - postSample));

  if (abs(preSample - postSample) >= 40 || preSample > 50) {
    Serial.println("Pressure leaked after shut off ---> shut off failed");
    return false;
  } else {
    Serial.println("Pressure not leaking after shut off ---> shut off passed");
    return true;
  }
}

bool pressureDecay(int input, int output) {
  float pressureTest = 0;
  float nextSample = 0;


  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  delay(1000);

  digitalWrite(input, LOW);

  pressureTest = inletPressureSensor();
  Serial.print("System pressure before test: ");
  Serial.println(pressureTest);
  delay(5000);
  nextSample = inletPressureSensor();
  Serial.print("System pressure after test: ");
  Serial.println(nextSample);

  digitalWrite(output, HIGH);
  delay(1000);

  Serial.print("Pressure decay: ");
  Serial.println(abs(nextSample - pressureTest));

  if (abs(nextSample - pressureTest) > 50) {
    return false;
  } else {
    return true;
  }
}

bool springPressureHold(int springDirPin,int springStepPin,int input,int output) {
  float preSample = 0;
  float postSample = 0;

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);
  delay(500);

  preSample = pressureSensor();
  Serial.print("Pressure before pressure is adjusted : ");
  Serial.println(preSample);

  digitalWrite(springDirPin, HIGH);

  for (int i = 0; i <= 800; i++) {
    digitalWrite(springStepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(springStepPin, LOW);
    delayMicroseconds(500);
  }

  postSample = pressureSensor();
  Serial.print("Pressure after pressure is adjusted : ");
  Serial.println(postSample);

  if (abs(preSample - postSample) < 20) {
    Serial.println("Test 2 : No pressure change noticed ---> pressure is not holding");
    return false;
  } else {
    Serial.println("Test 2 : Pressure change is considerable when spring is driven ---> pressure is holding");
    return true;
  }
}

bool springPressure(float pressureSetpoint) {
  float postSample = 0;

  Serial.print("Pressure before delay to check for creep : ");
  Serial.println(pressureSetpoint);
  delay(15000);

  postSample = pressureSensor();
  Serial.print("Pressure after delay to check for creep : ");
  Serial.println(postSample);

  if (abs(pressureSetpoint - postSample) > 15) {
    return false;
  } else {
    return true;
  }
}

bool secondPulse(float currentPressure,int input,int output) {

  digitalWrite(input, HIGH);

  int pulseNumber = 0;
  float newPressure = 0;
  bool holdingPressure = true;

  while (pulseNumber < 3) {
    digitalWrite(output, LOW);
    delay(1500);
    newPressure = pressureSensor();
    Serial.print("Pressure after ");
    Serial.print(pulseNumber + 1);
    Serial.print(" pulse : ");
    Serial.println(newPressure);
    if (abs(currentPressure - newPressure) >= 20) {
      holdingPressure = false;
      Serial.println("Pulsing failed");
      return holdingPressure;
    }
    digitalWrite(output, HIGH);
    delay(500);
    pulseNumber += 1;
  }
  Serial.println("Pulsing passed");
  return holdingPressure;
}

bool spindleReversal(int spindleDirPin,int spindleStepPin,int input, int output,int spindleUpButton, int spindleDownButton, int spindlePassButton, int spindleFailButton) {

  int upCount = 0;

  Serial.println("Loosening for leak test");
  digitalWrite(spindleDirPin, LOW);
  for (int i = 0; i <= 400; i++) {
    digitalWrite(spindleStepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(spindleStepPin, LOW);
    delayMicroseconds(500);
  }

  digitalWrite(input, HIGH);
  digitalWrite(output, LOW);

  bool spindleLeakState = false;

  while (spindleLeakState == false) {

    bool spindleUpState = digitalRead(spindleUpButton);
    bool spindleDownState = digitalRead(spindleDownButton);
    bool spindlePass = digitalRead(spindlePassButton);
    bool spindleFail = digitalRead(spindleFailButton);


    if (spindleDownState == HIGH) {
      digitalWrite(spindleDirPin, HIGH);
      for (int i = 0; i <= 400; i++) {
        digitalWrite(spindleStepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(spindleStepPin, LOW);
        delayMicroseconds(500);
      }
      upCount--;
      Serial.println(upCount);
    }

    if (spindleUpState == HIGH) {
      digitalWrite(spindleDirPin, LOW);
      for (int i = 0; i <= 400; i++) {
        digitalWrite(spindleStepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(spindleStepPin, LOW);
        delayMicroseconds(500);
      }
      upCount++;
      Serial.println(upCount);
    }

    if (spindleFail == HIGH) {
      return false;
    }

    if (upCount >= 6) {
      if (spindlePass == HIGH) {
        return true;
      }
    }
  }
}