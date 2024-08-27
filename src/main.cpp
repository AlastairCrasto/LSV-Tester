#include <Arduino.h>
#include "functions.h"

// initializing all of the variables

// button variables
int springCylinder = 51;
int spindleCylinder = 53;
int cylinderButton = 45;
int spindleButton = 43;
int startButton = 41;
int spindleLeakButton = 39;
int cylinderButtonState = 0;
int spindleButtonState = 0;
int startButtonState = 0;

// checking to see if both drivers are active
bool springState = false;
bool spindleState = false;

// motor pins
int big_stepPin = 4;
int big_dirPin = 2;
int big_enPin = 3;
int small_stepPin = 6;
int small_dirPin = 7;
int small_enPin = 8;

// output pressure setpoint ----> need to re-evaluate once HMI display is in action
int setpoint = 500;

// variables used in the loop function for various purposes
float pressure = 0;
float postPressure = 0;
float error = 0;

// LED functionality
int TESTING = 9; // Testing phase LED
int FAIL = 11;   // Red LED
int PASS = 10;   // Green LED

// input and output solenoids
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

// Variables for testing limits

int inOutDiff = 250;
int setLimit = 30;
int shutOffLimit = 40;
int preShutOff = 50;
int decayLimit = 50;
int holdingLimit = 20;
int creepLimit = 15;
int pulse = 3;
int pulseLimit = 20;
int checkpoints = 6;

// the reset function resets all of the variables used in the loop function after a pass or fail test to carry out testing for the next valve
void reset(bool status)
{
  if (status == true)
  {
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
    pressureSetCheck = false;
    spindleCheck = false;
    springCheck = false;
    startProgram = false;
  }
  else if (status == false)
  {
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

void setup()
{
  Serial.begin(9600);

  pinMode(springCylinder, OUTPUT);
  pinMode(spindleCylinder, OUTPUT);
  pinMode(cylinderButton, INPUT);
  pinMode(spindleButton, INPUT);
  pinMode(startButton, INPUT);
  pinMode(spindleLeakButton, INPUT);
  // pinMode(spindleUpButton, INPUT);
  // pinMode(spindleDownButton, INPUT);

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

void loop()
{

  cylinderButtonState = digitalRead(cylinderButton);
  spindleButtonState = digitalRead(spindleButton);
  startButtonState = digitalRead(startButton);

  if (cylinderButtonState == HIGH)
  { // if HIGH, spring housing driver is lowered
    springState = !springState;
    digitalWrite(springCylinder, springState);
    if (springState == false)
    {
      springCheck = true;
    }
    delay(500);
  }

  if (spindleButtonState == HIGH)
  { // if HIGH, spindle driver is lowered
    spindleState = !spindleState;
    digitalWrite(spindleCylinder, spindleState);
    if (spindleState == false)
    {
      spindleCheck = true;
    }
    delay(500);
  }

  if (springCheck == true && spindleCheck == true && startButtonState == HIGH)
  { // Testing only begins once both drivers are lowered and start button is clicked
    startProgram = true;
    digitalWrite(PASS, LOW);
    digitalWrite(FAIL, LOW);
  }

  if (startProgram == true)
  {
    Serial.println("Entered main program function");

    digitalWrite(inputRelay, HIGH);
    digitalWrite(outputRelay, HIGH);
    delay(500);

    digitalWrite(TESTING, HIGH);
    shutOffCheck = shutOffFunction(small_dirPin, small_stepPin, inputRelay, outputRelay,shutOffLimit,preShutOff); // Tests shut off functionality
    digitalWrite(TESTING, LOW);

    if (shutOffCheck == true)
    {
      digitalWrite(PASS, HIGH);
    }
    else if (shutOffCheck == false)
    {
      digitalWrite(fail, HIGH);
    }

    if (shutOffCheck == true)
    {

      digitalWrite(TESTING, HIGH);
      spindleLeak = spindleReversal(small_dirPin, small_stepPin, inputRelay, outputRelay, spindleButton, cylinderButton, spindleLeakButton, startButton,checkpoints); // operators to test for leaks
      digitalWrite(TESTING, LOW);
      digitalWrite(inputRelay, LOW);
      digitalWrite(outputRelay, HIGH);

      if (spindleLeak == true)
      { // operator passes the valve after observing no leaks
        pressure = inletPressureSensor();
        Serial.print("Current inlet pressure : ");
        Serial.println(pressure);
        postPressure = pressureSensor();
        Serial.print("Current outlet pressure : ");
        Serial.println(postPressure);

        // This test is to ensure that the spring housing is not faulty allowing the mains pressure to go through it with no restriction
        if (abs(pressure - postPressure) > inOutDiff)
        {
          Serial.println("Test 1 : Output pressure much lower than input pressure ----> pressure is held");
          springPressureCheck = true;
        }
        else
        {
          Serial.println("Test 1 : Output pressure similar to the input pressure ----> pressure is not held");
        }

        springChecking = springPressureHold(big_dirPin, big_stepPin, inputRelay, outputRelay, holdingLimit); // checking if winding the spring causes expected changes to output pressure

        digitalWrite(TESTING, HIGH);
        pressureHold = pressureDecay(inputRelay, outputRelay,decayLimit); // secondary check to capture any leaks in the system
        digitalWrite(TESTING, LOW);

        if (pressureHold == true)
        {
          digitalWrite(PASS, HIGH);
        }
        else
        {
          digitalWrite(FAIL, HIGH);
        }

        if (pressureHold == true && springPressureCheck == true && springChecking == true)
        { // only if all the previous tests are passed, the output pressure is adjusted to the setpoint
          Serial.println("Successfully holding pressure");

          while (pressureSetCheck == false)
          {
            digitalWrite(inputRelay, HIGH);
            digitalWrite(outputRelay, LOW);

            pressure = pressureSensor();

            Serial.print("Setpoint : ");
            Serial.print(setpoint);
            Serial.print("  ");
            Serial.print("Current pressure : ");
            Serial.print(pressure);
            Serial.println("  ");

            if ((abs(pressure - setpoint)) < setLimit)
            { // Tolerance of 30kPa +/- of the setpoint
              pressureSetCheck = true;
              Serial.println("Set pressure Achieved");
            }
            else
            {
              error = pressure - setpoint;
              pressureAdjustment(error, big_dirPin, big_stepPin, inputRelay, outputRelay, setpoint);
            }
            delay(10);
          }

          digitalWrite(inputRelay, HIGH);
          digitalWrite(outputRelay, LOW);
          delay(250);
          pressure = pressureSensor();
          Serial.print("Pressure test for the pulsing : ");
          Serial.println(pressure);
          secondPulseCheck = pulsingCheck(pressure, inputRelay, outputRelay,pulse,pulseLimit); // pulsing test to check if pressure sticks to the setpoint

          digitalWrite(inputRelay, HIGH);
          digitalWrite(outputRelay, LOW);
          delay(250);
          pressure = pressureSensor();
          Serial.print("Checking for pressure creep after setting the pressure. Pressure set to ");
          Serial.println(pressure);
          digitalWrite(TESTING, HIGH);
          springPressureCheck = springPressure(pressure,creepLimit); // check for pressure creep on the output after pressure setting
          digitalWrite(TESTING, LOW);

          if (springPressureCheck == false)
          {
            digitalWrite(FAIL, HIGH);
          }
          else
          {
            digitalWrite(PASS, HIGH);
          }

          if (secondPulseCheck == true && springPressureCheck == true)
          {
            Serial.print("Pressure holding at setpoint of ");
            Serial.println(setpoint);

            reset(pass); // All tests are complete and the valve is good to go
          }
          else
          {
            Serial.println("Pulsing failed or pressure not holding to the setpoint");
            reset(fail); // Valve failed pulsing test
          }
        }
        else
        {
          Serial.println("Spring issue, pressure on the output not valid");
          reset(fail); // Valve failed due to faulty spring housing
        }
      }
      else
      {
        Serial.println("Leak from the spindle based on operators observation");
        reset(fail); // Valve failed due to leak observed by the operator
      }
    }
    else
    {
      Serial.println("Shut off failed");
      reset(fail); // Shut off function failed
    }
  }
}
