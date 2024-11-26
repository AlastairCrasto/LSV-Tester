#include <Arduino.h>
#include <FastLED.h>
#include "functions.h"

// initializing all of the variables

#define NUM_LEDS 120
#define LED_PIN 22

CRGB leds[NUM_LEDS];

// button variables
int springCylinder = 38;
int spindleCylinder = 4;
int submergeCylinder = 40;
int cylinderButton = 45;
int spindleButton = 43;
int startButton = 41;
int spindleLeakButton = 39;
int cylinderButtonState = 0;
int spindleButtonState = 0;
int startButtonState = 0;

int airMotorForward = 36;
int airMotorReverse = 34;

// checking to see if both drivers are active
bool springState = false;
bool spindleState = false;

// motor pins  ---> check these
int big_stepPin = 5;
int big_dirPin = 6;
int big_enPin = 7;

// output pressure setpoint ----> need to re-evaluate once HMI display is in action
int setpoint = 500;

// variables used in the loop function for various purposes
float pressure = 0;
float postPressure = 0;
float error = 0;

// input and output solenoids
int inputRelay = 2;
int outputRelay = 3;

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

bool driverCheck = false;
int modeChoice = 0;
bool modeState = false;

// Variables for testing limits

int inOutDiff = 250;
int setLimit = 30;
int shutOffLimit = 40;
int preShutOff = 50;
int decayLimit = 50;
int holdingLimit = 20;
int creepLimit = 20; // 15
int pulse = 3;
int pulseLimit = 20;
int checkpoints = 6;
bool pressureSettingCheck = false;
bool driverDown = false;
bool driverUp = false;

// the reset function resets all of the variables used in the loop function after a pass or fail test to carry out testing for the next valve
void reset(bool status)
{
  if (status == true)
  {
    Serial.println("Testing Complete ---> Valve passed all tests");

    Serial3.print("t2.pco=12000");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.print("t2.txt=\"Valve passed all tests \"");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);

    Serial3.print("t1.pco=0");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.print("t1.txt=\"\"");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);

    for (int i = 0; i < NUM_LEDS - 60; i++)
    {
      leds[i] = CRGB::Green;
    }

    FastLED.show();

    digitalWrite(springCylinder, LOW);
    digitalWrite(spindleCylinder, LOW);
    digitalWrite(inputRelay, LOW);
    digitalWrite(outputRelay, HIGH);
    delay(600);
    digitalWrite(outputRelay, LOW);
    pressureSetCheck = false;
    spindleCheck = false;
    springCheck = false;
    startProgram = false;
    driverCheck = false;
  }
  else if (status == false)
  {

    for (int i = 0; i < NUM_LEDS - 60; i++)
    {
      leds[i] = CRGB::Red;
    }

    FastLED.show();

    digitalWrite(springCylinder, LOW);
    digitalWrite(spindleCylinder, LOW);
    digitalWrite(inputRelay, LOW);
    digitalWrite(outputRelay, HIGH);
    delay(1000);
    digitalWrite(outputRelay, LOW);
    pressureSetCheck = false;
    spindleCheck = false;
    springCheck = false;
    startProgram = false;
    driverCheck = false;

    Serial3.print("t1.pco=0");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.print("t1.txt=\"\"");
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial3.begin(9600);

  pinMode(springCylinder, OUTPUT);
  pinMode(spindleCylinder, OUTPUT);
  pinMode(cylinderButton, INPUT);
  pinMode(spindleButton, INPUT);
  pinMode(startButton, INPUT);
  pinMode(spindleLeakButton, INPUT);
  pinMode(airMotorForward, OUTPUT);
  pinMode(airMotorReverse, OUTPUT);
  pinMode(submergeCylinder, OUTPUT);

  pinMode(big_stepPin, OUTPUT);
  pinMode(big_dirPin, OUTPUT);
  pinMode(big_enPin, LOW);

  pinMode(inputRelay, OUTPUT);
  pinMode(outputRelay, OUTPUT);

  digitalWrite(springCylinder, LOW);
  digitalWrite(spindleCylinder, LOW);

  digitalWrite(inputRelay, LOW);
  digitalWrite(outputRelay, LOW);

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(225);
  for (int i = 0; i < NUM_LEDS - 60; i++)
  {
    leds[i] = CRGB::White;
  }

  FastLED.show();
}
void loop()
{

  if (Serial3.available() && modeState == false)
  {
    delay(30);
    char modeSelection = (Serial3.read());

    if (modeSelection == 'n')
    {
      Serial.println("In normal test mode");
      modeChoice = 1;
      modeState = true;
    }

    else if (modeSelection == 't')
    {
      Serial.println("In manual test mode");
      modeChoice = 2;
      modeState = true;
    }

    else if (modeSelection == 'c')
    {
      Serial.println("In calibration mode");
      modeChoice = 3;
      modeState = true;
    }
  }

  if (modeChoice == 1)
  {

    if (Serial3.available())
    {
      char driverMode = (Serial3.read());

      if (driverMode == 'T')
      {
        Serial3.print("t2.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t2.txt=\"Setpoint of 350 kPa selected\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t0.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t0.txt=\"350kPa\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        setpoint = 350;
        pressureSettingCheck = true;
      }

      if (driverMode == 'I')
      {
        Serial3.print("t2.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t2.txt=\"Setpoint of 500 kPa selected\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t0.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t0.txt=\"500kPa\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        setpoint = 500;
        pressureSettingCheck = true;
      }

      if (driverMode == 'd')
      {
        Serial3.print("t2.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t2.txt=\"Drivers lowered\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        Serial3.print("t4.pco=0");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t4.txt=\"\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        driverUp = true;
        driverCheck = true;
      }

      if (driverMode == 'u')
      {
        Serial3.print("t2.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t2.txt=\"Drivers raised\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        driverDown = true;
        driverCheck = false;
      }

      if (driverMode == 's' && driverCheck == true && pressureSettingCheck == true)
      {
        Serial3.print("t2.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t2.txt=\"Test procedure begun\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        startProgram = true;
      }

      if (driverMode == 'b' && driverCheck == false)
      {
        Serial.println("Going back to selection mode");
        modeChoice = 0;
        modeState = false;
      }
    }

    if (driverDown == HIGH)
    {
      springState = false;
      spindleState = false;

      // digitalWrite(springCylinder, springState);
      digitalWrite(spindleCylinder, spindleState);

      delay(200);

      driverDown = false;
    }

    if (driverUp == HIGH)
    {
      springState = true;
      spindleState = true;

      // digitalWrite(springCylinder, springState);
      digitalWrite(spindleCylinder, spindleState);

      driverUp = false;
    }

    if (driverCheck == true && startProgram == true)
    {
      Serial.println("Entered main program function");

      digitalWrite(inputRelay, HIGH);
      digitalWrite(outputRelay, HIGH);
      delay(500);

      pressureHold = pressureDecay(inputRelay, outputRelay, decayLimit); // secondary check to capture any leaks in the system

      shutOffCheck = shutOffFunction(airMotorForward, airMotorReverse, inputRelay, outputRelay, shutOffLimit, preShutOff); // Tests shut off functionality

      if (shutOffCheck == true)
      {
        spindleLeak = spindleReversal(airMotorForward, airMotorReverse, spindleCylinder, springCylinder, inputRelay, outputRelay, submergeCylinder); // operators to test for leaks

        digitalWrite(inputRelay, LOW);
        digitalWrite(outputRelay, HIGH);

        if (spindleLeak == true && pressureHold == true)
        { // operator passes the valve after observing no leaks and the pressure was held during decay test

          pressure = inletPressureSensor();
          Serial.print("Current inlet pressure : ");
          Serial.println(pressure);
          postPressure = pressureSensor();
          Serial.print("Current outlet pressure : ");
          Serial.println(postPressure);

          Serial3.print("t2.pco=63015");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("t2.txt=\"Testing catridge\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);

          // This test is to ensure that the spring housing is not faulty by not allowing the mains pressure to go through it with no restriction
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

          // if (pressureHold == true && springPressureCheck == true && springChecking == true)
          if (springPressureCheck == true && springChecking == true)
          { // only if all the previous tests are passed, the output pressure is adjusted to the setpoint
            Serial.println("Successfully holding pressure");

            Serial3.print("t2.pco=63015");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.print("t2.txt=\"Pressure setting in action\"");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);

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
              {
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

            Serial3.print("t2.pco=63015");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.print("t2.txt=\"Pulsing test\"");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);

            pressure = pressureSensor();
            Serial.print("Pressure test for the pulsing : ");
            Serial.println(pressure);
            secondPulseCheck = pulsingCheck(pressure, inputRelay, outputRelay, pulse, pulseLimit); // pulsing test to check if pressure sticks to the setpoint

            digitalWrite(inputRelay, HIGH);
            digitalWrite(outputRelay, LOW);
            delay(250);
            pressure = pressureSensor();
            Serial.print("Checking for pressure creep after setting the pressure. Pressure set to ");
            springPressureCheck = springPressure(pressure, creepLimit); // check for pressure creep on the output after pressure setting

            if (secondPulseCheck == true && springPressureCheck == true)
            {
              Serial.print("Pressure holding at setpoint of ");
              Serial.println(setpoint);

              Serial3.print("t4.pco=12000");
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.print("t4.txt=\"PASS\"");
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.write(0xff);

              reset(pass); // All tests are complete and the valve is good to go
            }
            else
            {
              Serial.println("Pulsing failed or pressure not holding to the setpoint");

              Serial3.print("t2.pco=63488");
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.print("t2.txt=\"Pulse/Pressure setting failed\"");
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.write(0xff);

              Serial3.print("t4.pco=63488");
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.print("t4.txt=\"FAIL\"");
              Serial3.write(0xff);
              Serial3.write(0xff);
              Serial3.write(0xff);
              reset(fail); // Valve failed pulsing test
            }
          }
          else
          {
            Serial.println("Spring issue, pressure on the output not valid");

            Serial3.print("t2.pco=63488");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.print("t2.txt=\"Catridge not functioning correctly\"");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);

            Serial3.print("t4.pco=63488");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.print("t4.txt=\"FAIL\"");
            Serial3.write(0xff);
            Serial3.write(0xff);
            Serial3.write(0xff);
            reset(fail); // Valve failed due to faulty spring housing
          }
        }
        else
        {
          Serial.println("Leak from the spindle based on operators observation");

          Serial3.print("t2.pco=63488");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("t2.txt=\"Operator fail/Leak \"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);

          Serial3.print("t4.pco=63488");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("t4.txt=\"FAIL\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          reset(fail); // Valve failed due to leak observed by the operator
        }
      }
      else
      {
        Serial3.print("t2.pco=63488");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t2.txt=\"Shut off failure\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        Serial3.print("t4.pco=63488");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("t4.txt=\"FAIL\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        reset(fail); // Shut off function failed
      }
    }
  }
  else if (modeChoice == 2)
  {
    if (Serial3.available())
    {
      char buttonSelect = (Serial3.read());

      if (buttonSelect == 'b')
      {
        Serial.println("Going back to selection mode");
        modeChoice = 0;
        modeState = false;
      }

      if (buttonSelect == 'a')
      {
        Serial.println("Drivers raised");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Drivers raised\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(spindleCylinder, LOW);
        digitalWrite(springCylinder, LOW);
      }

      if (buttonSelect == 'c')
      {
        Serial.println("Drivers lowered");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Drivers lowered\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(spindleCylinder, HIGH);
        digitalWrite(springCylinder, HIGH);
      }

      if (buttonSelect == 'd')
      {
        Serial.println("Submerge raise");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Valve mount ascending\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(submergeCylinder, LOW);
        digitalWrite(inputRelay, LOW);
      }

      if (buttonSelect == 'e')
      {
        Serial.println("Submerge raise");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Valve mount descending\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(submergeCylinder, HIGH);
        digitalWrite(spindleCylinder, LOW);
        digitalWrite(springCylinder, LOW);
        digitalWrite(inputRelay, HIGH);
      }

      if (buttonSelect == 'f')
      {
        Serial.println("spindle open");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Opening spindle\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(airMotorReverse, HIGH);
        delay(150);
        digitalWrite(airMotorReverse, LOW);
      }

      if (buttonSelect == 'g')
      {
        Serial.println("spindle closing");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Closing spindle\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(airMotorForward, HIGH);
        delay(150);
        digitalWrite(airMotorForward, LOW);
      }

      if (buttonSelect == 'h')
      {
        Serial.println("Input pressure on");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Incoming pressure started\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(inputRelay, HIGH);
      }

      if (buttonSelect == 'i')
      {
        Serial.println("Input pressure off");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Incoming pressure stopped\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(inputRelay, LOW);
        digitalWrite(outputRelay, HIGH);
        delay(1000);
        digitalWrite(outputRelay, LOW);
      }

      if (buttonSelect == 'j')
      {
        Serial.println("Ending test program");
        Serial3.print("status.pco=65535");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Ending test program\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(inputRelay, LOW);
        digitalWrite(outputRelay, HIGH);
        delay(1000);
        digitalWrite(outputRelay, LOW);
        digitalWrite(spindleCylinder, LOW);
        digitalWrite(springCylinder, LOW);
        digitalWrite(submergeCylinder, LOW);
      }

      if (buttonSelect == 'k')
      {
        Serial.println("leak test");
        Serial3.print("status.pco=63015");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Leak test\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        bool pressureLeak = pressureDecay(inputRelay, outputRelay, decayLimit);

        if (pressureLeak == false)
        {
          Serial3.print("status.pco=63488");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Leak detected\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
        else
        {
          Serial3.print("status.pco=12000");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Leak test passed\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
      }

      if (buttonSelect == 's')
      {
        Serial.println("leak test");
        Serial3.print("status.pco=63015");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Shut off test\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        bool shutOffLeak = shutOffFunction(airMotorForward, airMotorReverse, inputRelay, outputRelay, shutOffLimit, preShutOff);

        if (shutOffLeak == false)
        {
          Serial3.print("status.pco=63488");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Shut off failed\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
        else
        {
          Serial3.print("status.pco=12000");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Shut off achieved\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
      }

      if (buttonSelect == 'p')
      {
        Serial.println("pulse test");
        Serial3.print("status.pco=63015");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Pulse test\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(inputRelay, HIGH);
        digitalWrite(outputRelay, LOW);
        delay(250);

        float currentPressure = pressureSensor();
        Serial.print("Pressure test for the pulsing : ");
        Serial.println(currentPressure);
        bool pulseCheck = pulsingCheck(currentPressure, inputRelay, outputRelay, pulse, pulseLimit); // pulsing test to check if pressure sticks to the setpoint

        digitalWrite(inputRelay, HIGH);
        digitalWrite(outputRelay, LOW);

        if (pulseCheck == false)
        {
          Serial3.print("status.pco=63488");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Pulse test failed\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
        else
        {
          Serial3.print("status.pco=12000");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Pulse test passed\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
      }

      if (buttonSelect == 'm')
      {
        Serial.println("pressure creep test");
        Serial3.print("status.pco=63015");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.print("status.txt=\"Pressure creep test\"");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);

        digitalWrite(inputRelay, HIGH);
        digitalWrite(outputRelay, LOW);
        delay(250);

        float setPressure = pressureSensor();
        bool creepTest = springPressure(setPressure, creepLimit);

        if (creepTest == false)
        {
          Serial3.print("status.pco=63488");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Pressure creeping\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
        else
        {
          Serial3.print("status.pco=12000");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.print("status.txt=\"Pressure is holding at setpoint\"");
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
      }
    }
  }
  else if (modeChoice == 3)
  {
    if (Serial3.available())
    {
      char buttonSelect = (Serial3.read());

      if (buttonSelect == 'b')
      {
        Serial.println("Going back to selection mode");
        modeChoice = 0;
        modeState = false;
      }
    }
  }
}
