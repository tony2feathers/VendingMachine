//+------------------------------------------------------------------------
//
// Two Feathers LLC - (c) 2022 Robert Nelson. All Rights Reserved.
//
// File: main.cpp
//
// Description:
//      Program for controlling the vending machine and starting the marble run puzzle
//
// History:
//      SEP-24-2022       tony2feathers     Created
//      OCT-13-2023       tony2feathers     Changed to TB6600 Motor Driver
//                                          and added exButton library/functions
//                                          for limit switches in place of IR sensors.
//      FEB-8-2025        tony2feathers     Added revised MQTT and Wi-Fi functionality
//-------------------------------------------------------------------------

// INCLUDES
#include <Arduino.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include "esp_secrets.h"
#include <lights.h>
#include <ezButton.h>
#include <WifiFunctions.h> // Our new Wi-Fi & MQTT handling header

// I think these are remnants of old code??
long lastMsg = 0;
char msg[50];
int value = 0;

const long interval = 1000;
unsigned long curMillis = 0;
unsigned long curTime = 0;
unsigned long preTime = 0;
unsigned long previousMillis = 0;
const long coinInt = 500;

// Define project-specific MQTT topics and device ID
const char DeviceTopic[] = "ToDevice/VendingMachine";
const char hostTopic[] = "ToHost/VendingMachine";
const char deviceID[] = "VendingMachine";


int count = 0;

// Variable to count the balls currently in the escalator
int ballReturn = 0;
static bool sensorActive = false; // Are we currently counting a "blocked" state?
static unsigned long lastUnblockTime = 0;
const unsigned long debounceTime = 200; // 200 ms for sensor transitions

// Attach the coin counter to pin 21 and define other things related to the coin counter
const byte coinCounter = 21;

// Number of impulses detected
volatile int impulseCount = 0;

// Number of pulses for a coin detected
const int targetPulses = 1;

// Define pins for the Nema17 Stepper motor used for the dispenser
const byte DispenserDir = 14;
const byte DispenserStep = 26;
// Define motor interface type
#define motorInterfaceType 1

// define pins for the motor used for the escalator
const byte MotorA = 4;
const byte MotorIN1 = 19;
const byte MotorIN2 = 18;

bool motorRunning = false;

// define pins for the linear actuator
const byte MotorB = 16;
const byte MotorIN3 = 5;
const byte MotorIN4 = 17;

// Initiate the instance of the stepper motor
AccelStepper stepper = AccelStepper(motorInterfaceType, DispenserStep, DispenserDir);

// Setting PWM properties to control motor speed
const int freq = 500;
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int resolution = 8;

#ifndef DEBUG
#define DEBUG
#endif

//*************************************************************************
// FUNCTION DECLARATIONS
//*************************************************************************
void IRAM_ATTR incomingImpuls(); // for the coin interrupt

// Puzzle logic (inbound MQTT message functions)
void onDispense();
void onStepForward();
void onStepBackward();
void onStop();
void onOpen();
void onClose();
void onReverse();
void onForward();
void escalatorStart();
void escalatorStop();

// define buttons for entrance and exit of escalator
ezButton entranceButton(23);
ezButton exitButton(22);

//****************************LIGHTS**********************************
// Strip of LEDs includes two segments; one for static lighting and one for motion lighting
const int Strip1_Pin = 32;
uint16_t Strip1_NUM_LEDS = 53;

const int Strip1AStart = 0;
const int Strip1BStart = 29;

const int Strip1ALEN = 29;
const int Strip1BLEN = 24;

// Create instance of NeoPixel for the strip of LEDs
NeoPatterns Strip1leds(Strip1_NUM_LEDS, Strip1_Pin, NEO_GRB + NEO_KHZ800, nullptr);

//*************************************************************************
// SETUP()
//*************************************************************************
void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Setting up DC Motors...");
#endif

  pinMode(MotorA, OUTPUT);
  pinMode(MotorB, OUTPUT);

  // attach the channel to the GPIO to be controlled
  ledcSetup(pwmChannelA, freq, resolution);
  ledcAttachPin(MotorA, pwmChannelA);
  ledcSetup(pwmChannelB, freq, resolution);
  ledcAttachPin(MotorB, pwmChannelB);

  // Turn off motors - Initial State
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);

  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  pinMode(MotorIN1, OUTPUT);
  pinMode(MotorIN2, OUTPUT);
  motorRunning = false;

  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  pinMode(MotorIN3, OUTPUT);
  pinMode(MotorIN4, OUTPUT);
  delay(500);

#ifdef DEBUG
  Serial.println("All motors should be off!");
#endif

  //***************************
  // Setup the wifi and MQTT connections
  //***************************
  wifiSetup();
  mqttSetup();
  delay(500);
  Serial.println("Connected to WIFI and MQTT!");

  // Configure stepper motor
  pinMode(33, OUTPUT); // stepper enable (if used)
  stepper.setMinPulseWidth(3);
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(50);
  stepper.setSpeed(100);
  stepper.setCurrentPosition(0);
  delay(500);
  Serial.println("Stepper Motor settings established!");

  // Quick test: rotate forward 100 steps
  stepper.enableOutputs();
  while (stepper.currentPosition() != 100)
  {
    stepper.setSpeed(200);
    stepper.runSpeed();
  }
  delay(1500);
  stepper.disableOutputs();
  Serial.println("Stepper motor test complete!");

  // Set coinCounter pin and button pins
  pinMode(coinCounter, INPUT);
  entranceButton.setDebounceTime(5);
  exitButton.setDebounceTime(5);
  delay(500);
  Serial.println("Entrance and exit buttons established");

  // Interrupt connected to pin 21 executing IncomingImpuls
  // function when signal goes from High to Low
  attachInterrupt(digitalPinToInterrupt(coinCounter), incomingImpuls, FALLING);

  //***************************
  // Initialize the NeoPixel strip
  //***************************
#ifdef DEBUG
  Serial.println("LED's initializing");
#endif

  Strip1leds.begin();
  Strip1leds.show();
  Strip1leds.setBrightness(175);

  // Cycle the lights blue
  for (int x = 0; x < Strip1_NUM_LEDS; x++)
  {
    Strip1leds.setPixelColor(x, Strip1leds.Color(0, 0, 255));
  }
  Strip1leds.show();
  delay(500);

  // Cycle lights off
  for (int y = 0; y < Strip1_NUM_LEDS; y++)
  {
    Strip1leds.setPixelColor(y, Strip1leds.Color(0, 0, 0));
  }
  Strip1leds.show();
  delay(500);
  Serial.println("LED's tested, turning on ambient lights!");

  Strip1leds.ColorSet(Strip1leds.Color(204, 85, 0), Strip1BStart, Strip1BLEN);
  Serial.println("Checking Strip 1A");
  Strip1leds.CylonEye(Strip1leds.Color(255, 0, 0), 100, Strip1AStart, Strip1ALEN, 3, 10, forward);
  Strip1leds.Update();
  Strip1leds.ColorSet(Strip1leds.Color(0, 0, 0), Strip1AStart, Strip1ALEN);
  Strip1leds.Update();
  Serial.println("Setup Complete!");
  delay(500);
}

//*************************************************************************
// LOOP()
//*************************************************************************
void loop()
{
  entranceButton.loop();
  exitButton.loop();

  curTime = millis();

  // If we have received a coin, activate stuff
  if ((curTime - preTime >= coinInt) && (impulseCount == targetPulses))
  {
    Serial.print("Coin counted! impulseCount = ");
    Serial.println(impulseCount);
    Serial.println("Dispensing due to coin interrupt");
    onDispense();
    impulseCount = 0; // Reset
  }

  // Ball enters escalator (leading-edge detection)
  if (entranceButton.isPressed())
  {
    // If we just went from "not pressed" to "pressed"
    if (!sensorActive)
    {
      // Mark sensor as active
      sensorActive = true;

      // Increment the ball count
      ballReturn++;
#ifdef DEBUG
      Serial.print("Balls in the return: ");
      Serial.println(ballReturn);
#endif

      // If the motor is currently off, start it and run the scanner pattern
      if (!motorRunning)
      {
        // Start the Scanner pattern
        Strip1leds.Scanner(
            Strip1leds.Color(0, 119, 178),
            Strip1leds.Color(0, 119, 178),
            20,
            Strip1AStart, Strip1ALEN,
            7,
            Strip1BStart, Strip1BLEN,
            5);
        Strip1leds.Update();

        // Turn on the escalator motor
        ledcWrite(pwmChannelA, 255);
        digitalWrite(MotorIN1, HIGH);
        digitalWrite(MotorIN2, LOW);
        motorRunning = true;
      }
      else
      {
        // Motor is already running, so just update the lights
        Strip1leds.Update();
      }

      // Notify the host
      MQTTclient.publish(hostTopic, "Ball entered escalator");
    }
  }
  else
  {
    // Sensor is not pressed (unblocked)
    // If we were active, reset after a short wait or instantly
    if (sensorActive)
    {
      sensorActive = false;
      lastUnblockTime = millis();
    }
  }

  // Ball exits escalator
  if (exitButton.isPressed())
  {
    ballReturn--;
    if (ballReturn < 0)
    {
      ballReturn = 0;
      Serial.println("Ball Return reset to 0");
      Strip1leds.ColorSet(Strip1leds.Color(204, 85, 0), Strip1BStart, Strip1BLEN);
      Strip1leds.ColorSet(Strip1leds.Color(0, 0, 0), Strip1AStart, Strip1ALEN);
      Strip1leds.show();
    }
    MQTTclient.publish(hostTopic, "Ball exited escalator");

#ifdef DEBUG
    Serial.print("Ball exited escalator! Balls in the return = ");
    Serial.println(ballReturn);
#endif
  }

  // If ballReturn hits zero, stop the escalator
  if ((ballReturn == 0) && motorRunning)
  {
    ledcWrite(pwmChannelA, 0);
    digitalWrite(MotorIN1, LOW);
    digitalWrite(MotorIN2, LOW);
    motorRunning = false;
    Strip1leds.ColorSet(Strip1leds.Color(204, 85, 0), Strip1BStart, Strip1BLEN);
    Strip1leds.ColorSet(Strip1leds.Color(0, 0, 0), Strip1AStart, Strip1ALEN);
    delay(500);
    Strip1leds.show();
#ifdef DEBUG
    Serial.println("Ball return is now empty!");
#endif
  }
  else if (ballReturn > 0 && motorRunning)
  {
    Strip1leds.Update();
  }

  //***********************************************************
  // Handle Wi-Fi / MQTT reconnections
  //***********************************************************
  checkWiFi(); // from WifiFunctions.h
  mqttLoop();  // from WifiFunctions.h
}

//*************************************************************************
// INTERRUPT SERVICE ROUTINE for coin
//*************************************************************************
void IRAM_ATTR incomingImpuls()
{
  impulseCount = 1;
  preTime = curTime;
}

//*************************************************************************
// PUZZLE / MQTT COMMAND HANDLERS
//*************************************************************************
void onDispense()
{
  stepper.enableOutputs();
  Serial.println("Ball dispense activated!");
  MQTTclient.publish(hostTopic, "Ball dispense activated!");

  delay(500);
  stepper.setCurrentPosition(0);
  while (stepper.currentPosition() != 100)
  {
    stepper.setSpeed(200);
    stepper.runSpeed();
  }
  stepper.disableOutputs();
}

void onStepForward()
{
  stepper.enableOutputs();
#ifdef DEBUG
  Serial.println("Command to step motor forward (1 step) received");
#endif
  stepper.setCurrentPosition(0);
  while (stepper.currentPosition() != 1)
  {
    stepper.setSpeed(100);
    stepper.runSpeed();
  }
  stepper.disableOutputs();
}

void onStepBackward()
{
  stepper.enableOutputs();
#ifdef DEBUG
  Serial.println("Command to step motor backward (1 step) received");
#endif
  stepper.setCurrentPosition(0);
  while (stepper.currentPosition() != -1)
  {
    stepper.setSpeed(-100);
    stepper.runSpeed();
  }
  stepper.disableOutputs();
}

void onStop()
{
  // Force all motors to come to a stop
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
  delay(500);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  motorRunning = false;
#ifdef DEBUG
  Serial.println("All motor stop command received!");
#endif
  MQTTclient.publish(hostTopic, "All motor stop command received!");
}

void onOpen()
{
  // Activate the linear actuator for 2 seconds (example)
  ledcWrite(pwmChannelB, 255);
  delay(500);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, HIGH);
  delay(2000);
  ledcWrite(pwmChannelB, 0);
  delay(500);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);

#ifdef DEBUG
  Serial.println("Coin OPEN command received");
#endif
  MQTTclient.publish(hostTopic, "Coin OPEN command received");
}

void onClose()
{
  ledcWrite(pwmChannelB, 255);
  digitalWrite(MotorIN3, HIGH);
  digitalWrite(MotorIN4, LOW);
  delay(2000);
  ledcWrite(pwmChannelB, 0);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);

#ifdef DEBUG
  Serial.println("Coin CLOSE command received");
#endif
  MQTTclient.publish(hostTopic, "Coin CLOSE command received");
}

void onReverse()
{
  ledcWrite(pwmChannelA, 255);
#ifdef DEBUG
  Serial.println("Reverse motor command received");
#endif
  MQTTclient.publish(hostTopic, "Reverse motor command received");
  motorRunning = true;
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, HIGH);
  delay(3500);
  ledcWrite(pwmChannelA, 0);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  motorRunning = false;
}

void onForward()
{
  ledcWrite(pwmChannelA, 255);
#ifdef DEBUG
  Serial.println("Forward motor command received");
#endif
  MQTTclient.publish(hostTopic, "Forward motor command received");
  delay(500);
  digitalWrite(MotorIN1, HIGH);
  digitalWrite(MotorIN2, LOW);
  motorRunning = true;
  delay(3500);
  ledcWrite(pwmChannelA, 0);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  motorRunning = false;
}

void escalatorStart()
{
  // If the motor is already running, no need to start again
  if (!motorRunning)
  {
    ballReturn++;

    // Turn on the escalator motor
    ledcWrite(pwmChannelA, 255);
    digitalWrite(MotorIN1, HIGH);
    digitalWrite(MotorIN2, LOW);
    motorRunning = true;

    // Optional: feedback to MQTT or Serial
    MQTTclient.publish(hostTopic, "Escalator forced to start");
    Serial.println("Escalator forced to start");
  }
  else
  {
    // Motor is already running
    MQTTclient.publish(hostTopic, "Escalator already running");
  }
}

void escalatorStop()
{
  // Immediately stop the escalator motor
  ledcWrite(pwmChannelA, 0);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  motorRunning = false;

  // If you want to reset the ball counter, do so here:
  ballReturn = 0;

  // Optional: feedback to MQTT or Serial
  MQTTclient.publish(hostTopic, "Escalator forced to stop");
  Serial.println("Escalator forced to stop");
}
