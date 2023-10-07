//+------------------------------------------------------------------------
//
// Two Feathers LLC - (c) 2022 Robert Nelson. All Rights Reserved.
//
//File: main.cpp
//
// Description:
//
//      Program for controlling the vending machine and starting the marble run puzzle
//
// History:     SEP-24-2022       tony2feathers     Created
//
//-------------------------------------------------------------------------

//INCLUDES
#include <Arduino.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Stepper.h>
#include <Adafruit_NeoPixel.h>
#include "esp_secrets.h"
#include <lights.h>

// GLOBALS

// Constants
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;  // the WiFi radio's status

const char* mqtt_server = "10.1.10.55";

WiFiClient espClient;
PubSubClient client(espClient);

// I think these are remnants of old code??
long lastMsg = 0;
char msg[50];
int value = 0;

const long interval = 1000;
unsigned long curMillis = 0;
unsigned long curTime = 0;
unsigned long preTime = 0;
unsigned long previousMillis = 0;
const long coinInt = 1000;


// constants for MQTT
const char topic[] = "ToDevice/VendingMachine";
const char hostTopic[] = "ToHost/VendingMachine";
const char* deviceID = "VendingMachine";

int count = 0;

// Variable to count the balls currently in the escalator
int ballReturn = 0;

// Attach the coin counter to pin 34 and define other things related to the coin counter
const byte coinCounter = 34;

// Number of impulses detected
volatile int impulseCount = 0;

// Number of pulses for a coin detected
const int targetPulses = 1;

// Variable to measure intervals between imposes


// Define pins for the Nema17 Stepper motor used for the dispenser
const byte Dispenser1 = 14;
const byte Dispenser2 = 27;
const byte Dispenser3 = 26;
const byte Dispenser4 = 25;

// define pins for the motor used for the escalator
const byte MotorA = 4;
const byte MotorIN1 = 19;
const byte MotorIN2 = 18;

// define pins for the linear actuator
const byte MotorB = 16;
const byte MotorIN3 = 5;
const byte MotorIN4 = 17;

// Initiate the instance of the stepper motor
const int stepsPerRevolution = 200;
Stepper dispenserStepper(stepsPerRevolution, 14, 27, 26, 25);

// DEFINES
/*
// Setting PWM properties to control motor speed
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 200;
*/

#ifndef DEBUG         

#define DEBUG       

// define IR sensor pins
const int IRentrancePin = 23;
const int IRexitPin = 22;

// ***************************LIGHTS*************************

// Strip of LEDs includes two segments; one for static lighting and one for motion lighting
const int Strip1_Pin = 32;
uint16_t Strip1_NUM_LEDS = 53;

const int Strip1AStart = 0;
const int Strip1BStart = 29;

const int Strip1ALEN = 29;
const int Strip1BLEN = 24;


// Create instance of NeoPixel for the strip of LEDs
NeoPatterns Strip1leds(Strip1_NUM_LEDS, Strip1_Pin, NEO_GRB+NEO_KHZ800, nullptr);

//************INTERRUPT FUNCTIONS****************/
void IRAM_ATTR incomingImpuls();

//**********************INBOUND MQTT MESSAGE FUNCTIONS*********************/
void onDispense() {
  dispenserStepper.step(100);  // Turn the stepper 180 degrees
  Serial.println("Ball dispense activated. Players should now have a ball in the marble run!");
  //client.publish(hostTopic, "Ball dispense activated. Players should now have a ball in the marble run!");   
  //preTime = curTime;
  impulseCount = 0;
  /*Serial.println(impulseCount);
  Serial.print("Current time is ");
  Serial.println(curTime);
  Serial.print("Previous Time is ");
  Serial.println(preTime);*/
}

void onStop() {
  // Force all motors to come to a stop immediately
  analogWrite(MotorA, 0);
  analogWrite(MotorB, 0);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  #ifdef DEBUG
  Serial.print("All motor stop command received!");
  #endif
  client.publish(hostTopic, "All motor stop command received!");
}

void onOpen() {
  // Activate the linear actuator for 5 seconds
  analogWrite(MotorB, 255);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, HIGH);
  delay(500);
  analogWrite(MotorB, 0);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  #ifdef DEBUG
  Serial.print("Coin OPEN command received");
  #endif
  client.publish(hostTopic, "Coin OPEN command received");
}

void onClose() {
  // Activate the linear actuator in the other direction for 5 seconds
  analogWrite(MotorB, 255);
  digitalWrite(MotorIN3, HIGH);
  digitalWrite(MotorIN4, LOW);
  delay(500);
  analogWrite(MotorB, 0);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  #ifdef DEBUG
  Serial.print("Coin CLOSE command received");
  #endif
  client.publish(hostTopic, "Coin CLOSE command received");
}

void onReverse() {
  analogWrite(MotorA, 255);
  digitalWrite(MotorIN1, HIGH);
  digitalWrite(MotorIN2, LOW);
  #ifdef DEBUG
  Serial.print("Reverse motor command received");
  #endif
  client.publish(hostTopic, "Reverse motor command received");
  delay(500);
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, HIGH);
}


//************WIFI and MQTT FUNCTIONS************/
void callback(char* thisTopic, byte* message, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(thisTopic);
  Serial.print("] ");
  Serial.println("Message: ");
  
  // Convert byte array to C-style string
  char messageArrived[length + 1];
  memcpy(messageArrived, message, length);
  messageArrived[length] = '\0'; // Null-terminate the string
 
  for (unsigned int i = 0; i < length; i++) {
    messageArrived[i] = tolower(messageArrived[i]);
  }

  // Use strcmp to look for certain messages
  if (strcmp(messageArrived, "dispense") == 0){
    onDispense();
  } else if (strcmp(messageArrived, "reverse") == 0){
    onReverse();
  } else if (strcmp(messageArrived, "stop") == 0){
    onStop();
  } else if (strcmp(messageArrived, "open") == 0){
    onOpen();
  } else if (strcmp(messageArrived, "close") == 0){
    onClose();
  }
  Serial.println(messageArrived);
  Serial.println();
}

void wifiSetup() {
  Serial.println();
  Serial.println("****************************");
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP Address: ");
  Serial.print(WiFi.localIP());
}

void MQTTsetup() {
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.subscribe(topic);
  while (!client.connected()) {

    // Debug info
    Serial.print("Attempting to connect to the MQTT broker at ");
    Serial.println(mqtt_server);

    // Attempt to connect
    if (client.connect(deviceID)) {

      // Debug info
      Serial.println("Connected to MQTT broker");

      // Once connected, publish an announcement to the host
      client.publish(hostTopic, "SPADS Connected!");
      // Subscribe to topics meant for this device
      client.subscribe(topic);
      Serial.println("Subscribed to topic: ");
      Serial.println(topic);
    } else {
      // Debug info
      Serial.print("Failed to connect to MQTT broker, rc =");
      Serial.print(client.state());
      Serial.println("Retrying in 5 seconds...");
      // Wait 5 seconds before retrying
      delay(500);
    }
  }
}

//************SETUP FUNCTION********************
void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Setup the wifi and MQTT connections
  wifiSetup();
  MQTTsetup();
  Serial.print("Connected to WIFI and MQTT!");

  // Set the speed of the dispenser stepper motor
  dispenserStepper.setSpeed(60);

  // Set all the control pins to outputs and inputs as appropriate.
  pinMode(coinCounter, INPUT);
  pinMode(IRentrancePin, INPUT);
  pinMode(IRexitPin, INPUT);

  // Interrupt connected to pin 34 executing IncomingImpuls function when signal goes from High to Low
  attachInterrupt(digitalPinToInterrupt(coinCounter), incomingImpuls, FALLING);

  pinMode(MotorA, OUTPUT);
  pinMode(MotorIN1, OUTPUT);
  pinMode(MotorIN2, OUTPUT);

  pinMode(MotorB, OUTPUT);
  pinMode(MotorIN3, OUTPUT);
  pinMode(MotorIN4, OUTPUT);

  pinMode(Dispenser1, OUTPUT);
  pinMode(Dispenser2, OUTPUT);
  pinMode(Dispenser3, OUTPUT);
  pinMode(Dispenser4, OUTPUT);

  // Turn off motors - Initial State
  digitalWrite(MotorIN1, LOW);
  digitalWrite(MotorIN2, LOW);
  digitalWrite(MotorIN3, LOW);
  digitalWrite(MotorIN4, LOW);
  Serial.println("All motors should be off! Setup complete");

  // attach the channel to the GPIO to be controlled
  //ledcAttachPin(MotorA, pwmChannel);

  // Initialize the NeoPixel strip objects
#ifdef DEBUG
  Serial.println("LED's initializing");
#endif

  // Initialize the NeoPixel strip objects
  Strip1leds.begin();
  Strip1leds.show();
  Strip1leds.setBrightness(75);

  for (int x = 0; x < Strip1_NUM_LEDS; x++) {
    // Cycle the lights blue
    Strip1leds.setPixelColor(x, Strip1leds.Color(0, 0, 255));
  }
  Strip1leds.show();
  delay(500);

  for (int y = 0; y < Strip1_NUM_LEDS; y++) {
    // Cycle the lights off
    Strip1leds.setPixelColor(y, Strip1leds.Color(0, 0, 0));
  }
  Strip1leds.show();
  delay(500);
  Serial.println("LED's tested, turning on ambient lights!");

  Strip1leds.ColorSet(Strip1leds.Color(204, 85, 0), Strip1BStart, Strip1BLEN);
  Serial.println("Setup Complete!");
}

//***************MAIN PROGRAM*******************/
void loop() {  
  curTime = millis();

  //If we have received a coin, activate stuff
  if (curTime - preTime >= coinInt and impulseCount==targetPulses) {
    Serial.print("Coin counted!");
    //Serial.println(impulseCount);
    onDispense(); 
    //impulseCount=0;            // Reset the impulse counter
  }

  //If we haven't received a coin, keep waiting
  else {
  }
  /*
  Serial.print("Current Time = ");
  Serial.println(curTime);
  delay(1000);
  Serial.print("Previous Time = ");
  Serial.println(preTime);
  delay(1000);
  Serial.print("Impulse Count = ");
  Serial.println(impulseCount);
  */

  int IRentrance = digitalRead(IRentrancePin);
  if (IRentrance == LOW) {
    Serial.print("Ball entrance triggered!");
    ballReturn++;
    Serial.print(ballReturn);
    Strip1leds.Scanner(Strip1leds.Color(0, 119, 178), 50, Strip1AStart, Strip1_NUM_LEDS);
    analogWrite(MotorA, 255);
    digitalWrite(MotorIN1, HIGH);
    digitalWrite(MotorIN2, LOW);
    client.publish(hostTopic, "Ball entered escalator");
  }

  int IRexit = digitalRead(IRexitPin);
  if (IRexit == LOW){
    Serial.print("Ball exit triggered!");
    Serial.print(ballReturn);
    ballReturn--;
    client.publish(hostTopic, "Ball exited escalator");
    Strip1leds.ColorSet(Strip1leds.Color(204, 85, 0), Strip1BStart, Strip1BLEN);
    Strip1leds.ColorSet(Strip1leds.Color(0,0,0), Strip1AStart, Strip1ALEN);
  }
  if (ballReturn == 0) {
    analogWrite(MotorA, 0);
    digitalWrite(MotorIN1, LOW);
    digitalWrite(MotorIN2, LOW);
  }

  //delay(500);
  client.loop();



  }
  void IRAM_ATTR incomingImpuls() {
  impulseCount = 1;
  preTime = curTime;
  }
#endif