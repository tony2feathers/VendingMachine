#pragma once

#include "esp_secrets.h"
#include <WiFi.h>
#include <SPI.h>
#include <PubSubClient.h>
#include "MqttCommandLookup.h" // Command lookup table header

//*************************************************************
//  CONSTANTS & GLOBALS
//*************************************************************
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

// Wi-Fi credentials (from esp_secrets.h)
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

//*************************************************************
//  EXTERN DECLARATIONS (defined in main.cpp)
//*************************************************************
extern const char DeviceTopic[];
extern const char HostTopic[];
extern const char deviceID[];


// Connection timeouts / intervals
const unsigned long wifiTimeout = 120000; // 2 minutes
const unsigned long mqttTimeout = 120000; // 2 minutes
unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;
unsigned long reconnectDelay = 5000;      // 5 seconds
const unsigned long maxDelay = 60000;     // 1 minute

// Wi-Fi / MQTT status flags
bool wifiConnected = false;
bool mqttConnected = false;

// Create the Wi-Fi and MQTT clients
WiFiClient espClient;
PubSubClient MQTTclient(espClient);

//*************************************************************
//  MQTT CALLBACK USING LOOKUP TABLE
//*************************************************************
void mqttCallback(char* thisTopic, byte* message, unsigned int length) {
    DEBUG_PRINT("Message arrived [");
    DEBUG_PRINT(thisTopic);
    DEBUG_PRINT("]: ");

    // Convert byte array to a null-terminated string
    char messageArrived[length + 1];
    memcpy(messageArrived, message, length);
    messageArrived[length] = '\0'; // Null-terminate

    // Convert the message to lowercase for case-insensitive comparison
    for (unsigned int i = 0; i < length; i++) {
        messageArrived[i] = tolower(messageArrived[i]);
    }

    // Attempt to find the command in the lookup table
    for (const auto& command : mqttCommandLookup) {
        if (strcmp(messageArrived, command.command) == 0) {
            DEBUG_PRINTLN("Executing command: " + String(command.command));
            command.callback(); // Execute the associated function
            return;
        }
    }

    DEBUG_PRINTLN("Unknown MQTT command received: " + String(messageArrived));
}


//*************************************************************
//  WIFI SETUP
//*************************************************************
void wifiSetup() {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, pass);
    WiFi.setAutoReconnect(true);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nWi-Fi connected. IP Address: ");
    Serial.println(WiFi.localIP());
    reconnectDelay = 5000; // Reset delay after successful connection
    wifiConnected = true;
}

//*************************************************************
//  MQTT SETUP (Dynamic Topic)
//*************************************************************
void mqttSetup() {
    MQTTclient.setServer(MQTT_SERVER_IP, 1883);
    MQTTclient.setCallback(mqttCallback);
    MQTTclient.setKeepAlive(120); // Keep the connection alive with a 120-second timeout

    while (!MQTTclient.connected()) {
        Serial.println("Connecting to MQTT broker...");
        if (MQTTclient.connect(deviceID)) {
            Serial.println("Connected to MQTT broker.");
            MQTTclient.subscribe(DeviceTopic); // Dynamically subscribe to the topic
            mqttConnected = true;
        } else {
            Serial.print("MQTT connection failed, state=");
            Serial.println(MQTTclient.state());
            Serial.println("Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

//*************************************************************
//  CHECK WiFi RECONNECT
//*************************************************************
void checkWiFi() {
    // If Wi-Fi is not connected, attempt reconnect after wifiTimeout
    if ((WiFi.status() != WL_CONNECTED) && (millis() - lastWiFiAttempt > wifiTimeout)) {
        lastWiFiAttempt = millis();
        WiFi.reconnect();
        Serial.println("Attempting to reconnect to Wi-Fi...");
    } else {
        wifiConnected = true;
    }
}

//*************************************************************
//  MQTT LOOP / RECONNECT (Dynamic Topic)
//*************************************************************
void mqttLoop() {
    // If MQTT is disconnected, attempt reconnect after mqttTimeout
    if (!MQTTclient.connected()) {
        if (millis() - lastMQTTAttempt > mqttTimeout) {
            lastMQTTAttempt = millis();
            Serial.println("Attempting to reconnect to MQTT...");
            if (MQTTclient.connect(deviceID)) {
                Serial.println("Reconnected to MQTT broker.");
                MQTTclient.subscribe(DeviceTopic);
                mqttConnected = true;
            } else {
                Serial.print("MQTT connection failed, state=");
                Serial.println(MQTTclient.state());
            }
        }
    } else {
        // Stay alive
        MQTTclient.loop();
    }
}
