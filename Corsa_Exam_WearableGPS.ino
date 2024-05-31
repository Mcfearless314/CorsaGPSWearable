#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <vector>
#include <time.h>

// LCD and GPS
LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;
static const int RXPin = 4, TXPin = 36;
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin);

// WiFi and MQTT
const char* ssid = "OnePlus 8T";
const char* password = "d9388qji";
const char* mqtt_server = "mqtt.flespi.io";
const int mqttPort = 1883;
const char* mqttUser = "FlespiToken gRjwJtetQRQNxY3bmDp8iWg469NFk8yPfQDhHj0OMdScYTbYWTJWpe7yjdVXg6DI";
const char* mqttPassword = "";

WiFiClient espClient;
PubSubClient client(espClient);

// Define topics
const char* registrationCheckTopic = "devices/registration/check";
String registrationResponseTopic;
String registrationSuccessTopic;
const char* dataTopic = "gps/data";

// Device Registration
bool isRegistered = false;
String deviceID;

// Data logging
struct GpsData {
  double latitude;
  double longitude;
  time_t timestamp;
};

std::vector<GpsData> gpsLog;
unsigned long lastLogTime = 0;
unsigned long startTime = 0;
const unsigned long logInterval = 10000; // 10 seconds

bool isLogging = false;
bool wifiConnected = false;
bool confirmationMode = false;
bool runFinished = false;

#define BUTTON_START_STOP 15
#define BUTTON_RESET 16

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);

  // Generate unique device ID
  deviceID = String((uint32_t)ESP.getEfuseMac(), HEX);

  // Define response topics with device ID
  registrationResponseTopic = "devices/registration/response/" + deviceID;
  registrationSuccessTopic = "devices/registration/response/" + deviceID + "/regSuccess";

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");

  // Initialize serial for GPS
  ss.begin(GPSBaud);

  // Initialize buttons
  pinMode(BUTTON_START_STOP, INPUT_PULLUP);
  pinMode(BUTTON_RESET, INPUT_PULLUP);

  // Initialize MQTT
  client.setServer(mqtt_server, mqttPort);
  client.setCallback(mqttCallback);

  // Initialize time
  configTime(0, 0, "pool.ntp.org"); // Set NTP server

  // Attempt to connect to WiFi and check registration
  wifiConnectAndCheckRegistration();
}

void loop() {
  if (digitalRead(BUTTON_START_STOP) == LOW) {
    delay(200); // debounce
    if (confirmationMode) {
      resetData();
      confirmationMode = false;
      runFinished = false;
      updateLCD();
    } else if (runFinished) {
      wifiConnectAndSend();
      runFinished = false;
      updateLCD();
    } else if (isLogging) {
      isLogging = false;
      runFinished = true;
      updateLCD();
    } else {
      isLogging = !isLogging;
      startTime = millis();
      updateLCD();
    }
  }

  if (digitalRead(BUTTON_RESET) == LOW) {
    delay(200); // debounce
    if (runFinished) {
      confirmationMode = true;
      updateLCD();
    } else {
      resetData();
      updateLCD();
    }
  }

  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated() && isLogging) {
      logDataIfInterval();
    }
  }

  if (wifiConnected && client.connected()) {
    client.loop();
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (confirmationMode) {
    lcd.print("Clear data?");
    lcd.setCursor(0, 1);
    lcd.print("Yes: Start No: Reset");
  } else if (runFinished) {
    unsigned long runTime = millis() - startTime;
    lcd.print("Run time:");
    lcd.setCursor(0, 1);
    lcd.print(runTime / 60000); // minutes
    lcd.print(".");
    lcd.print((runTime % 60000) / 1000); // seconds
    lcd.print(" minutes");
  } else if (isLogging) {
    lcd.print("Logging...");
  } else {
    lcd.print("Start a new run!");
  }
}

void resetData() {
  gpsLog.clear();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Log deleted");
  lcd.setCursor(0, 1);
  lcd.print("Start a new run!");
  delay(2000);
}

void logDataIfInterval() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= logInterval) {
    lastLogTime = currentMillis;
    logData();
  }
}

void logData() {
  GpsData data;
  data.latitude = gps.location.lat();
  data.longitude = gps.location.lng();
  time(&data.timestamp);
  gpsLog.push_back(data);
  Serial.print("Logged: LAT=");
  Serial.print(data.latitude, 6);
  Serial.print(" LON=");
  Serial.println(data.longitude, 6);
}

void wifiConnectAndCheckRegistration() {
  WiFi.begin(ssid, password);
  lcd.setCursor(0, 1);
  lcd.print("Connecting WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  wifiConnected = true;
  if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
    checkRegistration();
    if (isRegistered) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Device Registered!");
      delay(2000);
      updateLCD();
    } else {
      displayDeviceID();
    }
  }
  WiFi.disconnect(true);
  wifiConnected = false;
}

void checkRegistration() {
  // Publish device ID for registration check
  String payload = "{\"DeviceId\":\"" + deviceID + "\"}";
  client.publish(registrationCheckTopic, payload.c_str());
  client.subscribe(registrationResponseTopic.c_str());
  client.subscribe(registrationSuccessTopic.c_str());

  // Wait for response
  unsigned long startTime = millis();
  while (!isRegistered && millis() - startTime < 10000) { // 10 seconds timeout
    client.loop();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) == registrationSuccessTopic) {
    isRegistered = true;
  }
}

void displayDeviceID() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Corsa ID:");
  lcd.setCursor(0, 1);
  lcd.print(deviceID);
}

void wifiConnectAndSend() {
  WiFi.begin(ssid, password);
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  wifiConnected = true;
  if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
    sendData();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Data sent!");
    delay(2000);
    updateLCD();
  }
  WiFi.disconnect(true);
  wifiConnected = false;
}

void sendData() {
  String payload = "{";
  payload += "\"DeviceId\":\"" + deviceID + "\",";
  payload += "\"gpsCordsList\":[";
  for (size_t i = 0; i < gpsLog.size(); ++i) {
    payload += "{";
    payload += "\"Latitude\":";
    payload += String(gpsLog[i].latitude, 6);
    payload += ",\"Longitude\":";
    payload += String(gpsLog[i].longitude, 6);
    payload += ",\"Timestamp\":";
    payload += String(gpsLog[i].timestamp);
    payload += "}";
    if (i < gpsLog.size() - 1) {
      payload += ",";
    }
  }
  payload += "]}";
  client.publish(dataTopic, payload.c_str());
  gpsLog.clear();
  Serial.println("Data sent to MQTT broker.");
}
