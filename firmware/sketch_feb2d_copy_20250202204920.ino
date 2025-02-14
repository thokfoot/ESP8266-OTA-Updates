#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <FirebaseESP8266.h>
#include <DHT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>  // OTA support

#define FIREBASE_HOST "https://humidity-and-temperature-54adc-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "m069MsJMajuGddygj9RRiDFvuVb0rBpJlDmMr3oS"

#define DHTPIN D4
#define DHTTYPE DHT22
#define LED_PIN 2             // Built-in LED (active LOW)
#define EEPROM_SIZE 512
#define RESET_FLAG_ADDR 0     // EEPROM address to store reset flag
#define RESET_BUTTON_PIN D3   // External Reset Button connected to GPIO D3

DHT dht(DHTPIN, DHTTYPE);
WiFiUDP udp;
NTPClient timeClient(udp, "pool.ntp.org", 19800, 60000);
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;
ESP8266WebServer server(80);

bool isConnected = false;  // Flag to track Wi-Fi connection status
unsigned long previousMillis = 0;  // Store last data transmission time
const long interval = 180000;      // Data transmission interval (3 minutes)
bool dataTransmissionActive = false;

void debug(const String &message) {
  Serial.println(message);
}

void resetWiFiSettings() {
  debug("Resetting Wi-Fi settings now...");
  WiFi.disconnect(true);
  WiFiManager wifiManager;

  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();

  digitalWrite(LED_PIN, LOW); // LED ON in AP mode
  wifiManager.startConfigPortal("ESP8266_AP");

  if (WiFi.status() == WL_CONNECTED) {
    debug("Wi-Fi connected successfully. Restarting device...");
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.write(RESET_FLAG_ADDR, 0);
    EEPROM.commit();
    EEPROM.end();
    ESP.restart();
  } else {
    debug("Device is in AP mode. No Wi-Fi connection.");
  }
}

void checkResetButton() {
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  static unsigned long buttonPressStart = 0;

  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    if (buttonPressStart == 0) {
      buttonPressStart = millis();
      debug("Reset button pressed.");
    }

    if (millis() - buttonPressStart >= 3000) {
      debug("Reset button held for 3 seconds. Starting Wi-Fi reset...");
      EEPROM.begin(EEPROM_SIZE);
      EEPROM.write(RESET_FLAG_ADDR, 1);
      EEPROM.commit();
      EEPROM.end();
      ESP.restart();
    }
  } else {
    buttonPressStart = 0;
  }
}

void indicateConnectionStatus() {
  if (WiFi.status() == WL_CONNECTED && !isConnected) {
    for (int i = 0; i < 2; i++) {
      digitalWrite(LED_PIN, LOW);
      delay(200);
      digitalWrite(LED_PIN, HIGH);
      delay(200);
    }
    debug("Device connected to Wi-Fi successfully.");
    isConnected = true;
  } 
}

String getCurrentDate() {
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime(&epochTime);

  int day = ptm->tm_mday;
  int month = ptm->tm_mon + 1;  // tm_mon is 0-based
  int year = ptm->tm_year + 1900;  // tm_year is years since 1900

  return String(day) + "-" + String(month) + "-" + String(year);
}

void sendDataToFirebase() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    debug("Error: Failed to read from DHT sensor.");
    dataTransmissionActive = false;
    return;
  }

  String deviceID = String(ESP.getChipId());
  String timestamp = timeClient.getFormattedTime();
  String date = getCurrentDate();
  String path = "/devices/" + deviceID + "/data/" + date + "_" + timestamp;

  FirebaseJson json;
  json.set("timestamp", timestamp);
  json.set("temperature", temperature);
  json.set("humidity", humidity);

  if (Firebase.setJSON(firebaseData, path, json)) {
    if (!dataTransmissionActive) {
      debug("Data transmission started.");
      dataTransmissionActive = true;
    }
  } else {
    if (dataTransmissionActive) {
      debug("Error: Data transmission stopped due to connection issue.");
      dataTransmissionActive = false;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow time for the Serial Monitor to initialize
  debug("Device is starting...");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  checkResetButton();

  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(RESET_FLAG_ADDR) == 1) {
    debug("Wi-Fi reset detected. Switching to AP mode...");
    resetWiFiSettings();
  } else {
    debug("Trying to connect to saved Wi-Fi...");
    WiFi.begin();
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      debug("Connecting to Wi-Fi...");
    }
    if (WiFi.status() != WL_CONNECTED) {
      debug("Failed to connect. Switching to AP mode...");
      resetWiFiSettings();
    }
  }
  EEPROM.end();

  // Print Wi-Fi connection details
  if (WiFi.status() == WL_CONNECTED) {
    debug("Wi-Fi connected. IP Address: " + WiFi.localIP().toString());
  } else {
    debug("Wi-Fi not connected.");
  }

  delay(1000); // Allow connection to stabilize

  // OTA Setup
  ArduinoOTA.onStart([]() {
    Serial.println("OTA update started.");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA update finished.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Ready. IP Address: " + WiFi.localIP().toString());

  dht.begin();
  timeClient.begin();
  timeClient.update();

  // Firebase Initialization
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  firebaseData.setBSSLBufferSize(512, 1024);
  Firebase.reconnectWiFi(true);

  debug("Device setup complete.");
}

void loop() {
  ArduinoOTA.handle();
  checkResetButton();
  timeClient.update();

  if (WiFi.status() == WL_CONNECTED) {
    indicateConnectionStatus();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      sendDataToFirebase();
    }
  } else {
    if (dataTransmissionActive) {
      debug("Warning: Wi-Fi disconnected. Data transmission stopped.");
      dataTransmissionActive = false;
    }
    digitalWrite(LED_PIN, LOW);
    isConnected = false;
  }
}
