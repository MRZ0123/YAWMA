#include "credentials.h"

#include <WiFi.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#include <Wire.h>
#include <Adafruit_BME280.h>

#define ASYNC_TCP_SSL_ENABLED true // false

#include <AsyncMQTT_ESP32.h>

#define SEALEVELPRESSURE_HPA (1013.25)

#if ASYNC_TCP_SSL_ENABLED
#define MQTT_SECURE true
const uint8_t MQTT_SERVER_FINGERPRINT[] = { 0xA9, 0xD9, 0xD0, 0xAF, 0x37, 0xBE, 0xDA, 0x51, 0x7A, 0x7F, 0x8B, 0xA6, 0xC4, 0xDC, 0xD0, 0x46, 0xA1, 0xEC, 0x9C, 0xAC };
const char* PubTopic = "test/lol";  // Topic to publish
#define MQTT_PORT 8883
#else
const char* PubTopic = "test/lol";  // Topic to publish
#define MQTT_PORT 1883
#endif

// Raspberry Pi Mosquitto MQTT Broker 192.168.178.65
#define MQTT_HOST IPAddress(192, 168, 25, 109)

//MQTT Topics
#define MQTT_PUB_TEMP_BME280 "esp32/bme280/temperature"
#define MQTT_PUB_HUM_BME280  "esp32/bme280/humidity"
#define MQTT_PUB_PRES_BME280 "esp32/bme280/pressure"
#define MQTT_PUB_ALT_BME280 "esp32/bme280/altitude"

Adafruit_BME280 bme;

float temperature_BME280, humidity_BME280, pressure_BME280, altitude_BME280;   //variables for BME280

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;
const long interval = 10000;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  mqttClient.connect();
}

bool was_connected = false;
bool is_connected = false;

void WiFiEvent(WiFiEvent_t event) {
  wl_status_t current_status = WiFi.status();
  Serial.printf("[WiFi-event] event: %d\n", event);
  Serial.printf("[WiFi-status] status: %d\n", current_status);
  switch(current_status){
    case WL_CONNECTED:
      is_connected=true;
      break;
    case WL_DISCONNECTED:
    case WL_NO_SSID_AVAIL:
    case WL_CONNECTION_LOST:
    case WL_CONNECT_FAILED:
      is_connected=false;
      break;
    default:
      break;
  }
  if (is_connected && !was_connected) {
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
  } else if(!is_connected && was_connected) {
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0);  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
  }
}


void printSeparationLine() {
  Serial.println("************************************************");
}

void onMqttConnect(bool sessionPresent) {
  Serial.print("Connected to MQTT broker: ");
  Serial.print(MQTT_HOST);
  Serial.print(", port: ");
  Serial.println(MQTT_PORT);
  Serial.printf("Topics: %s, %s, %s, %s\n", MQTT_PUB_TEMP_BME280, MQTT_PUB_HUM_BME280, MQTT_PUB_PRES_BME280, MQTT_PUB_ALT_BME280);

  printSeparationLine();
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  printSeparationLine();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  switch(reason){
    case AsyncMqttClientDisconnectReason::TCP_DISCONNECTED:
      Serial.println("Reason: TCP_DISCONNECTED");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_UNACCEPTABLE_PROTOCOL_VERSION:
      Serial.println("Reason: MQTT_UNACCEPTABLE_PROTOCOL_VERSION");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_IDENTIFIER_REJECTED:
      Serial.println("Reason: MQTT_IDENTIFIER_REJECTED");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_SERVER_UNAVAILABLE:
      Serial.println("Reason: MQTT_SERVER_UNAVAILABLE");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_MALFORMED_CREDENTIALS:
      Serial.println("Reason: MQTT_MALFORMED_CREDENTIALS");
      break;
    case AsyncMqttClientDisconnectReason::MQTT_NOT_AUTHORIZED:
      Serial.println("Reason: MQTT_NOT_AUTHORIZED");
      break;
    case AsyncMqttClientDisconnectReason::ESP8266_NOT_ENOUGH_SPACE:
      Serial.println("Reason: ESP8266_NOT_ENOUGH_SPACE");
      break;
    case AsyncMqttClientDisconnectReason::TLS_BAD_FINGERPRINT:
      Serial.println("Reason: TLS_BAD_FINGERPRINT");
      break;
    default:
      break;
  }

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(const uint16_t& packetId) {
  Serial.println("Publish acknowledged");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  if (!bme.begin(0x76, &Wire)) {
    Serial.println("Could not detect a BME280 sensor, Fix wiring connections!");
    while (1);
  }
  delay(1000);

  Serial.print("\nStarting YAWMA_mqtt on ");
  Serial.println(ARDUINO_BOARD);
  Serial.println(ASYNC_MQTT_ESP32_VERSION);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                    reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

#if ASYNC_TCP_SSL_ENABLED
  mqttClient.setSecure(MQTT_SECURE);
  if (MQTT_SECURE) {
    mqttClient.addServerFingerprint((const uint8_t*)MQTT_SERVER_FINGERPRINT);
  }
#endif

  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    //Read from BME280
    temperature_BME280 = bme.readTemperature();
    humidity_BME280 = bme.readHumidity();
    pressure_BME280 = bme.readPressure() / 100.0F;
    altitude_BME280 = bme.readAltitude(SEALEVELPRESSURE_HPA);


    // Publish an MQTT message on topic esp32/bme280/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP_BME280, 1, true, String(temperature_BME280).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP_BME280, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperature_BME280);

    // Publish an MQTT message on topic esp32/bme280/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM_BME280, 1, true, String(humidity_BME280).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_HUM_BME280, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity_BME280);
    
    // Publish an MQTT message on topic esp32/bme280/pressure
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES_BME280, 1, true, String(pressure_BME280).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_PRES_BME280, packetIdPub3);
    Serial.printf("Message: %.2f \n", pressure_BME280);

    // Publish an MQTT message on topic esp32/bme280/altitude
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_ALT_BME280, 1, true, String(altitude_BME280).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_ALT_BME280, packetIdPub4);
    Serial.printf("Message: %.2f \n", altitude_BME280);

  }
}