#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

//replace with your network credentials
#define WIFI_SSID "BS4 PTR-A0"
#define WIFI_PASSWORD "SAE_sucks"

#define MQTT_USER "wroomImAnESP"
#define MQTT_PASS "meepmeepdoesthecar!\"§123"

// Raspberry Pi Mosquitto MQTT Broker 192.168.178.65
#define MQTT_HOST IPAddress(192, 168, 25, 109)
#define MQTT_PORT 1883

//MQTT Topics
#define MQTT_PUB_TEMP_BME280 "esp32/bme280/temperature"
#define MQTT_PUB_HUM_BME280  "esp32/bme280/humidity"
#define MQTT_PUB_PRES_BME280 "esp32/bme280/pressure"

Adafruit_BME280 bme;
  
float temperature_BME280, humidity_BME280, pressure_BME280;   //variables for BME280

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   
const long interval = 10000;        

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case 16: // IP_EVENT_STA_GOT_IP
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); 
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
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

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
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


    // Publish an MQTT message on topic esp32/bme280/temperature
    uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_TEMP_BME280, 1, true, String(temperature_BME280).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP_BME280, packetIdPub3);
    Serial.printf("Message: %.2f \n", temperature_BME280);

    // Publish an MQTT message on topic esp32/bme280/humidity
    uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_HUM_BME280, 1, true, String(humidity_BME280).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_HUM_BME280, packetIdPub4);
    Serial.printf("Message: %.2f \n", humidity_BME280);
    
    // Publish an MQTT message on topic esp32/bme280/pressure
    uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_PRES_BME280, 1, true, String(pressure_BME280).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_PRES_BME280, packetIdPub5);
    Serial.printf("Message: %.2f \n", pressure_BME280);

  }
}