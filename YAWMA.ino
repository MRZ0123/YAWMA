//#include "DHT.h"
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>

//replace with your network credentials
#define WIFI_SSID "BS4 PTR-A0"
#define WIFI_PASSWORD "SAE_sucks"

// Raspberry Pi Mosquitto MQTT Broker 192.168.178.65
#define MQTT_HOST IPAddress(192, 168, 25, 109)
#define MQTT_PORT 1883

//MQTT Topics
//#define MQTT_PUB_TEMP_DHT "esp32/dht/temperature"
//#define MQTT_PUB_HUM_DHT  "esp32/dht/humidity"

#define MQTT_PUB_TEMP_BME280 "esp32/bme280/temperature"
#define MQTT_PUB_HUM_BME280  "esp32/bme280/humidity"
#define MQTT_PUB_PRES_BME280 "esp32/bme280/pressure"

//#define MQTT_PUB_TEMP_C "esp32/ds18b20/temperatureC"
//#define MQTT_PUB_TEMP_F "esp32/ds18b20/temperatureF"

//#define DHTPIN 15  
//#define DHTTYPE DHT22 
//DHT dht(DHTPIN, DHTTYPE);

Adafruit_BME280 bme;

const int SensorDataPin = 4;   
  
//OneWire oneWire(SensorDataPin);
//DallasTemperature sensors(&oneWire);

//float temperature_DHT, humidity_DHT; //variables for DHT
float temperature_BME280, humidity_BME280, pressure_BME280;   //variables for BME280
//float temperature_Celsius, temperature_Fahrenheit;  //variables for DS18B20

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

 // dht.begin();
 // delay(1000);

  if (!bme.begin(0x76, &Wire)) {
    Serial.println("Could not detect a BME280 sensor, Fix wiring connections!");
    while (1);
  }
  delay(1000);

  //sensors.begin();
  //delay(1000);
  
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

    /*
    //Read from DHT
    humidity_DHT = dht.readHumidity();
    temperature_DHT = dht.readTemperature();
    
    if (isnan(temperature_DHT) || isnan(humidity_DHT)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      return;
    }
    //delay(3000);
    */

    //Read from BME280
    temperature_BME280 = bme.readTemperature();
    humidity_BME280 = bme.readHumidity();
    pressure_BME280 = bme.readPressure() / 100.0F;
    //delay(3000);

    /*
    //Read from DS18B20
    sensors.requestTemperatures(); 
    temperature_Celsius = sensors.getTempCByIndex(0);
    temperature_Fahrenheit = sensors.getTempFByIndex(0);
    //delay(3000);
    */

    /*
    // Publish an MQTT message on topic esp32/dht/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP_DHT, 1, true, String(temperature_DHT).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP_DHT, packetIdPub1);
    Serial.printf("Message: %.2f \n", temperature_DHT);

    // Publish an MQTT message on topic esp32/dht/humidity
    uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM_DHT, 1, true, String(humidity_DHT).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM_DHT, packetIdPub2);
    Serial.printf("Message: %.2f \n", humidity_DHT);
    */

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

    /*
    // Publish an MQTT message on topic esp32/ds18b20/temperatureC
    uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_TEMP_C, 1, true, String(temperature_Celsius).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP_C, packetIdPub6);
    Serial.printf("Message: %.2f \n", temperature_Celsius);

    // Publish an MQTT message on topic esp32/ds18b20/temperatureF
    uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_TEMP_F, 1, true, String(temperature_Fahrenheit).c_str());                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_TEMP_F, packetIdPub7);
    Serial.printf("Message: %.2f \n", temperature_Fahrenheit);
    */
  }
}