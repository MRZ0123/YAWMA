#include <Wire.h>               //I2C
#include <Adafruit_SSD1306.h>   //Displaytreiber
#include <Adafruit_BME280.h>    //Sensortreiber
 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C     //I2C Adresse Display
#define SENSOR_ADDRESS 0x76     //I2C Adresse Sensor
#define SEALEVELPRESSURE_HPA (1013.25)
 
Adafruit_SSD1306 screen(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);   //Deklaration Display
Adafruit_BME280 bme;                                                       //Dekrlatation Sensor
 
void setup() {
  bme.begin(SENSOR_ADDRESS, &Wire);                   //setup Sensor
  screen.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS); //setup Display
  screen.clearDisplay();                              //
  screen.setTextSize(1);                              //
  screen.setTextColor(SSD1306_WHITE);                 //
  screen.display();                                   //
}
 
void loop() {
  screen.clearDisplay();
 
  screen.setCursor(5, 5);                                //Temperatur lesen & anzeigen
  screen.print("Temp: ");                                //
  screen.print(bme.readTemperature());                   //
  screen.print(" C");                                    //
 
  screen.setCursor(5,20);                                //Höhe lesen & anzeigen
  screen.print("Alt:  ");                                //
  screen.print(bme.readAltitude(SEALEVELPRESSURE_HPA));  //
  screen.print(" m");                                    //
 
  screen.setCursor(5,35);                                //Luftfeuchtigkeit lesen & anzeigen
  screen.print("Hum:  ");                                //
  screen.print(bme.readHumidity());                      //
  screen.print(" %");                                    //
 
  screen.setCursor(5,50);                                // Luftdruck lesen & anzeigen
  screen.print("Pres: ");                                //
  screen.print(bme.readPressure() / 100.0F);             //
  screen.print(" hPa");                                  //
 
  screen.display();
  delay(500);
}