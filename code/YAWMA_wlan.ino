#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
 
#define SEALEVELPRESSURE_HPA (1013.25)
 
Adafruit_BME280 bme;
 
float temperature, humidity, pressure, altitude;
 
//Hier muss die jeweilige SSID und das Passwort des WLAN-Netzwerks eingegeben werden
const char* ssid = "BS4 PTR-A0";  //  SSID 
const char* password = "SAE_sucks";  //Passwort
 
WebServer server(80);             
 
void setup() {
  Serial.begin(115200);
  delay(100);
 
  bme.begin(0x76);   
 
  Serial.println("Connecting to ");
  Serial.println(ssid);
 
  //connect to your local wi-fi network
  WiFi.begin(ssid, password);
 
  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
  delay(1000);
  Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  Serial.print("Got IP: ");  Serial.println(WiFi.localIP());
 
  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);
 
  server.begin();
  Serial.println("HTTP server started");
 
}
void loop() {
  server.handleClient();
}
 
void handle_OnConnect() {
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  server.send(200, "text/html", SendHTML(temperature,humidity,pressure,altitude)); 
}
 
void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}
 
String SendHTML(float temperature,float humidity,float pressure,float altitude){
  String ptr = "<!doctypehtml><title>ESP32 Weather Station</title><meta content='width=device-width,initial-scale=1'name=viewport><link href='https://fonts.googleapis.com/css?family=Open+Sans:300,400,600'rel=stylesheet><style>html{font-family:'Open Sans',sans-serif;display:block;margin:0 auto;text-align:center;color:#444}body{margin:0}h1{margin:50px auto 30px}.side-by-side{display:table-cell;vertical-align:middle;position:relative}.text{font-weight:600;font-size:19px;width:200px}.reading{font-weight:300;font-size:50px;padding-right:25px}.temperature .reading{color:#111}.humidity .reading{color:#3b97d3}.pressure .reading{color:#26b99a;letter-spacing:0}.altitude .reading{color:#955ba5}.superscript{font-size:17px;font-weight:600;position:absolute;top:10px}.data{padding:10px}.container{display:table;margin:0 auto}.icon{width:65px}::after{font-weight:300;font-size:50px}.warm::after{content:'\\2600\\FE0F'}.kalt::after{content:'\\2744\\FE0F'}.wet::after{content:'\\1F4A7'}.moist::after{content:'\\1F966'}.dry::after{content:'\\1FAA8'}.highp::after{content:'\\25AA\\FE0F'}.medp::after{content:'\\25FE'}.lowp::after{content:'\\2B1B'}.high::after{content:'\\2708\\FE0F'}.low::after{content:'\\1F3E0'}</style><h1>YAWMA</h1><div class=container><div class='data temperature'><div class='side-by-side text'>Temperature</div><div class='side-by-side reading'>";
  ptr += (int)temperature;
  ptr +="<span class=superscript>&#xB0;C</span></div></div><div class='data humidity'><div class='side-by-side text'>Humidity</div><div class='side-by-side reading'>";
  ptr += (int)humidity;
  ptr +="<span class=superscript>%</span></div></div><div class='data pressure'><div class='side-by-side text'>Pressure</div><div class='side-by-side reading'>";
  ptr += (int)pressure;
  ptr +="<span class=superscript>hPa</span></div></div><div class='data altitude'><div class='side-by-side text'>Altitude</div><div class='side-by-side reading'>";
  ptr += (int)altitude;
  ptr +="<span class=superscript>m</span></div></div></div><script>let readings=document.querySelectorAll('.reading'); let elements=document.querySelectorAll('.side-by-side.text'); if (readings[0].innerText.split('\\n')[0] < 15){elements[0].classList.remove('warm'); elements[0].classList.add('kalt');}else{elements[0].classList.remove('kalt'); elements[0].classList.add('warm');}if (readings[1].innerText.split('\\n')[0] < 40){elements[1].classList.remove('moist', 'wet'); elements[1].classList.add('dry');}else if (readings[1].innerText.split('\\n')[0] > 60){elements[1].classList.remove('dry', 'moist'); elements[1].classList.add('wet');}else{elements[1].classList.remove('dry', 'wet'); elements[1].classList.add('moist');}if (readings[2].innerText.split('\\n')[0] < 950){elements[2].classList.remove('highp', 'medp'); elements[2].classList.add('lowp');}else if (readings[2].innerText.split('\\n')[0] > 1000){elements[2].classList.remove('lowp', 'medp'); elements[2].classList.add('highp');}else{elements[2].classList.remove('lowp', 'highp'); elements[2].classList.add('medp');}if (readings[3].innerText.split('\\n')[0] < 430){elements[3].classList.remove('high'); elements[3].classList.add('low');}else{elements[3].classList.remove('low'); elements[3].classList.add('high');}</script>";
  return ptr;
}
