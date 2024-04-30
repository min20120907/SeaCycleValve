#include "DFRobot_EnvironmentalSensor.h"
#include <SoftwareSerial.h>
#include <Ultrasonic.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#define MODESWITCH        /*UART:*/0 /*I2C: 0*/

#if MODESWITCH
  SoftwareSerial mySerial(/*rx =*/D6, /*tx =*/D7); // WeMos D1的RX和TX腳位
  DFRobot_EnvironmentalSensor environment(/*addr =*/SEN050X_DEFAULT_DEVICE_ADDRESS, /*s =*/&mySerial);
#else
DFRobot_EnvironmentalSensor environment(/*addr = */SEN050X_DEFAULT_DEVICE_ADDRESS, /*pWire = */&Wire);
#endif

#define echoPin D6 // Echo Pin
#define trigPin D7 // Trigger Pin
#define relayPin D3 // Relay Pin
Ultrasonic ultrasonic(D6, D7);

float temp; // Temperature
float humidity; // Humidity
float ultraviolet_intensity; // Ultraviolet intensity
float luminous_intensity; // Luminous intensity
float airpressure; // Atmospheric pressure
float elevation; // Altitude
float distance; // Water level (distance)
WiFiClient client;
// WiFi settings
const char* ssid = "ZenFone7 Pro_7128";
const char* password = "14ced07a3454";
// const char* ssid ="tku";
// const char* password = "";
void setup()
{
#if MODESWITCH
  //Init MCU communication serial port
  mySerial.begin(9600);
#endif
  Serial.begin(115200);
  while(environment.begin() != 0){
    Serial.println(" Sensor initialize failed!!");
    delay(1000);
  }
  Serial.println(" Sensor  initialize success!!");

  pinMode(relayPin, OUTPUT); // Set relay pin as output

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop()
{
  //Print the data obtained from sensor
  Serial.println("-------------------------------");
  Serial.print("Temp: ");
  float temp = environment.getTemperature(TEMP_C);
  Serial.print(temp);
  Serial.println(" ℃");
  // ... repeat for other sensor readings ...
  temp = environment.getTemperature(TEMP_C);
humidity = environment.getHumidity();
ultraviolet_intensity = environment.getUltravioletIntensity();
luminous_intensity = environment.getLuminousIntensity();
airpressure = environment.getAtmospherePressure(HPA);
elevation = environment.getElevation();
distance = ultrasonic.distanceRead();
if(distance==357) distance=0;

  // Relay logic
  float targetSpeed = 20.0; // Target water speed in L/min
  float maxSpeed = 400.0; // Max water speed of the pump in L/min
  digitalWrite(relayPin, HIGH); // Turn on the relay (pump)
  delay((targetSpeed / maxSpeed) * 1000); // Delay proportional to the target speed
  digitalWrite(relayPin, LOW); // Turn off the relay (pump)

  // Send data to server
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(client, "http://163.13.127.50:5000/insert_data_from_sensors");
    http.addHeader("user-agent", "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_11_6) AppleWebKit/537.36" \
                 "(KHTML, like Gecko) Chrome/55.0.2883.95 Safari/537.36");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    String httpRequestData = "username=min20120907&sensor_id=sensor1&realTemp=" + String(temp)
                        + "&humidity=" + String(humidity)
                        + "&Ultraviolet_intensity=" + String(ultraviolet_intensity)
                        + "&LuminousIntensity=" + String(luminous_intensity)
                        + "&airPressure=" + String(airpressure)
                        + "&Altitude=" + String(elevation)
                        + "&waterLevel=" + String(7-distance)
                        + "&water_Flow_Speed=" + String(targetSpeed);
    Serial.println(httpRequestData);
    int httpResponseCode = http.POST(httpRequestData);
    if (httpResponseCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  }
  else {
    Serial.println("WiFi Disconnected");
  }

  Serial.println("-------------------------------");
  delay(500);
}
