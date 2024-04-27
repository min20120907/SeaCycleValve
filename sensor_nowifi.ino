#include "DFRobot_EnvironmentalSensor.h"
#include <SoftwareSerial.h>

#define MODESWITCH        /*UART:*/1 /*I2C: 0*/

#if MODESWITCH
  SoftwareSerial mySerial(/*rx =*/D6, /*tx =*/D7); // WeMos D1的RX和TX腳位
  DFRobot_EnvironmentalSensor environment(/*addr =*/SEN0500/SEN0501_DEFAULT_DEVICE_ADDRESS, /*s =*/&mySerial);
#else
DFRobot_EnvironmentalSensor environment(/*addr = */SEN0500/SEN0501_DEFAULT_DEVICE_ADDRESS, /*pWire = */&Wire);
#endif

#define echoPin D5 // Echo Pin
#define trigPin D4 // Trigger Pin
#define relayPin D3 // Relay Pin
long duration;
int distance;

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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relayPin, OUTPUT); // Set relay pin as output
}

void loop()
{
  //Print the data obtained from sensor
  Serial.println("-------------------------------");
  Serial.print("Temp: ");
  Serial.print(environment.getTemperature(TEMP_C));
  Serial.println(" ℃");
  Serial.print("Temp: ");
  Serial.print(environment.getTemperature(TEMP_F));
  Serial.println(" ℉");
  Serial.print("Humidity: ");
  Serial.print(environment.getHumidity());
  Serial.println(" %");
  Serial.print("Ultraviolet intensity: ");
  Serial.print(environment.getUltravioletIntensity());
  Serial.println(" mw/cm2");
  Serial.print("LuminousIntensity: ");
  Serial.print(environment.getLuminousIntensity());
  Serial.println(" lx");
  Serial.print("Atmospheric pressure: ");
  Serial.print(environment.getAtmospherePressure(HPA));
  Serial.println(" hpa");
  Serial.print("Altitude: ");
  Serial.print(environment.getElevation());
  Serial.println(" m");

  // HC-SR04 sensor logic
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance= duration*0.034/2;
  Serial.print("Water Level: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Relay logic
  float targetSpeed = 200.0; // Target water speed in L/min
  float maxSpeed = 400.0; // Max water speed of the pump in L/min
  float currentSpeed = getWaterSpeed(); // Assume we have a function to get the current water speed
  if (currentSpeed < targetSpeed) { // If current water speed is less than target
    digitalWrite(relayPin, HIGH); // Turn on the relay (pump)
    delay((targetSpeed / maxSpeed) * 1000); // Delay proportional to the target speed
    digitalWrite(relayPin, LOW); // Turn off the relay (pump)
  }
  
  Serial.println("-------------------------------");
  delay(500);
}
