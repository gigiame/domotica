#include <Arduino.h>
#include "Wire.h"
#include "EEPROM.h"
#include "OneWire.h"
#include "DallasTemperature.h"  
void sendActuatorStatus(int actuatorId, byte buttonStatus);
void setup();
String getValue(String data, char separator, int index);
void sendTemperature(int deviceId, float temperature);
void processSerialRequests();
void processTempSensors();
void loop();
#line 1 "src/heatingcontrol.ino"
//#include "Wire.h"
//#include "EEPROM.h"
//#include "OneWire.h"
//#include "DallasTemperature.h"  
//#include "ApplicationMonitor.h"
//#include <avr/wdt.h>

// arduino MySensor Serial Protocol 2.x 
#define NODE_ID 2
#define SP2_SEP ";"
#define V_STATUS 2
#define V_TEMP 0
#define V_TRIPPED 16

// read temp sensors every 60 seconds
#define TEMP_READ_DELAY 60000 

// Imposta la comunicazione oneWire per comunicare
// con un dispositivo compatibile
OneWire oneWire10(10);
//OneWire oneWire9(11);
//OneWire oneWire10(12);

// Passaggio oneWire reference alla Dallas Temperature. 
DallasTemperature sensors10(&oneWire10);
//DallasTemperature sensors9(&oneWire9);
//DallasTemperature sensors10(&oneWire10);

unsigned long lastTempRead = 0;
bool controlLed = false;
    

void sendActuatorStatus(int actuatorId, byte buttonStatus) {
  Serial.print(NODE_ID);
  Serial.print(SP2_SEP);
  Serial.print(actuatorId);
  Serial.print(SP2_SEP);
  Serial.print(1); // set command
  Serial.print(SP2_SEP);
  Serial.print(0); // ack (normal message)
  Serial.print(SP2_SEP);
  Serial.print(V_STATUS); // Binary status (0 or 1)
  Serial.print(SP2_SEP);
  Serial.print(buttonStatus ? "ON" : "OFF"); // payload
  Serial.print("\n");
}

void setup() {  
  
  Serial.begin(9600); 
 
  // Start up the library
  //sensors8.begin();
  //sensors9.begin();
  sensors10.begin();
  
  for (int i=2; i<10; ++i) {
    pinMode(i, OUTPUT);  
    digitalWrite(i, HIGH); // high = closed
    sendActuatorStatus(i, LOW); // inverted logic
  }

}



String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void sendTemperature(int deviceId, float temperature) {
  Serial.print(NODE_ID);
  Serial.print(SP2_SEP);
  Serial.print(deviceId);
  Serial.print(SP2_SEP);
  Serial.print(1); // set command
  Serial.print(SP2_SEP);
  Serial.print(0); // ack (normal message)
  Serial.print(SP2_SEP);
  Serial.print(V_TEMP); 
  Serial.print(SP2_SEP);
  Serial.print(temperature);
  Serial.print("\n");
}

void processSerialRequests() {
  if (Serial.available())
  {
    String inputMQTT = Serial.readStringUntil('\n');
    int nodeId = getValue(inputMQTT, ';', 0).toInt();
    if (nodeId == NODE_ID) {
      int actuatorId = getValue(inputMQTT, ';', 1).toInt() + 2;
      String value3 = getValue(inputMQTT, ';', 2);
      String value4 = getValue(inputMQTT, ';', 3);
      String value5 = getValue(inputMQTT, ';', 4);
      String acutuatorStatusStr = getValue(inputMQTT, ';', 5);
      int acutuatorStatus = (acutuatorStatusStr == "ON") ? 0 : 1; // inverted logic (0 = open) 
      digitalWrite(actuatorId, acutuatorStatus);
    }
  }
}

void processTempSensors() {
  unsigned long sensorReadTime = millis();
  if (sensorReadTime - lastTempRead > TEMP_READ_DELAY) {
    sensors10.requestTemperatures();
    //sensors9.requestTemperatures();
    //sensors10.requestTemperatures();
    float temp1 = sensors10.getTempCByIndex(0);
    float temp2 = sensors10.getTempCByIndex(0);
    float temp3 = sensors10.getTempCByIndex(0);
    sendTemperature(16, temp2);
    sendTemperature(17, temp3);
    sendTemperature(18, temp1);
    lastTempRead = sensorReadTime;
  }
}

void loop() {
  processSerialRequests();
  //processTempSensors(); 
}
