#include "Wire.h"
#include "Adafruit_MCP23017.h"
#include "EEPROM.h"
#include "OneWire.h"
#include "DallasTemperature.h"  
#include "ApplicationMonitor.h"
#include <avr/wdt.h>

// arduino MySensor Serial Protocol 2.x 
#define NODE_ID 0
#define SP2_SEP ";"
#define V_STATUS 2
#define V_TEMP 0
#define V_TRIPPED 16

// read temp sensors every 60 seconds
#define TEMP_READ_DELAY 60000 

// Imposta la comunicazione oneWire per comunicare
// con un dispositivo compatibile
OneWire oneWire8(8);
OneWire oneWire9(9);

// Passaggio oneWire reference alla Dallas Temperature. 
DallasTemperature sensors8(&oneWire8);
DallasTemperature sensors9(&oneWire9);

// Basic pin reading and pullup test for the MCP23017 I/O expander
// public domain!

// Connect pin #12 of the expander to Analog 5 (i2c clock)
// Connect pin #13 of the expander to Analog 4 (i2c data)
// Connect pins #15, 16 and 17 of the expander to ground (address selection)
// Connect pin #9 of the expander to 5V (power)
// Connect pin #10 of the expander to ground (common ground)
// Connect pin #18 through a ~10kohm resistor to 5V (reset pin, active low)

// Input #0 is on pin 21 so connect a button or switch from there to ground

Adafruit_MCP23017 mcpInput1;
Adafruit_MCP23017 mcpRelay1;

Watchdog::CApplicationMonitor ApplicationMonitor;

/** the current address in the EEPROM (i.e. which byte we're going to write to next) **/
int eepromBaseAddr = 0;

const int MCP_SIZE = 16;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Variables will change:
//int ledState = LOW;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
byte initialValue = 0;
unsigned long lastTempRead = 0;
unsigned long buttonMatrix[MCP_SIZE][4];

const byte pir11 = 11;
const byte pir12 = 12;
const byte pir13 = 13;
int pir11Status = 0;    
int pir12Status = 0;     
int pir13Status = 0;      

// number of iterations completed. 
int g_nIterations = 0;   


void sendButtonStatus(int buttonId, byte buttonStatus) {
  Serial.print(NODE_ID);
  Serial.print(SP2_SEP);
  Serial.print(buttonId);
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

  ApplicationMonitor.Dump(Serial);
  ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_4s);
  
  // Start up the library
  sensors8.begin();
  sensors9.begin();
  
  mcpInput1.begin(0); // default address = 0
  mcpRelay1.begin(1); 

  pinMode(pir11, INPUT);
  pinMode(pir12, INPUT);
  pinMode(pir13, INPUT);
  
  for (int i=0; i<MCP_SIZE; ++i) {
    mcpInput1.pinMode(i, INPUT);
    mcpInput1.pullUp(i, HIGH);  

    mcpRelay1.pinMode(i, OUTPUT);
    mcpRelay1.pullUp(i, LOW);  

    // setting initial state from eeprom   
    initialValue = EEPROM.read(i);
    //initialValue = 0;
    mcpRelay1.digitalWrite(i, initialValue);
    buttonMatrix[i][3] = initialValue;
    sendButtonStatus(i, initialValue);
  }
}


void processPushButton(int button) {

  int reading = !mcpInput1.digitalRead(button);
  int buttonState = buttonMatrix[button][0];
  int lastButtonState = buttonMatrix[button][1];
  unsigned long debounceTime = buttonMatrix[button][2];
  int ledState = buttonMatrix[button][3];
  
  
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    unsigned long timer = millis();
    buttonMatrix[button][2] = timer;
  }

  if ((millis() - buttonMatrix[button][2]) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    
    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;
      buttonMatrix[button][0] = buttonState;
     
      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        ledState = !ledState;
        buttonMatrix[button][3] = ledState;
        EEPROM.write(button, (byte)ledState);
        sendButtonStatus(button, (byte)ledState);
	      //sendDebugMessage();
      }
    }
  }
  
  mcpRelay1.digitalWrite(button, ledState);

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  buttonMatrix[button][1] = reading; 
  
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

void sendDebugMessage() {
	Serial.print(NODE_ID);
  Serial.print(SP2_SEP);
  Serial.print(15);
  Serial.print(SP2_SEP);
  Serial.print(1); // set command
  Serial.print(SP2_SEP);
  Serial.print(0); // ack (normal message)
  Serial.print(SP2_SEP);
  Serial.print(V_STATUS); // Binary status (0 or 1)
  Serial.print(SP2_SEP);
  for (int i=0; i<MCP_SIZE; ++i) {
    Serial.print(i);
    Serial.print(".");
    Serial.print(buttonMatrix[i][0]);
    Serial.print(".");
    Serial.print(buttonMatrix[i][1]);
    Serial.print(".");
    Serial.print(buttonMatrix[i][2]);
    Serial.print(".");
    Serial.print(buttonMatrix[i][3]);
    Serial.print("|");
	}
  Serial.print("\n");
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

void sendPir(int deviceId, int value) {
  Serial.print(NODE_ID);
  Serial.print(SP2_SEP);
  Serial.print(deviceId);
  Serial.print(SP2_SEP);
  Serial.print(1); // set command
  Serial.print(SP2_SEP);
  Serial.print(0); // ack (normal message)
  Serial.print(SP2_SEP);
  Serial.print(V_TRIPPED); 
  Serial.print(SP2_SEP);
  Serial.print(value);
  Serial.print("\n");
}

void processSerialRequests() {
  if (Serial.available())
  {
    String inputMQTT = Serial.readStringUntil('\n');
    String value1 = getValue(inputMQTT, ';', 0);
    int buttonId = getValue(inputMQTT, ';', 1).toInt();
    String value3 = getValue(inputMQTT, ';', 2);
    String value4 = getValue(inputMQTT, ';', 3);
    String value5 = getValue(inputMQTT, ';', 4);
    String buttonStatusStr = getValue(inputMQTT, ';', 5);
    int buttonStatus = (buttonStatusStr == "ON") ? 1 : 0; 
    buttonMatrix[buttonId][3] = buttonStatus;
    EEPROM.write(buttonId, (byte)buttonStatus);

    if (buttonId=15) {
	    sendDebugMessage();
    }
  }
}

void processTempSensors() {
  unsigned long sensorReadTime = millis();
  if (sensorReadTime - lastTempRead > TEMP_READ_DELAY) {
    sensors8.requestTemperatures();
    sensors9.requestTemperatures();
    float temp8 = sensors8.getTempCByIndex(0);
    float temp9 = sensors9.getTempCByIndex(0);
    sendTemperature(16, temp8);
    sendTemperature(17, temp9);
    lastTempRead = sensorReadTime;
  }
}

void processPirSensors() {
  int pirStatus = digitalRead(pir11);
  if (pirStatus != pir11Status) {
    pir11Status = pirStatus;
    //sendPir(100+pir11, pir11Status);
  }
  pirStatus = digitalRead(pir12);
  if (pirStatus != pir12Status) {
    pir12Status = pirStatus;
    sendPir(100+pir12, pir12Status);

    buttonMatrix[13][3] = pir12Status;
    EEPROM.write(13, (byte)pir12Status);
    
  }
  pirStatus = digitalRead(pir13);
  if (pirStatus != pir13Status) {
    pir13Status = pirStatus;
    sendPir(100+pir13, pir13Status);
  }
}

void loop() {
  for (int i=0; i<MCP_SIZE; ++i) {
    processPushButton(i);
  }
  processSerialRequests();
  processTempSensors();
  processPirSensors();

  ApplicationMonitor.IAmAlive();
  ApplicationMonitor.SetData(g_nIterations++);
  
}
