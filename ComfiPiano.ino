/*Pinout:
 * D0 -> TX, D1 -> RX of wifi module.
 * D4 - Humidifier Active LED (PWM Possible)
 * D5 - Dryer Active LED (PWM Possible)
 * D6 - Power LED (PWM Possible)
 * D7 - Data in from AM2302 / DHT22 Temperature Sensor
 * D14 / A0 - Analog input for Water Level Sensing
 * D22 - Dryer Relay
 * D23 - Humidifier Relay
 * D21 - FIll LED
 */

#include "DHT.h"
#include <EEPROM.h>

//This is Version 1 of the Code
#define VERSION 1

#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

#define HUMID_LED 4
#define DRYER_LED 5
#define POWER_LED 6
#define FILL_LED 21
#define WATER_SENSOR A0 //Analog Pin Number
#define DRYER_RELAY 22
#define HUMID_RELAY 23

#define LOWER_THRESHOLD_ADDRESS 0x0
#define UPPER_THRESHOLD_ADDRESS 0xF

#define WATER_THRESHOLD 400

String THINGSPEAK_WRITE_KEY = "XXXXXXXXXXXXX";

//global variables for serial comms
boolean stringComplete = false;
String inputString = "";

float lowerThreshold = 60.0;
float upperThreshold = 65.0;
float midThreshold = 62.5;

int loopCounter;

byte currentControlState = 0; // This represents the current operation state 0 - off, 1 - dryer, 2 - humider

float readFloat(unsigned int addr) {
  union {
    byte b[4];
    float f;
  } data;

  for(int i = 0; i < 4; i++) {
    data.b[i] = EEPROM.read(addr+i);
  }

  return data.f;
}


void writeFloat(unsigned int addr, float x) {
  union {
    byte b[4];
    float f;
    } data;

  data.f = x;
  for(int i = 0; i < 4; i++) {
    EEPROM.write(addr+i, data.b[i]);
  }
}

void readEEPROMValues() {
  // read thresholds from EEPROM
  lowerThreshold = readFloat(LOWER_THRESHOLD_ADDRESS);
  Serial.print("EEPROM Lower Threshold: ");
  Serial.print(lowerThreshold);
  upperThreshold = readFloat(UPPER_THRESHOLD_ADDRESS);
  Serial.print("\t Upper Threshold: ");
  Serial.println(upperThreshold);
  setMidThreshold();

}

void writeEEPROMValues(float lower, float upper) {
  // write thresholds from EEPROM
  writeFloat( LOWER_THRESHOLD_ADDRESS, lower );
  writeFloat( UPPER_THRESHOLD_ADDRESS, upper );

}

int readWaterLevel() {
  return analogRead(WATER_SENSOR);
}

void setHumidifier(bool state) {
  Serial.print("Setting Humidifier: ");
  if (state) {
    Serial.println("ON");
    digitalWrite(HUMID_RELAY,1);
    analogWrite(HUMID_LED,100);
  } else {
    Serial.println("OFF");
    digitalWrite(HUMID_RELAY,0);
    analogWrite(HUMID_LED,0);
  }
}

void setDryer(bool state) {
  Serial.print("Setting Dryer: ");
  if (state) {
    Serial.println("ON");
    digitalWrite(DRYER_RELAY,1);
    analogWrite(DRYER_LED,100);
  } else {
    Serial.println("OFF");
    digitalWrite(DRYER_RELAY,0);
    analogWrite(DRYER_LED,0);
  }
}

void doHumidControl(float curHumidity) {
  switch (currentControlState) {
    case 0:
      //nothing currently running
      if (curHumidity>upperThreshold) {
        setDryer(1);
        currentControlState = 1;
      } else if (curHumidity<lowerThreshold) {
        setHumidifier(1);
        currentControlState = 2;
      }
      break;
    case 1:
      //currently drying, check if complete
      if (curHumidity<midThreshold) {
        setDryer(0);
        currentControlState = 0;
      }
      break;
    case 2:
      //currently weting
      if (curHumidity>midThreshold) {
        setHumidifier(0);
        currentControlState = 0;
      }
      break;
  }

}

void setMidThreshold() {
  midThreshold = (upperThreshold+lowerThreshold)/2;
  Serial.print("Mid Threshold: ");
  Serial.println(midThreshold);
}

void setup() {

  //setup pins
  pinMode(POWER_LED, OUTPUT);
  pinMode(DRYER_RELAY, OUTPUT);
  pinMode(HUMID_RELAY, OUTPUT);
  pinMode(FILL_LED, OUTPUT);

  //start sensor library
  dht.begin();

  //turn on Power Indicator
  digitalWrite(POWER_LED,HIGH);

  //wait 30s for wifi module to stabilise before connecting to it
  for (int i=0; i <= 30; i++){
    digitalWrite(POWER_LED,LOW);
    delay(500);
    digitalWrite(POWER_LED,HIGH);
    delay(500);
   }

  //open serial port to host
  Serial.begin( 9600 );

  //open serial port to USR Wifi Module
  Serial1.begin( 9600 );

  //setup wifi module for AT commands
  Serial1.print("+++");
  delay(500);
  Serial1.print("a");

  //setup thingspeak IP address for HTTP packets
  Serial1.println("AT+HTTPURL=184.106.153.149,80");
  delay(200);
  Serial1.println("AT+HTTPTP=POST");
  delay(200);
  Serial1.println("AT+HTTPPH=/update"); //protocol header path

  //readEEPROMValues();

  //ensure IO state is correct for other pins
  analogWrite(HUMID_LED, 0);
  analogWrite(DRYER_LED, 0);
  digitalWrite(DRYER_RELAY,LOW);
  digitalWrite(HUMID_RELAY,LOW);

  delay(50);
  //blink Power LED to confirm bootup
  digitalWrite(POWER_LED,LOW);
  delay(150);
  digitalWrite(POWER_LED,HIGH);

  loopCounter = 100000;

  setMidThreshold();

}

void loop() {
  loopCounter++;

  serialEvent(); //process any serial data

  if (stringComplete) { //if a complete new line has been received on the serial port.
    String inCommand =  inputString.substring(0,1);//inputString first character
    char inCommandChar[2];
    inCommand.toCharArray(inCommandChar,2);
    Serial.println(String(int(inCommandChar[0])));
    String inData = inputString.substring(1);//rest of inputString with new line character
    int datLength = inData.length();
    inData = inData.substring(0,datLength-1); //remove last character
    Serial.println(inData);

    switch (inCommandChar[0]) {
      case 'U':
        //set upperThreshold
        upperThreshold = inData.toFloat();
        setMidThreshold();
        Serial.print("U:");
        Serial.print(upperThreshold);
        Serial.println("OK");
        break;
      case 'L':
        //set lowerThreshold
        lowerThreshold = inData.toFloat();
        setMidThreshold();
        Serial.print("L:");
        Serial.print(lowerThreshold);
        Serial.println("OK");
        break;
      case 'C':
        //currentThresholds
        Serial.print("L:");
        Serial.print(lowerThreshold);
        Serial.print("M:");
        Serial.print(midThreshold);
        Serial.print("U:");
        Serial.print(upperThreshold);
        Serial.println("OK");
        break;
      case 'E':
        writeEEPROMValues(lowerThreshold,upperThreshold);  //save to EEPROM
        Serial.println("E:OK");
        break;
      case 'F':
        readEEPROMValues();
        Serial.println("F:OK");
        break;
      case 'G':
        //get current Temperature and Humidity
        Serial.print("G:OKT:");
        Serial.print(dht.readTemperature());
        Serial.print("H:");
        Serial.println(dht.readHumidity());
        break;
      case 'R':
        //reset wifi config using AT Command on Serial2
        Serial.println("R:OK");
        Serial1.print("AT+RELD\r");
        break;
      default:
        Serial.println("???");
    }

    // clear the string:
    inputString = "";
    stringComplete = false;
  } //end of handling string if statement

  //check water level
  int waterLevel = readWaterLevel();

  //set water fill LED
  if (waterLevel>WATER_THRESHOLD) {
    digitalWrite(FILL_LED,HIGH);
  } else {
    digitalWrite(FILL_LED,LOW);
  }


  if (loopCounter>=30000) {  //wait for approximately 30 seconds
    loopCounter = 0;

    // read temperature sensor
    float currentTemperature = dht.readTemperature();
    float currentHumidity  = dht.readHumidity();

    // do humidity control
    doHumidControl(currentHumidity);

    // report state to internets
    updateSystemState(currentTemperature, currentHumidity, waterLevel, currentControlState);

  }

  // wait 1ms between iterations
  delay(1);

}

void updateSystemState(float temperature, float humidity, int wLevel, int controlState) {
  Serial.print("#");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print(wLevel);
  Serial.print(",");
  Serial.print(controlState);
  Serial.println("$");

  String dryerState;
  String humidifierState;

  if (controlState==1) {
    dryerState = "1";
  } else {
    dryerState = "0";
  }

  if (controlState==2) {
    humidifierState="1";
  } else {
    humidifierState="0";
  }
  //publish state to web via Serial2 Wifi Bridge
  String publishString = "AT+HTTPDT=";
  publishString += "key="+THINGSPEAK_WRITE_KEY;
  publishString += "&field1=" + String(temperature);
  publishString += "&field2=" + String(humidity);
  publishString += "&field3=" + String(wLevel);
  publishString += "&field4=" + dryerState;
  publishString += "&field5=" + humidifierState;
  publishString += "&field6=" + String(lowerThreshold);
  publishString += "&field7=" + String(upperThreshold);
  //Serial.println(publishString);
  Serial1.println(publishString);

}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
