/*
  File:           Greenhouse_DEV_FW.ino
  Date Last Mod:  2AUG2021
  Description:    Proof-of-concept for a greenhouse monitoring device to be installed at Snyder Farm in Everett, PA

  Top Level Requirements:

  Sensor Inputs

    - [x]  Temperature
    - [x]  Humidity
    - [ ]  Duration of sunlight
        - [x]  Measure illuminance (FC)
    - [ ]  Soil pH  (TBD)
    - [ ]  Hydroponics water quality  (TBD)
    - [ ]  propane heater particulate matter (CO2 or TVOC) (???)
    - [ ]  Monitor position of LOUVERS
    - [ ]  Battery status
    - [ ]  Hydrostatic water level sensor

  Actuator Outputs

    - [ ]  Open/close LOUVERS
    - [ ]  Relay to turn FAN on/off
    - [ ]  Relay to turn HEATER on/off
    - [x]  Onboard LED

  Communications

    - [x]  Send telemetry to cloud
    - [x]  Send manual control command to device
    - [x]  WiFi

  UI / UX

    - [ ]  Mobile App
    - [ ]  Web Browser

*/

#include "arduino_secrets.h"
#include "thing_properties.h"
#include "system_settings.h"
#include <Arduino_MKRENV.h>
#include <WiFiNINA.h>
#include <utility/wifi_drv.h>
#include <openmvrpc.h>

#define DEBUG
#define PRINT_TELEMETRY_SERIAL

#define FW_VERSION_ID "0.0.1-alpha"

#ifdef DEBUG
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif


int SENSOR_READING_COUNTER = 0;
bool ENOUGH_SENSOR_READINGS_FOR_AVG = false;

float avgTemperature = 0.0;
float avgHumidity = 0.0;
float avgPressure = 0.0;
float avgIlluminance = 0.0;


openmv::rpc_scratch_buffer<256> scratch_buffer;
openmv::rpc_callback_buffer<8> callback_buffer;
openmv::rpc_software_serial_uart_slave interface(2, 3, 19200);



/*

*/
void setup() {

  interface.register_callback(F("serial_print"), serial_print_example);
  // Startup the RPC interface and a debug channel.
  interface.begin();
  
  /* Initialize serial and wait up to 5 seconds for port to open */
  Serial.begin(115200);
  for (unsigned long const serialBeginTime = millis(); !Serial && (millis() - serialBeginTime > 8000);) {}

  if (!ENV.begin()) {
    Serial.println("Failed to initialize MKR ENV shield!");
    while (1)
      ;
  }

  /* Configure LED pin as an output */
  WiFiDrv::pinMode(25, OUTPUT);  //RED LED
  WiFiDrv::pinMode(26, OUTPUT);  //GREEN LED
  WiFiDrv::pinMode(27, OUTPUT);  //BLUE LED
  WiFiDrv::analogWrite(25, 0);   //RED
  WiFiDrv::analogWrite(26, 0);   //GREEN
  WiFiDrv::analogWrite(27, 0);   //BLUE

  Serial.print(F("Snyder Greenhouse Environmental Control System (SG:ECS Version "));
  Serial.print(FW_VERSION_ID);
  Serial.println(F(") initializing..."));

  /* This function takes care of connecting your sketch variables to the ArduinoIoTCloud object */
  initProperties();

  /* Initialize Arduino IoT Cloud library */
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(DBG_INFO);
  ArduinoCloud.printDebugInfo();
  delay(1000);
  Serial.println(F("Initialization complete.  System running...\n\n"));
}



/*

*/
void loop() {
  interface.loop();
  ArduinoCloud.update();
  readSensors();
  recordSensorData();
  calculateAverageSensorReadings();

#ifdef PRINT_TELEMETRY_SERIAL
  printSensorSerial();
  if (ENOUGH_SENSOR_READINGS_FOR_AVG == true) {
    printAvgSensorSerial();
  }
#endif

  if (MANUAL_CONTROL_ON) {     //Manual Control Mode is ON
    DEBUG_PRINTLN("MANUAL CONTROL is ENABLED.  AUTOMATIC MODE is OFF.");
  }
  else {    // Automatic Mode is ON
    DEBUG_PRINTLN("MANUAL CONTROL is DISABLED.  AUTOMATIC MODE is ON.");
    engageAutomaticControlFunctions();
  }

  delay(500);
}



/*

*/
void readSensors() {
  temperature = ENV.readTemperature(FAHRENHEIT);
  humidity = ENV.readHumidity();
  pressure = ENV.readPressure(PSI);
  illuminance = ENV.readIlluminance(FOOTCANDLE);
  uva = ENV.readUVA();
  uvb = ENV.readUVB();
  uvIndex = ENV.readUVIndex();
}



/*

*/
void recordSensorData() {

  if (SENSOR_READING_COUNTER > 9) {
    SENSOR_READING_COUNTER = 0;
    ENOUGH_SENSOR_READINGS_FOR_AVG = true;
  }

  TEMP_LAST_X_READINGS[SENSOR_READING_COUNTER] = temperature;
  HUMIDITY_LAST_X_READINGS[SENSOR_READING_COUNTER] = humidity;
  PRESSURE_LAST_X_READINGS[SENSOR_READING_COUNTER] = pressure;
  ILLUMINANCE_LAST_X_READINGS[SENSOR_READING_COUNTER] = illuminance;
  SENSOR_READING_COUNTER++;
}



/*

*/
void calculateAverageSensorReadings() {
  if (ENOUGH_SENSOR_READINGS_FOR_AVG == true) {
    for (int ctr = 0; ctr < NUM_SENSOR_READINGS_TO_STORE; ctr++) {
      avgTemperature += TEMP_LAST_X_READINGS[ctr];
      avgHumidity += HUMIDITY_LAST_X_READINGS[ctr];
      avgPressure += PRESSURE_LAST_X_READINGS[ctr];
      avgIlluminance += ILLUMINANCE_LAST_X_READINGS[ctr];
    }

    avgTemperature /= NUM_SENSOR_READINGS_TO_STORE;
    avgHumidity /= NUM_SENSOR_READINGS_TO_STORE;
    avgPressure /= NUM_SENSOR_READINGS_TO_STORE;
    avgIlluminance /= NUM_SENSOR_READINGS_TO_STORE;
  }
}


/*

*/
void printSensorSerial() {
  Serial.print(F("Temperature (F): "));
  Serial.println(temperature);
  Serial.print(F("Humidity(%): "));
  Serial.println(humidity);
  Serial.print(F("Pressure (PSI): "));
  Serial.println(pressure);
  Serial.print(F("Illuminance (FC): "));
  Serial.println(illuminance);
  Serial.print(F("UVA: "));
  Serial.println(uva);
  Serial.print(F("UVB: "));
  Serial.println(uvb);
  Serial.print(F("UV Index: "));
  Serial.println(uvIndex);
  Serial.print("Manual Control ON? ");
  if (MANUAL_CONTROL_ON) {
    Serial.println(F("YES"));
  }
  else {
    Serial.println(F("NO"));
  }

  Serial.print(F("Fan On? "));
  Serial.println(FAN_ON);
  Serial.print(F("Heater On? "));
  Serial.println(HEATER_ON);
  Serial.print(F("Louver Open? "));
  Serial.println(LOUVER_OPEN);
  Serial.println();
}



/*

*/
void printAvgSensorSerial() {
  Serial.print(F("Avg. Temperature (F): "));
  Serial.println(avgTemperature);
  Serial.print(F("Avg. Humidity(%): "));
  Serial.println(avgHumidity);
  Serial.print(F("Avg. Pressure (PSI): "));
  Serial.println(avgPressure);
  Serial.print(F("Avg. Illuminance (FC): "));
  Serial.println(avgIlluminance);
  Serial.println();
}



/*

*/
void onManualControlSwitchChange() {
  DEBUG_PRINT(F("MANUAL CONTROL set to "));
  if (MANUAL_CONTROL_SWITCH_ON == false) {
    WiFiDrv::analogWrite(25, 0);  //RED
    WiFiDrv::analogWrite(26, 0);  //GREEN
    WiFiDrv::analogWrite(27, 0);  //BLUE
    DEBUG_PRINTLN(F("OFF."));
    MANUAL_CONTROL_ON = false;
  } else {
    WiFiDrv::analogWrite(25, 0);    //RED
    WiFiDrv::analogWrite(26, 255);  //GREEN
    WiFiDrv::analogWrite(27, 0);    //BLUE
    DEBUG_PRINTLN(F("ON."));
    MANUAL_CONTROL_ON = true;
  }
}



/*

*/
void onFanSwitchChange() {
  if (MANUAL_CONTROL_ON) {
    if (FAN_SWITCH_ON) {
      FAN_ON = true;
    }
    else {
      FAN_ON = false;
    }
    DEBUG_PRINT("FAN ON: ");
    DEBUG_PRINTLN(FAN_ON);
  }
}



/*

*/
void onHeaterSwitchChange() {
  if (MANUAL_CONTROL_ON) {
    if (HEATER_SWITCH_ON) {
      HEATER_ON = true;
    }
    else {
      HEATER_ON = false;
    }

    DEBUG_PRINT("HEATER ON: ");
    DEBUG_PRINTLN(HEATER_ON);
  }
}



/*

*/
void onLouverSwitchChange() {
  if (MANUAL_CONTROL_ON) {
    if (LOUVER_SWITCH_ON) {
      LOUVER_OPEN = true;
    }
    else {
      LOUVER_OPEN = false;
    }
    DEBUG_PRINT("LOUVER OPEN: ");
    DEBUG_PRINTLN(LOUVER_OPEN);

  }
}



/*

*/
void engageAutomaticControlFunctions() {
  HEATER_ON = false;
  LOUVER_OPEN = false;
  FAN_ON = false;
}



/*

*/
void serial_print_example(void *in_data, size_t in_data_len) {
  // Create the string on the stack (extra byte for the null terminator).
  char buff[in_data_len + 1]; memset(buff, 0, in_data_len + 1);

  // Copy what we received into our data type container.
  memcpy(buff, in_data, in_data_len);

  // Use it now.
  DEBUG_PRINTLN(buff);
}
