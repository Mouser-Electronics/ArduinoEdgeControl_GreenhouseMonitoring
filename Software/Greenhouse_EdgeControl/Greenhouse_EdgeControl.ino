/*
  File:           Greenhouse_EdgeControl.ino
  Date Last Mod:  2AUG2021
  Description:    Proof-of-concept for a greenhouse monitoring device. This file contains
                  the code for the Arduino Edge Control carrier boardto be used in conjunction 
                  with a MKR1010 expansion board for Wi-Fi communications and interface with
                  the MKRENV environmental sensor shield.
*/


#include <Arduino_EdgeControl.h>
#include <openmvrpc.h>

openmv::rpc_scratch_buffer<256> scratch_buffer; // All RPC objects share this buffer.
openmv::rpc_i2c_master interface(0x12, 10000);

const uint32_t printInterval = 5000;
uint32_t printNow = 0;

#define DEBUG
#define FW_VERSION_ID "0.0.1-alpha"

#ifdef DEBUG
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINT(x) Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif


/*

*/
void setup()
{
  interface.begin();
  Serial.begin(115200);
  
  DEBUG_PRINT(F("Greenhouse Environmental Control System (GECS Version "));
  DEBUG_PRINT(FW_VERSION_ID);
  DEBUG_PRINTLN(F(") - Carrier Board initializing..."));
  
  EdgeControl.begin();
  Power.on(PWR_3V3);
  Power.on(PWR_VBAT);
  Power.on(PWR_MKR2);
  delay(5000);
  
  Wire.begin();
  delay(500);

  DEBUG_PRINT("I/O Expander initializazion ");
  if (!Expander.begin()) {
    DEBUG_PRINTLN(F("failed."));
    DEBUG_PRINTLN(F("Please, be sure to enable gated 3V3 and 5V power rails"));
    DEBUG_PRINTLN(F("via Power.on(PWR_3V3) and Power.on(PWR_VBAT)."));
  }
  DEBUG_PRINTLN(F("succeeded."));

  Expander.pinMode(EXP_FAULT_SOLAR_PANEL, INPUT);
  Expander.pinMode(EXP_FAULT_5V, INPUT);

  printNow = millis();
  DEBUG_PRINT(F("Carrier Board initialization complete.  System running.\n\n"));
}



/*

*/
void loop()
{
  if (millis() > printNow) {
    send_battery_voltage();
    printNow = millis() + printInterval;
  }
}



/*

*/
void send_battery_voltage()
{
  auto vbat = Power.getVBat();
  String str = "@BATTERY_VOLTAGE,";
  str += vbat;
  str += "#";
  char buffer[str.length() + 1] {};
  str.toCharArray(buffer, sizeof(buffer));
  interface.call("serial_print", buffer, sizeof(buffer));
}
