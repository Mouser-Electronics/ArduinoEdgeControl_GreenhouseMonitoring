#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

#if defined(BOARD_HAS_WIFI)
#elif defined(BOARD_HAS_GSM)
#elif defined(BOARD_HAS_LORA)
#elif defined(BOARD_HAS_NB)
#else
#error "Arduino IoT Cloud currently only supports MKR1000, MKR WiFi 1010, MKR WAN 1300/1310, MKR NB 1500 and MKR GSM 1400"
#endif

#define THING_ID "e4cf7b40-381d-483a-840d-c2db6290f6ce"
#define BOARD_ID "e6eb1992-abe2-46a5-a17f-94ab742fe25c"

void onManualControlSwitchChange();
void onFanSwitchChange();
void onHeaterSwitchChange();
void onLouverSwitchChange();

bool MANUAL_CONTROL_ON = false;
bool MANUAL_CONTROL_SWITCH_ON = false;
bool FAN_ON = false;
bool FAN_SWITCH_ON = false;
bool HEATER_ON = false;
bool HEATER_SWITCH_ON = false;
bool LOUVER_OPEN = false;
bool LOUVER_SWITCH_ON = false;
float temperature = 0.0;
float humidity = 0.0;
float pressure = 0.0;
float illuminance = 0.0;
float uva = 0.0;
float uvb = 0.0;
float uvIndex = 0.0;

void initProperties() {
#if defined(BOARD_ESP8266)
  ArduinoCloud.setBoardId(BOARD_ID);
  ArduinoCloud.setSecretDeviceKey(SECRET_DEVICE_KEY);
#endif
  ArduinoCloud.setThingId(THING_ID);
#if defined(BOARD_HAS_WIFI) || defined(BOARD_HAS_GSM) || defined(BOARD_HAS_NB) || defined(BOARD_HAS_LORA)
  ArduinoCloud.addProperty(MANUAL_CONTROL_SWITCH_ON, Permission::Write).onUpdate(onManualControlSwitchChange);
  ArduinoCloud.addProperty(FAN_SWITCH_ON, Permission::Write).onUpdate(onFanSwitchChange);
  ArduinoCloud.addProperty(HEATER_SWITCH_ON, Permission::Write).onUpdate(onHeaterSwitchChange);
  ArduinoCloud.addProperty(LOUVER_SWITCH_ON, Permission::Write).onUpdate(onLouverSwitchChange);
  ArduinoCloud.addProperty(temperature, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(humidity, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(pressure, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(illuminance, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(uva, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(uvb, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(uvIndex, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(HEATER_ON, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(FAN_ON, Permission::Read).publishOnChange(10);
  ArduinoCloud.addProperty(LOUVER_OPEN, Permission::Read).publishOnChange(10);
#endif
}

#if defined(BOARD_HAS_WIFI)
WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_SSID, SECRET_PASS);
#elif defined(BOARD_HAS_GSM)
GSMConnectionHandler ArduinoIoTPreferredConnection(SECRET_PIN, SECRET_APN, SECRET_LOGIN, SECRET_PASS);
#elif defined(BOARD_HAS_LORA)
LoRaConnectionHandler ArduinoIoTPreferredConnection(SECRET_APP_EUI, SECRET_APP_KEY, _lora_band::EU868, _lora_class::CLASS_A);
#elif defined(BOARD_HAS_NB)
NBConnectionHandler ArduinoIoTPreferredConnection(SECRET_PIN, SECRET_APN, SECRET_LOGIN, SECRET_PASS);
#endif
