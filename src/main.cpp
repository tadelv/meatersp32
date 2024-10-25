#include "AverageThermistor.h"
#include "NTC_Thermistor.h"
#include <Arduino.h>
#include <MAX6675.h>
#ifndef STM32
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <NimBLEServer.h>
#define DEBUG_SERIAL Serial
#else
#define DEBUG_SERIAL Serial1
#endif

MAX6675 thermocouple(MCS_PIN, MDO_PIN, MCLK_PIN);

Thermistor *thermistor;
#ifndef STM32
static NimBLEServer *pServer;
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer) {
    DEBUG_SERIAL.println("Client connected");
    DEBUG_SERIAL.println("Multi-connect support: start advertising");
    NimBLEDevice::startAdvertising();
  };
  /** Alternative onConnect() method to extract details of the connection.
   *  See: src/ble_gap.h for the details of the ble_gap_conn_desc struct.
   */
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
    DEBUG_SERIAL.print("Client address: ");
    DEBUG_SERIAL.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    /** We can use the connection handle here to ask for different connection
     * parameters. Args: connection handle, min connection interval, max
     * connection interval latency, supervision timeout. Units; Min/Max
     * Intervals: 1.25 millisecond increments. Latency: number of intervals
     * allowed to skip. Timeout: 10 millisecond increments, try for 5x interval
     * time for best results.
     */
    pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
  };
  void onDisconnect(NimBLEServer *pServer) {
    DEBUG_SERIAL.println("Client disconnected - start advertising");
    NimBLEDevice::startAdvertising();
  };
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc) {
    DEBUG_SERIAL.printf("MTU updated: %u for connection ID: %u\n", MTU,
                        desc->conn_handle);
  };
};

void initBT() {

  NimBLEDevice::init("MEATER ESP32");
#ifdef ESP_PLATFORM
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
  NimBLEDevice::setPower(9); /** +9db */
#endif

  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService *tempService =
      pServer->createService("a75cc7fc-c956-488f-ac2a-2dbc08b63a04");
  NimBLECharacteristic *temperatureCharacteristic =
      tempService->createCharacteristic("7edda774-045e-4bbf-909b-45d1991a2876",
                                        NIMBLE_PROPERTY::READ |
                                            NIMBLE_PROPERTY::NOTIFY);

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();

  temperatureCharacteristic->setValue(0);

  tempService->start();

  pAdvertising->addServiceUUID(tempService->getUUID());

  pAdvertising->setScanResponse(true);
  NimBLEHIDDevice device(pServer);
  device.pnp(3, 0xf0a, 0, 0x03);
  device.startServices();
  pAdvertising->start();

  DEBUG_SERIAL.println("Advertising Started");
}
#else
void initBT() {
  DEBUG_SERIAL.println("Bluetooth not supported on this platform");
}
#endif

std::vector<uint8_t> getTemperature() {
  float last = thermocouple.getTemperature();
  int status = thermocouple.read();
  DEBUG_SERIAL.printf("status: %d\n", status);
  std::vector<uint8_t> temperature;
  // read actual temperature from sensor
  float tipTemperature = 25.7;
  if (status == STATUS_OK) {
    float temp = thermocouple.getTemperature(); // thermocouple.getInternal();
    DEBUG_SERIAL.printf("probe: ");
    DEBUG_SERIAL.println(temp);
    /*DEBUG_SERIAL.printf("Internal Temp: %.2f\n", temp);*/
    /*tipTemperature = temp;*/
    /*float internal = thermocouple.getInternal();*/
    /*DEBUG_SERIAL.printf("internal: ");*/
    /*DEBUG_SERIAL.println(internal);*/
    /*DEBUG_SERIAL.printf("Internal Temp: %.2f\n", internal);*/
  } else {
    DEBUG_SERIAL.println("Error reading thermocouple!");
  }

  // Reverse the process to get temperatureRawStatus[0] and
  // temperatureRawStatus[1]
  int tempSum = static_cast<int>(tipTemperature * 16 - 8);
  int temperatureRawStatus1 = static_cast<int>(std::floor(tempSum / 256.0));
  int temperatureRawStatus0 = tempSum - temperatureRawStatus1 * 256;
  temperature.push_back(temperatureRawStatus0);
  temperature.push_back(temperatureRawStatus1);
  return temperature;
}
double R1 = 10000.0;  // voltage divider resistor value
double Beta = 3950.0; // Beta value
double To = 298.15;   // Temperature in Kelvin for 25 degree Celsius
double Ro = 0.0;      // Resistance of Thermistor at 25 degree Celsius
int ThermistorPin = 10;
double adcMax = 4095.0;
double Vs = 3.3;

std::vector<uint8_t> readNTC() {
  std::vector<uint8_t> temperature;
  const double Tc = thermistor->readCelsius();
	/*const int resistorValue = 10000;*/
	/*int sensorValue = analogRead(10);*/
	/*float resistance = resistorValue / ((1023.0 / sensorValue) - 1.0);*/
	/*float Tc = log(resistance / 220000.0) / 3950.0 + 1.0 / (25 + 273.15);*/
  DEBUG_SERIAL.println(Tc);

  int tempSum = static_cast<int>(Tc * 16 - 8);
  int temperatureRawStatus1 = static_cast<int>(std::floor(tempSum / 256.0));
  int temperatureRawStatus0 = tempSum - temperatureRawStatus1 * 256;
  temperature.push_back(temperatureRawStatus0);
  temperature.push_back(temperatureRawStatus1);
  return temperature;
}

void setup() {
  delay(3000);
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("Starting BLE work!");
  initBT();

  delay(500);
	thermistor = new NTC_Thermistor_ESP32(10, 10, 220000, 25, 3950, 3300, 4095);
  /*thermocouple.begin();*/
}

void loop() {
  delay(1000);
#ifndef STM32
  NimBLEService *tempService =
      pServer->getServiceByUUID("a75cc7fc-c956-488f-ac2a-2dbc08b63a04");
  NimBLECharacteristic *temperatureCharacteristic =
      tempService->getCharacteristic("7edda774-045e-4bbf-909b-45d1991a2876");
  temperatureCharacteristic->setValue(readNTC());
  temperatureCharacteristic->notify();
#else
  getTemperature();
#endif
}
