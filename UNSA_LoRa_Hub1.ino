#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <ArduinoModbus.h>
#include <DallasTemperature.h>
#include "DFRobot_MultiGasSensor.h"
#include <MKRWAN.h>
#include <OneWire.h>
#include <SensirionI2cScd4x.h>
#include <SPI.h>
#include <Wire.h>

// LoRa modem
LoRaModem modem(Serial1);
String appEui = "0000000000000000";
String appKey = "26a184d167f2d5a71e16274ab9692e88";

// OLED Display configuration
#define SCREEN_WIDTH     128
#define SCREEN_HEIGHT     32
#define SCREEN_ADDRESS  0x3C
#define OLED_RESET       -1

// Sensor Pins and Constants
#define ONE_WIRE_BUS       1
#define VOLTAGE         3.30
#define OFFSET            40
#define ArrayLenth        40
#define orpPin            A2
const int sensorPin         = A1;
const float adcVoltageRef   = 3.3;
const int adcResolution     = 4095;
const float sensorMaxVoltage = 2.0;

// Display and sensors initialization
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
SensirionI2cScd4x sensor;

// NH3 gas sensor over I2C
#define I2C_COMMUNICATION
#define I2C_ADDRESS 0x74
DFRobot_GAS_I2C gas(&Wire, I2C_ADDRESS);

// SCD41 status and error buffers
static char errorMessage[64];
static int16_t error;
#define NO_ERROR 0

// ORP variables
double orpValue;
int orpArray[ArrayLenth];
int orpArrayIndex = 0;

// RS485 Sensor values
float LeafTemp;
float LeafWet;
bool modbusConnected = false;

// Function to print 64-bit serial number
void PrintUint64(uint64_t& value) {
  Serial.print("0x");
  Serial.print((uint32_t)(value >> 32), HEX);
  Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX);
}

// Average array values excluding outliers
double averagearray(int* arr, int number) {
  int i, max, min;
  double avg;
  long amount = 0;
  if (number <= 0) return 0;
  if (number < 5) {
    for (i = 0; i < number; i++) amount += arr[i];
    return amount / number;
  } else {
    if (arr[0] < arr[1]) { min = arr[0]; max = arr[1]; }
    else { min = arr[1]; max = arr[0]; }

    for (i = 2; i < number; i++) {
      if (arr[i] < min) { amount += min; min = arr[i]; }
      else if (arr[i] > max) { amount += max; max = arr[i]; }
      else { amount += arr[i]; }
    }
    avg = (double)amount / (number - 2);
    return avg;
  }
}

// Setup function
void setup() {
  Serial.begin(9600);

  // Start LoRaWAN
  if (!modem.begin(US915)) {
    Serial.println("Failed to start module");
    while (1) {}
  }

  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("LoRa failed. Try near a window.");
    while (1) {}
  }
  modem.minPollInterval(60);

  // Start I2C
  Wire.begin();

  // Initialize sensors
  sensor.begin(Wire, SCD41_I2C_ADDR_62);
  sensors.begin();
  gas.begin();
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  // OLED test screen
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }
  display.display();
  delay(1000);

  // Initialize SCD41
  uint64_t serialNumber = 0;
  error = sensor.wakeUp();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  error = sensor.stopPeriodicMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  error = sensor.reinit();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  error = sensor.getSerialNumber(serialNumber);
  if (error == NO_ERROR) {
    Serial.print("Serial Number: ");
    PrintUint64(serialNumber);
    Serial.println();
  }

  error = sensor.startPeriodicMeasurement();
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  // NH3 Sensor Configuration
  while (!gas.begin()) {
    Serial.println("No NH3 sensor found!");
    delay(1000);
  }
  gas.setTempCompensation(gas.ON);
  gas.changeAcquireMode(gas.INITIATIVE);
  delay(1000);

  // RS485 leaf sensor init
  Serial.println("S-YM-01B Leaf Wetness & Temperature Sensor");
  modbusConnected = ModbusRTUClient.begin(9600, SERIAL_8N1);
  if (!modbusConnected) Serial.println("Modbus failed. Retrying...");

  // ADC resolution
  analogReadResolution(12);
}

// Main loop
void loop() {
  // Read temperature from DS18B20
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Temp sensor disconnected");
  }

  // Read data from SCD41
  bool dataReady = false;
  uint16_t co2Concentration = 0;
  float temperature = 0.0;
  float relativeHumidity = 0.0;

  error = sensor.getDataReadyStatus(dataReady);
  while (!dataReady) {
    error = sensor.getDataReadyStatus(dataReady);
    if (error != NO_ERROR) return;
  }

  error = sensor.readMeasurement(co2Concentration, temperature, relativeHumidity);
  if (error != NO_ERROR) return;

  // Read NH3 concentration
  if (gas.dataIsAvailable()) {
    // NH3 data ready
  }

  // Read RS485 leaf sensor
  if (modbusConnected) {
    if (!ModbusRTUClient.requestFrom(11, INPUT_REGISTERS, 0x0000, 2)) {
      Serial.print("RS485 Read error: ");
      Serial.println(ModbusRTUClient.lastError());
      modbusConnected = false;
    } else {
      int16_t rawTemp = ModbusRTUClient.read();
      uint16_t rawHumidity = ModbusRTUClient.read();
      LeafTemp = rawTemp / 100.0;
      LeafWet = rawHumidity / 100.0;
    }
  } else {
    if (ModbusRTUClient.begin(9600, SERIAL_8N1)) {
      modbusConnected = true;
      Serial.println("Reconnected Modbus!");
    } else {
      Serial.println("Modbus retry in 1 second...");
    }
  }

  // ORP reading
  static unsigned long orpTimer = millis();
  static unsigned long printTime = millis();

  if (millis() >= orpTimer) {
    orpTimer = millis() + 20;
    orpArray[orpArrayIndex++] = analogRead(orpPin);
    if (orpArrayIndex == ArrayLenth) orpArrayIndex = 0;

    orpValue = ((30 * VOLTAGE * 1000) - (75 * averagearray(orpArray, ArrayLenth) * VOLTAGE * 1000 / 1024)) / 75 - OFFSET;
  }

  // pH reading
  int rawADC = analogRead(sensorPin);
  float voltage = (rawADC * adcVoltageRef) / adcResolution;
  float ph = (voltage / sensorMaxVoltage) * 14.0;

  // Print all data
  Serial.print("Water Temp: "); Serial.println(tempC);
  Serial.print("Env Temp: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(relativeHumidity);
  Serial.print("CO2: "); Serial.println(co2Concentration);
  Serial.print("NH3: "); Serial.println(AllDataAnalysis.gasconcentration);
  Serial.print("ORP: "); Serial.println((int)orpValue);
  Serial.print("Leaf Temp: "); Serial.println(LeafTemp);
  Serial.print("Leaf Wet: "); Serial.println(LeafWet);
  Serial.print("pH: "); Serial.println(ph);
  Serial.println();

  // Show values on OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("WT:"); display.print(tempC);
  display.print(" ET:"); display.println(temperature);
  display.print("HR:"); display.print(relativeHumidity);
  display.print(" CO2:"); display.println(co2Concentration);
  display.print("NH3:"); display.print(AllDataAnalysis.gasconcentration);
  display.print(" ORP:"); display.println((int)orpValue);
  display.print("LT:"); display.print(LeafTemp);
  display.print(" LW:"); display.println(LeafWet);
  display.display();

  // Prepare payloads
  float WT = tempC;
  float ET = temperature;
  float HR = relativeHumidity;
  uint16_t CO2 = co2Concentration;
  float NH3 = AllDataAnalysis.gasconcentration;
  float ORP = orpValue;
  float LT = LeafTemp;
  float LW = LeafWet;
  float pH = ph;

  uint8_t payload1[10] = {
    CO2 >> 8, CO2 & 0xFF,
    (uint16_t)(WT * 100) >> 8, (uint16_t)(WT * 100) & 0xFF,
    (uint16_t)(ET * 100) >> 8, (uint16_t)(ET * 100) & 0xFF,
    (uint16_t)(HR * 100) >> 8, (uint16_t)(HR * 100) & 0xFF,
    (uint16_t)(NH3 * 100) >> 8, (uint16_t)(NH3 * 100) & 0xFF
  };

  uint8_t payload2[8] = {
    (int16_t)(ORP * 100) >> 8, (int16_t)(ORP * 100) & 0xFF,
    (uint16_t)(LT * 100) >> 8, (uint16_t)(LT * 100) & 0xFF,
    (uint16_t)(LW * 100) >> 8, (uint16_t)(LW * 100) & 0xFF,
    (uint16_t)(pH * 100) >> 8, (uint16_t)(pH * 100) & 0xFF
  };

  // Send payload 1
  modem.setPort(2);
  modem.beginPacket();
  for (int i = 0; i < 10; i++) {
    if (payload1[i] < 16) Serial.print("0");
    Serial.print(payload1[i], HEX); Serial.print(" ");
  }
  modem.write(payload1, sizeof(payload1));
  int error1 = modem.endPacket(true);
  if (error1 > 0) Serial.println("Payload 1 sent!");
  else Serial.println("Error sending payload 1");

  delay(10000); // Time between packets

  // Send payload 
  modem.setPort(3);
  modem.beginPacket();
  for (int i = 0; i < 8; i++) {
    if (payload2[i] < 16) Serial.print("0");
    Serial.print(payload2[i], HEX); Serial.print(" ");
  }
  modem.write(payload2, sizeof(payload2));
  int error2 = modem.endPacket(true);
  if (error2 > 0) Serial.println("Payload 2 sent!");
  else Serial.println("Error sending payload 2");
  
  delay(10000); // Time between packets 

}
