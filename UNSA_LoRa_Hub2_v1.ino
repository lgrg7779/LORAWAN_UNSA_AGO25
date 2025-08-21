#include <Adafruit_GFX.h>                 // Gráficos básicos para pantallas
#include <Adafruit_SSD1306.h>             // Controlador OLED SSD1306
#include <Arduino.h>                      // Núcleo de Arduino
#include <ArduinoModbus.h>                // Cliente Modbus RTU/ASCII
#include <DallasTemperature.h>            // Sensor DS18B20 (OneWire)
#include "DFRobot_MultiGasSensor.h"       // Sensor multigas DFRobot (NH3)
#include <MKRWAN.h>                       // Soporte LoRa/LoRaWAN MKR
#include <OneWire.h>                      // Bus OneWire para DS18B20
#include <SensirionI2cScd4x.h>            // SCD4x (CO2, Temp, HR) por I2C
#include <SPI.h>                          // SPI (para ciertos periféricos)
#include <Wire.h>                         // I2C (bus de sensores)

// --------------------------- LoRa modem ---------------------------
LoRaModem modem(Serial1);                 // Módem LoRa usando Serial1 del MKR

// ⚠️ Reemplaza por tus credenciales y rota la clave si ya fue expuesta
String appEui = "0000000000000000";       // AppEUI (OTAA)
String appKey = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"; // AppKey (OTAA)

// ---------------------- Configuración del OLED --------------------
#define SCREEN_WIDTH     128              // Ancho en píxeles
#define SCREEN_HEIGHT     32              // Alto en píxeles
#define SCREEN_ADDRESS  0x3C              // Dirección I2C del OLED
#define OLED_RESET       -1               // Sin pin de reset (a -1)

// --------------- Pines y constantes de sensores analógicos --------
#define ONE_WIRE_BUS       1              // Pin para OneWire (DS18B20)
#define VOLTAGE         3.30              // Vref usada en fórmulas ORP
#define OFFSET            40              // Offset de calibración ORP (mV)
#define ArrayLenth        40              // Tamaño de ventana para ORP
#define orpPin            A2              // Pin analógico ORP
const int   sensorPin         = A1;       // Pin analógico pH
const float adcVoltageRef     = 3.3;      // Referencia del ADC para pH
const int   adcResolution     = 4095;     // ⚠️ ADC de 12 bits (0–4095)
const float sensorMaxVoltage  = 2.0;      // Máx V esperado del módulo pH

// --------------- Objetos de display y sensores digitales ----------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // OLED
OneWire oneWire(ONE_WIRE_BUS);           // Bus OneWire (DS18B20)
DallasTemperature sensors(&oneWire);      // Objeto DS18B20
SensirionI2cScd4x sensor;                 // Objeto SCD41/40 (CO2/Temp/HR)

// ----------------------- Sensor NH3 por I2C -----------------------
#define I2C_COMMUNICATION                  // Modo I2C en la lib DFRobot
#define I2C_ADDRESS 0x74                   // Dirección por defecto
DFRobot_GAS_I2C gas(&Wire, I2C_ADDRESS);  // Objeto de sensor de gas (NH3)

// ----------------- Buffers/errores del SCD41 (Sensirion) ----------
static char    errorMessage[64];           // Texto de error legible
static int16_t error;                      // Código de error
#define NO_ERROR 0                         // Indicador de “sin error”

// ---------------------------- ORP ---------------------------------
double orpValue;                           // Resultado ORP (mV aprox)
int    orpArray[ArrayLenth];               // Ventana circular de muestras
int    orpArrayIndex = 0;                  // Índice de la ventana

// ------------------------- RS485 (hoja) ---------------------------
float LeafTemp;                            // Temperatura de hoja (°C)
float LeafWet;                             // Humedad hoja (%)
bool  modbusConnected = false;             // Estado del cliente Modbus

// ---------- Utilidad: imprimir uint64 en HEX (S/N del SCD41) ------
void PrintUint64(uint64_t& value) {
  Serial.print("0x");                      // Prefijo HEX
  Serial.print((uint32_t)(value >> 32), HEX);     // Parte alta (32 bits)
  Serial.print((uint32_t)(value & 0xFFFFFFFF), HEX); // Parte baja
}

// ------ Filtro: promedio eliminando mínimo y máximo (anti-ruido) --
double averagearray(int* arr, int number) {
  int i, max, min;
  double avg;
  long amount = 0;
  if (number <= 0) return 0;               // Protección
  if (number < 5) {                        // Pocas muestras → promedio simple
    for (i = 0; i < number; i++) amount += arr[i];
    return amount / number;
  } else {                                 // Descartar extremos (min y max)
    if (arr[0] < arr[1]) { min = arr[0]; max = arr[1]; }
    else { min = arr[1]; max = arr[0]; }

    for (i = 2; i < number; i++) {
      if (arr[i] < min)      { amount += min; min = arr[i]; }
      else if (arr[i] > max) { amount += max; max = arr[i]; }
      else                   { amount += arr[i]; }
    }
    avg = (double)amount / (number - 2);   // Promedio sin extremos
    return avg;
  }
}

// ------------------------------ SETUP ------------------------------
void setup() {
  Serial.begin(9600);                      // Abre puerto serie a 9600

  // --------------------- LoRaWAN (banda US915) ---------------------
  if (!modem.begin(US915)) {               // Inicia el módem LoRa
    Serial.println("Failed to start module");
    while (1) {}                           // Detiene si falla
  }
  int connected = modem.joinOTAA(appEui, appKey); // Unión OTAA con red
  if (!connected) {
    Serial.println("LoRa failed. Try near a window.");
    while (1) {}                           // Detiene si no se asocia
  }
  modem.minPollInterval(60);               // Intervalo mínimo para RX=60 s

  // --------------------- Buses y pantallas -------------------------
  Wire.begin();                            // Inicia bus I2C

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // Inicia OLED
    Serial.println(F("SSD1306 allocation failed"));           // Verifica
    while (1);                                                // Detiene si falla
  }
  display.clearDisplay();                  // Limpia framebuffer
  display.display();                       // Refresca (pantalla en negro)
  delay(500);                              // Pequeña pausa visual

  // ----------------------- Sensores digitales ----------------------
  sensors.begin();                         // DS18B20 (OneWire)

  sensor.begin(Wire, SCD41_I2C_ADDR_62);   // SCD41 en 0x62
  uint64_t serialNumber = 0;               // Para almacenar S/N

  error = sensor.wakeUp();                 // Despierta el SCD41
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  error = sensor.stopPeriodicMeasurement();// Asegura estado conocido
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  error = sensor.reinit();                 // Re-inicializa el sensor
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  error = sensor.getSerialNumber(serialNumber); // Lee S/N
  if (error == NO_ERROR) {
    Serial.print("Serial Number: ");
    PrintUint64(serialNumber);
    Serial.println();
  }

  error = sensor.startPeriodicMeasurement();// Comienza medición periódica
  if (error != NO_ERROR) {
    errorToString(error, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
  }

  // --------------------- Sensor NH3 (DFRobot) ----------------------
  while (!gas.begin()) {                   // Reintenta hasta detectar
    Serial.println("No NH3 sensor found!");
    delay(1000);
  }
  gas.setTempCompensation(gas.ON);         // Compensación térmica activada
  gas.changeAcquireMode(gas.INITIATIVE);   // Modo de adquisición activa
  delay(1000);                             // Estabilización

  // ----------------------- RS485 (hoja) ----------------------------
  Serial.println("S-YM-01B Leaf Wetness & Temperature Sensor");
  modbusConnected = ModbusRTUClient.begin(9600, SERIAL_8N1); // Inicia Modbus
  if (!modbusConnected) Serial.println("Modbus failed. Retrying...");

  // ----------------- Resolución del ADC (analógicos) ---------------
  analogReadResolution(12);                // 12 bits → [0..4095]
}

// ------------------------------- LOOP ------------------------------
void loop() {
  // ----------------- DS18B20: temperatura de agua ------------------
  sensors.requestTemperatures();                   // Dispara conversión
  float tempC = sensors.getTempCByIndex(0);        // Lee primer sensor
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Temp sensor disconnected");
  }

  // -------------------- SCD41: CO2, Temp, HR -----------------------
  bool  dataReady = false;                         // Flag de datos listos
  uint16_t co2Concentration = 0;                   // CO2 en ppm
  float temperature = 0.0;                         // Temp ambiente (°C)
  float relativeHumidity = 0.0;                    // Humedad relativa (%)

  error = sensor.getDataReadyStatus(dataReady);    // Pregunta si hay datos
  while (!dataReady) {                             
    error = sensor.getDataReadyStatus(dataReady);  // Espera activa
    if (error != NO_ERROR) return;                 // Sale si hay error
  }
  error = sensor.readMeasurement(co2Concentration, temperature, relativeHumidity); // Lee medición
  if (error != NO_ERROR) return;                   // Sale si hay error

  // ---------------------- NH3 (DFRobot) ----------------------------
  // En varios ejemplos, la librería actualiza AllDataAnalysis.gasconcentration.
  // Si tu versión usa otra API (p.ej. gas.readGasConcentration(&ppm)),
  // reemplaza la línea donde se imprime/usa NH3 más abajo.
  // Aquí solo confirmamos disponibilidad de datos.
  if (gas.dataIsAvailable()) {
    // Datos NH3 listos (la lib suele actualizar variables internas)
  }

  // --------------- RS485: sensor de hoja S-YM-01B -----------------
  if (modbusConnected) {
    // Lee 2 registros de entrada desde el esclavo 11 (temp*100, hum*100)
    if (!ModbusRTUClient.requestFrom(11, INPUT_REGISTERS, 0x0000, 2)) {
      Serial.print("RS485 Read error: ");
      Serial.println(ModbusRTUClient.lastError());
      modbusConnected = false;                      // Marca desconexión
    } else {
      int16_t  rawTemp     = ModbusRTUClient.read(); // Reg 0: temp*100
      uint16_t rawHumidity = ModbusRTUClient.read(); // Reg 1: hum*100
      LeafTemp = rawTemp / 100.0;                    // Convierte a °C
      LeafWet  = rawHumidity / 100.0;                // Convierte a %
    }
  } else {
    // Reintento de conexión Modbus si se cayó
    if (ModbusRTUClient.begin(9600, SERIAL_8N1)) {
      modbusConnected = true;
      Serial.println("Reconnected Modbus!");
    } else {
      Serial.println("Modbus retry in 1 second...");
    }
  }

  // ---------------------- ORP con ventana/filtro -------------------
  static unsigned long orpTimer  = millis();       // Temporizador de muestreo
  static unsigned long printTime = millis();       // (reservado/log)

  if (millis() >= orpTimer) {                      // Cada 20 ms
    orpTimer = millis() + 20;
    orpArray[orpArrayIndex++] = analogRead(orpPin); // Guarda muestra cruda
    if (orpArrayIndex == ArrayLenth) orpArrayIndex = 0; // Índice circular

    // ⚠️ AJUSTE: usar resolución real del ADC (4095) y no 1024
    double avgSample = averagearray(orpArray, ArrayLenth); // Promedio
    orpValue = ((30 * VOLTAGE * 1000) 
               - (75 * avgSample * VOLTAGE * 1000 / adcResolution)) / 75
               - OFFSET;                                  // mV aprox
  }

  // ------------------------- pH (aproximado) -----------------------
  int   rawADC  = analogRead(sensorPin);               // Muestra cruda
  float voltage = (rawADC * adcVoltageRef) / adcResolution; // Voltaje
  float ph      = (voltage / sensorMaxVoltage) * 14.0; // Escala 0–14 (lineal)

  // ---------------------- Salida por puerto serie ------------------
  Serial.print("Water Temp: "); Serial.println(tempC);                 // DS18B20
  Serial.print("Env Temp: ");  Serial.println(temperature);            // SCD41
  Serial.print("Humidity: ");  Serial.println(relativeHumidity);       // SCD41
  Serial.print("CO2: ");       Serial.println(co2Concentration);       // SCD41
  Serial.print("NH3: ");       Serial.println(AllDataAnalysis.gasconcentration); // DFRobot (ver nota)
  Serial.print("ORP: ");       Serial.println((int)orpValue);          // mV aprox
  Serial.print("Leaf Temp: "); Serial.println(LeafTemp);               // RS485
  Serial.print("Leaf Wet: ");  Serial.println(LeafWet);                // RS485
  Serial.print("pH: ");        Serial.println(ph);                     // pH estimado
  Serial.println();                                                     // Separador

  // ------------------------- OLED (pantalla) -----------------------
  display.clearDisplay();                           // Limpia buffer
  display.setTextSize(1);                           // Tamaño de texto
  display.setTextColor(WHITE);                      // Color de texto
  display.setCursor(0, 0);                          // Origen
  display.print("WT:"); display.print(tempC);       // Temp agua
  display.print(" ET:"); display.println(temperature); // Temp ambiente
  display.print("HR:"); display.print(relativeHumidity); // Humedad
  display.print(" CO2:"); display.println(co2Concentration); // CO2
  display.print("NH3:"); display.print(AllDataAnalysis.gasconcentration); // NH3
  display.print(" ORP:"); display.println((int)orpValue);   // ORP
  display.print("LT:");  display.print(LeafTemp);           // Temp hoja
  display.print(" LW:"); display.println(LeafWet);          // Humedad hoja
  display.display();                                        // Refresca OLED

  // -------------------- Alias para empaquetado ---------------------
  float    WT  = tempC;                         // Agua (°C)
  float    ET  = temperature;                   // Ambiente (°C)
  float    HR  = relativeHumidity;              // Humedad relativa (%)
  uint16_t CO2 = co2Concentration;              // CO2 (ppm)
  float    NH3 = AllDataAnalysis.gasconcentration; // NH3 (ppm, según lib)
  float    ORP = orpValue;                      // ORP (mV)
  float    LT  = LeafTemp;                      // Temp hoja (°C)
  float    LW  = LeafWet;                       // Humedad hoja (%)
  float    pH_ = ph;                            // pH (0–14)

  // ------------------- Payload 1 (10 bytes) ------------------------
  // Orden: CO2 (2B), WT*100 (2B), ET*100 (2B), HR*100 (2B), NH3*100 (2B)
  uint8_t payload1[10] = {
    CO2 >> 8, CO2 & 0xFF,
    (uint16_t)(WT * 100) >> 8, (uint16_t)(WT * 100) & 0xFF,
    (uint16_t)(ET * 100) >> 8, (uint16_t)(ET * 100) & 0xFF,
    (uint16_t)(HR * 100) >> 8, (uint16_t)(HR * 100) & 0xFF,
    (uint16_t)(NH3 * 100) >> 8, (uint16_t)(NH3 * 100) & 0xFF
  };

  // ------------------- Payload 2 (8 bytes) -------------------------
  // Orden: ORP*100 (2B, con signo), LT*100 (2B), LW*100 (2B), pH*100 (2B)
  uint8_t payload2[8] = {
    (int16_t)(ORP * 100) >> 8, (int16_t)(ORP * 100) & 0xFF,
    (uint16_t)(LT * 100) >> 8, (uint16_t)(LT * 100) & 0xFF,
    (uint16_t)(LW * 100) >> 8, (uint16_t)(LW * 100) & 0xFF,
    (uint16_t)(pH_ * 100) >> 8, (uint16_t)(pH_ * 100) & 0xFF
  };

  // -------------------- Envío: payload 1 (fPort=2) -----------------
  modem.setPort(2);                              // Puerto 2
  modem.beginPacket();                           // Inicia paquete
  for (int i = 0; i < 10; i++) {                 // Log hexdump por serie
    if (payload1[i] < 16) Serial.print("0");
    Serial.print(payload1[i], HEX); Serial.print(" ");
  }
  modem.write(payload1, sizeof(payload1));       // Escribe payload
  int error1 = modem.endPacket(true);            // TX con confirmación
  if (error1 > 0) Serial.println("Payload 1 sent!");
  else            Serial.println("Error sending payload 1");

  delay(10000);                                   // Espera entre TX

  // -------------------- Envío: payload 2 (fPort=3) -----------------
  modem.setPort(3);                              // Puerto 3
  modem.beginPacket();                           // Inicia paquete
  for (int i = 0; i < 8; i++) {                  // Log hexdump por serie
    if (payload2[i] < 16) Serial.print("0");
    Serial.print(payload2[i], HEX); Serial.print(" ");
  }
  modem.write(payload2, sizeof(payload2));       // Escribe payload
  int error2 = modem.endPacket(true);            // TX con confirmación
  if (error2 > 0) Serial.println("Payload 2 sent!");
  else            Serial.println("Error sending payload 2");
  
  delay(10000);                                   // Ritmo de transmisión
}
