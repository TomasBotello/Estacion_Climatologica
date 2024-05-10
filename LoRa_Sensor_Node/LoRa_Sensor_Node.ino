#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <ArduinoJson.h>

#define ss 5
#define rst 14
#define dio0 2

Adafruit_BME680 bme; // Sensor BME680 connected via I2C

byte LocalAddress = 0x01;
byte Destination_ESP32_Slave = 0x02;

const float SEALEVELPRESSURE_HPA = 1013.25; // Definir SEALEVELPRESSURE_HPA aquí

unsigned long previousMillis_SendMSG = 0;
const long interval_SendMSG = 10000; // Enviar mensajes cada 5 segundos

float v2 = 0; // Velocidad máxima registrada

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRa.setPins(ss, rst, dio0);
  Serial.println("Start LoRa init...");
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  Serial.println("LoRa init succeeded.");

  // Inicializar el sensor BME680
  if (!bme.begin()) {
    Serial.println(F("Couldn't find a valid BME680 sensor, check wiring!"));
    while (1);
  }
  // Configurar sobremuestreo e inicializar el filtro
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320°C durante 150 ms

  pinMode(32, INPUT); // Configurar el pin A0 (32) como entrada para el sensor de viento
}

void loop() {
  unsigned long currentMillis_SendMSG = millis();
  
  if (currentMillis_SendMSG - previousMillis_SendMSG >= interval_SendMSG) {
    previousMillis_SendMSG = currentMillis_SendMSG;
    
    // Leer datos de los sensores
    float temperature = bme.temperature;
    float pressure = bme.pressure / 100.0; // Convertir de Pa a hPa
    float humidity = bme.humidity;
    float gas_resistance = bme.gas_resistance / 1000.0; // Convertir de Ohms a KOhms
    float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    
    // Leer la velocidad del viento
    float v1 = analogRead(32); // Lectura del sensor de viento en el pin A0 (32)
    float wind_speed = v1 * 0.02; // Convertir lectura analógica a velocidad del viento en Km/h
    
    // Formatear la velocidad del viento para mostrar tres dígitos decimales
    char buffer[10];
    dtostrf(wind_speed, 6, 3, buffer); // 6 es la longitud total, 3 es el número de dígitos decimales
    String formattedWindSpeed = String(buffer);

    // Actualizar la velocidad máxima
    if (wind_speed > v2) {
      v2 = wind_speed;
    }

    // Construir el mensaje con los datos de los sensores

// Construir el mensaje con los datos de los sensores
    StaticJsonDocument<200> doc;
    doc["temperatura"] = temperature;
    doc["presion"] = pressure;
    doc["humedad"] = humidity;
    doc["gas"] = gas_resistance;
    doc["altitud"] = altitude;
    doc["velocidad"] = wind_speed;
    doc["velocidad_maxima"] = v2;

    String OutgoingMessage;
    serializeJson(doc, OutgoingMessage);

    // Imprimir el mensaje saliente
    Serial.println("Sending message: " + OutgoingMessage);

    // Enviar el mensaje al esclavo
    sendMessage(OutgoingMessage, Destination_ESP32_Slave);
  }
}

void sendMessage(String Outgoing, byte Destination) {
  LoRa.beginPacket();
  LoRa.write(Destination);
  LoRa.write(LocalAddress);
  LoRa.write(Outgoing.length());
  LoRa.print(Outgoing);
  LoRa.endPacket();
}