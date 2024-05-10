#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#define ss 5
#define rst 14
#define dio0 2

byte LocalAddress = 0x02;
byte Destination_Master = 0x01;

#define WIFI_SSID "Redes"
#define WIFI_PASSWORD "Redes2002991"
#define API_ENDPOINT "https://estacion-meteorologica.onrender.com/insertar"

WiFiClient wifiClient;

void sendMessage(String Outgoing, byte Destination) {
  LoRa.beginPacket();
  LoRa.write(Destination);
  LoRa.write(LocalAddress);
  LoRa.write(Outgoing.length());
  LoRa.print(Outgoing);
  LoRa.endPacket();
}

void processJson(String json) {
  Serial.println("Received JSON: " + json);
  HTTPClient http;
  http.begin(API_ENDPOINT);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(json);
  if (httpResponseCode > 0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.print("HTTP Response: ");
    Serial.println(response);
  } else {
    Serial.print("Error in sending POST request: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}


void onReceive(int packetSize) {
  if (packetSize == 0) return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingLength = LoRa.read();

  String Incoming = "";
  while (LoRa.available()) {
    Incoming += (char)LoRa.read();
  }

  if (recipient != LocalAddress) {
    Serial.println("This message is not for me.");
    return;
  }

  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Message: " + Incoming);

  // Si el mensaje recibido es un JSON, procesarlo
  if (Incoming.startsWith("{") && Incoming.endsWith("}")) {
    processJson(Incoming);
  }
}

void setup() {
  Serial.begin(115200);
  LoRa.setPins(ss, rst, dio0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  Serial.println("LoRa init succeeded.");

  // Conectar WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
}

void loop() {
  onReceive(LoRa.parsePacket());
}
