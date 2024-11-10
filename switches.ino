#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <Servo.h>
//#include "Secrets.hpp" // Make sure this file contains the necessary secrets


String ssid = "IOT_HUB_AP";
String password = "RaspBerry";
String webSocketServerAddress = "192.168.4.1";
const int webSocketServerPort = 81;

WebSocketsClient webSocket;

String serial = "SW1";

float sensitivity = 0.66; // check sensitivity chart
float mCurrent = 30;      // 30A
int noise = 1;
float vOffset = 2.5; // some 5v
float cReading=0;


int c[1] = {A0};
int sensor[sizeof(c)];
float volt[sizeof(c)];
float current[sizeof(c)];

Servo servo[2];
int servoPin[2] = {D4, D1}; // [ports]
int state[2] = {0, 0};

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void setupWiFi();
float getReading();
void sendReading();
void registerSystem();
void serializeJson(const JsonDocument &doc, char *output, size_t outputSize); // Add the missing function declaration
void check_device(uint8_t *payload);
void excCmdEvent(bool cmd);
void setupServo();
void moveServo(int servoNum, int angle);
void sendState();

void setup()
{
  Serial.begin(115200);
  setupWiFi();
  webSocket.begin(webSocketServerAddress, webSocketServerPort, "/");
  webSocket.onEvent(webSocketEvent);
  setupServo();
  registerSystem();
}

void loop()
{
  webSocket.loop();
  cReading = getReading();
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 5000 && cReading>1) {
    sendReading(cReading); 
  }

  
  
}

void setupWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("WebSocket disconnected");
    break;
  case WStype_CONNECTED:
    Serial.println("WebSocket connected");
    registerSystem();
    break;
  case WStype_TEXT:
    Serial.printf("Received data: %s", payload);
    Serial.println();
    check_device(payload);
    break;
  }
}

float getReading()
{
  float currentReading;
  int sensorReading = analogRead(c[0]);
  float voltage = sensorReading * 5.0 / 1024.0;
  currentReading = ((voltage - vOffset) / sensitivity);
  if (currentReading > 1 && state[0] == 0)
  {
    state[0] = 1;
    sendState();
  }

  return currentReading;
}

void sendReading(float currentReading)
{
  

  StaticJsonDocument<96> doc;

  doc["type"] = "data";

  JsonObject data = doc.createNestedObject("data");
  data["clientId"] = serial;
  data["usage"] = currentReading;

  String sensorData;

  serializeJson(doc, sensorData);
  
  webSocket.sendTXT(sensorData);
}

void check_device(uint8_t *payload)
{
  StaticJsonDocument<192> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  const char *type = doc["type"]; // "register"

  JsonObject data = doc["data"];
  String data_ID = String(data["clientId"]); // "uuid"
  bool data_state = data["state"];           // true

  if (data_ID == serial)
  {
    excCmdEvent(data_state);
  }
}

void excCmdEvent(bool cmd)
{
  switch (cmd)
  {
  case true:
    moveServo(0, 0);
    delay(50);
    moveServo(0, 90);
    state[0] = 1;
    Serial.println("Command 1");
    break;
  case false:
    moveServo(1, 0);
    delay(50);
    moveServo(1, 90);
    state[0] = 0;
    Serial.println("Command 2");
    break;
  default:
    Serial.println("Command not found");
    break;
  }
}

void setupServo()
{
  for (int i = 0; i < 2; i++)
  {
    servo[i].attach(servoPin[i]);
  }
}

void moveServo(int servoNum, int angle)
{
  servo[servoNum].write(angle);
}

void registerSystem()
{
  StaticJsonDocument<128> doc;

  doc["type"] = "register";

  JsonObject data = doc.createNestedObject("data");
  data["clientId"] = serial;
  data["state"] = true;
  data["module"] = nullptr;

  String payload;
  serializeJson(doc, payload);
  webSocket.sendTXT(payload);
}

void sendState()
{
  StaticJsonDocument<96> doc;

  doc["type"] = "command";

  JsonObject data = doc.createNestedObject("data");
  data["clientId"] = serial;
  data["state"] = true;

  String payload;
  serializeJson(doc, payload);
  webSocket.sendTXT(payload);
}
