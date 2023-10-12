#include <ESP32Servo.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
Servo servo1;
Servo servo2;

int servo1Pin = 32;
int servo2Pin = 33;
const char *ssid = "GTvistor"; // don't steal my wifi password random github user. Also we did not name it as a joke. The sweet old lady in our house before us had the name Gay Love. Click the link if you dont believe me! https://www.legacy.com/us/obituaries/atlanta/name/gay-love-obituary?id=16562422
//const char *password = "12345678";

AsyncWebServer server(80);
WebSocketsServer webSocket(81);  // WebSocket server on port 81

float value1 = 0.0;
float value2 = 0.0;
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_TEXT:
      Serial.printf("[%u] Received text: %s\n", num, payload);
      break;
    case WStype_CONNECTED:
      Serial.printf("[%u] WebSocket client connected.\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] WebSocket client disconnected.\n", num);
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid);
  //WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(0);
  servo2.write(0);
  delay(1000);
  servo1.write(90);
  servo2.write(90);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = "<html><body>";
    html += "<h1>Float Values</h1>";
    html += "<p>Azimuth: <span id='value1'>" + String(value1) + "</span></p>";
    html += "<p>Altitude: <span id='value2'>" + String(value2) + "</span></p>";
    html += "<script>\n";
    html += "var socket = new WebSocket('ws://' + window.location.hostname + ':81/');\n";
    html += "socket.onmessage = function(event) {\n";
    html += "  var data = JSON.parse(event.data);\n";
    html += "  document.getElementById('value1').innerText = data.value1;\n";
    html += "  document.getElementById('value2').innerText = data.value2;\n";
    html += "}\n";
    html += "</script>\n";
    html += "</body></html>";
    request->send(200, "text/html", html);
  });

  // Start server
  server.begin();

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}


void loop() {
  if (Serial.available() >= 8) {
    int degrees1;
    int degrees2;

    Serial.readBytes((char *)&value1, sizeof(value1));
    Serial.readBytes((char *)&value2, sizeof(value2));
    String json = "{\"value1\":" + String(value1, 2) + ", \"value2\":" + String(value2, 2) + "}";
    webSocket.broadcastTXT(json);
    // Set the servo positions in degrees
    servo1.write(degrees1);
    servo2.write(degrees2);

    // Print the servo positions to the serial monitor
    Serial.print("Servo 1 Position (degrees): ");
    Serial.println(value1);
    Serial.print("Servo 2 Position (degrees): ");
    Serial.println(value2);
  }
}
