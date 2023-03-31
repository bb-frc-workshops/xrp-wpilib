#include <Arduino.h>
#include <ArduinoWebsockets.h>

using namespace websockets;

WebsocketsServer server;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("No WiFi Module");
    while (true);
  }

  Serial.println("Setting up Access Point...");
  bool result = WiFi.softAP("XRP-WPILib", "0123456789");
  if (result == true) {
    Serial.println("Ready");
  }
  else {
    Serial.println("Failed...");
  }

  server.listen(3300, "/wpilibws");

  Serial.print(server.available() ? "WS Server running and ready on " : "Server not running on ");
  Serial.println("XRP Robot");
  Serial.print("IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.print(", port: ");
  Serial.println(3300);
}

void loop() {
  auto client = server.accept();
  if (client.available()) {
    Serial.print("Client accepted...");
  }
}