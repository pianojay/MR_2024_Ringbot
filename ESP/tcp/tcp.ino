#include <WiFi.h>

// WiFi credentials
const char* ssid = "ESP_test";
const char* password = "ESP_test";

// Static IP configuration
IPAddress staticIP(192, 168, 1, 100);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// TCP server on port 8080
WiFiServer server(8080);
WiFiClient client;

void setup() {
  Serial.begin(115200);

  // Configure static IP before connecting
  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start TCP server
  server.begin();
  Serial.println("TCP server started");
}

void loop() {
  // Check if client connected
  if (!client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New client connected");
      client.println("Connected to ESP32");  // Send welcome message
    }
  }
  
  // If client is connected
  if (client.connected()) {
    // If data available from client
    while (client.available()) {
      String command = client.readStringUntil('\n');
      String response = "Received: " + command;
      
      // If no input then there should be no command to process.

      // Handle movement commands
      if (command == "w") {
        response = "Moving Forward";
        // Add your forward movement code here
      }
      else if (command == "s") {
        response = "Moving Backward";
        // Add your backward movement code here
      }
      else if (command == "a") {
        response = "Moving Left";
        // Add your left movement code here
      }
      else if (command == "d") {
        response = "Moving Right";
        // Add your right movement code here
      }
      
      // Send response back to app
      client.println(response);
    }
  }
}