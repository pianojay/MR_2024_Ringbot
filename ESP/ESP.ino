#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// WiFi credentials
const char* ssid = "ESP_test";
const char* password = "ESP_test";

// Create web server object on port 80
ESP8266WebServer server(80);

// Variables to store data
String dataToSend = "Hello from ESP8266";
int sensorValue = 0;  // Example sensor value

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // 2. Once the ESP8266 is connected to WiFi, it will print its IP address to the Serial Monitor. You'll need this IP address for your MIT App Inventor app.

  // Define server endpoints
  server.on("/getData", HTTP_GET, handleGetData);  // send data from ESP to app
  server.on("/setData", HTTP_GET, handleSetData);  // recieve data from app

/*
  // Single endpoint that always sends data
  server.on("/", []() {
    if (server.hasArg("value")) {
      dataToSend = server.arg("value");  // Update data if value parameter exists
    }
    server.send(200, "text/plain", dataToSend);  // Always send current data
  });

  // Web1.Url = "http://192.168.1.100/"
  // will result in recieving data
  // Web1.Url = "http://192.168.1.100/?value=" + TextBox1.Text
  // will result in sending value and recieving data

*/

  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

// Handle requests to get data from the ESP8266
void handleGetData() {
  String response = "{\"data\":\"" + dataToSend + "\",\"sensor\":" + String(sensorValue) + "}";
  server.send(200, "text/plain", response);
}

// Handle requests to set data on the ESP8266
void handleSetData() {
  if (server.hasArg("value")) {
    String newValue = server.arg("value");
    dataToSend = newValue;
    server.send(200, "text/plain", "Data updated to: " + newValue);
  } else {
    server.send(400, "text/plain", "Missing value parameter");
  }
}