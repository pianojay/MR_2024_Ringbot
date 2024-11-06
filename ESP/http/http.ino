#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "ESP_test";
const char* password = "ESP_test";

// Static IP configuration
IPAddress staticIP(192, 168, 1, 100);  // However, 192.168.4.1 for some reason
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// HTML page with continuous input handling
// PROGMEM: Store data at Flash instead of SRAM.
// What is rawliteral: https://dragontory.tistory.com/3
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
    <title>ESP32 Control Panel</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; margin: 0px auto; padding: 20px; }
        .button {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 16px 16px;
            text-decoration: none;
            font-size: 30px;
            margin: 2px;
            cursor: pointer;
            border-radius: 4px;
            user-select: none;
            touch-action: none;
        }
        .button:active {
            background-color: #45a049;
        }
        .controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 300px;
            margin: 0 auto;
        }
        #response {
            margin-top: 20px;
            padding: 10px;
            background-color: #f0f0f0;
            border-radius: 4px;
        }
    </style>
</head>
<body>
    <h1>ESP32 Control Panel</h1>
    <div class="controls">
        <div></div>
        <button class="button" id="up">W</button>
        <div></div>
        <button class="button" id="left">A</button>
        <button class="button" id="down">S</button>
        <button class="button" id="right">D</button>
    </div>
    <div id="response">Response will appear here</div>

    <script>
        let activeButton = null;
        let isProcessing = false;
        let commandInterval = null;
        const COMMAND_INTERVAL = 100; // Send command every 100ms

        const buttons = {
            'up': 'w',
            'down': 's',
            'left': 'a',
            'right': 'd'
        };

        function sendCommand(cmd) {
            if (isProcessing) return;
            isProcessing = true;

            fetch(`/?cmd=${cmd}`)
                .then(response => response.text())
                .then(data => {
                    document.getElementById('response').innerText = data;
                    isProcessing = false;
                })
                .catch(error => {
                    console.error('Error:', error);
                    document.getElementById('response').innerText = 'Error sending command';
                    isProcessing = false;
                });
        }

        function startCommand(buttonId) {
            if (activeButton === buttonId) return;
            stopCommand();
            activeButton = buttonId;
            sendCommand(buttons[buttonId]);
            commandInterval = setInterval(() => {
                sendCommand(buttons[buttonId]);
            }, COMMAND_INTERVAL);
        }

        function stopCommand() {
            if (commandInterval) {
                clearInterval(commandInterval);
                commandInterval = null;
            }
            if (activeButton) {
                sendCommand('stop');
                activeButton = null;
            }
        }

        // Mouse/Touch event handlers
        Object.keys(buttons).forEach(buttonId => {
            const button = document.getElementById(buttonId);
            
            // Mouse events
            button.addEventListener('mousedown', (e) => {
                e.preventDefault();
                startCommand(buttonId);
            });

            // Touch events
            button.addEventListener('touchstart', (e) => {
                e.preventDefault();
                startCommand(buttonId);
            });
        });

        // Global mouse/touch up handlers
        document.addEventListener('mouseup', stopCommand);
        document.addEventListener('touchend', stopCommand);
        document.addEventListener('touchcancel', stopCommand);

        // Handle cases where mouse/touch leaves the window
        document.addEventListener('visibilitychange', stopCommand);
        window.addEventListener('blur', stopCommand);

        // Keyboard controls
        const keyMap = {
            'w': 'up',
            's': 'down',
            'a': 'left',
            'd': 'right',
            'ArrowUp': 'up',
            'ArrowDown': 'down',
            'ArrowLeft': 'left',
            'ArrowRight': 'right'
        };

        document.addEventListener('keydown', (e) => {
            const buttonId = keyMap[e.key];
            if (buttonId && !e.repeat) {
                startCommand(buttonId);
            }
        });

        document.addEventListener('keyup', (e) => {
            const buttonId = keyMap[e.key];
            if (buttonId && activeButton === buttonId) {
                stopCommand();
            }
        });
    </script>
</body>
</html>
)rawliteral";

String command = "";
String response = "";

void setup() {
  Serial.begin(115200);

  // Configure static IP before connecting
  if (!WiFi.config(staticIP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }

  // Set WiFi to AP mode and configure
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Route for root / web page
  server.on("/", HTTP_GET, []() {
    if (server.hasArg("cmd")) {
      command = server.arg("cmd");  // This has to be global variable.
      response = "No command";      // This also has to be global variable.

      if (command == "w") {
        response = "Moving Forward";
      } else if (command == "s") {
        response = "Moving Backward";
      } else if (command == "a") {
        response = "Moving Left";
      } else if (command == "d") {
        response = "Moving Right";
      } else if (command == "stop") {
        response = "yaw or something passive like that";
      }
      server.send(200, "text/plain", response);
      Serial.println(response);
    } else {
      server.send(200, "text/html", index_html);
      Serial.println("Request detected!");
    }
  });

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  // set
  server.handleClient();  // get command
  command = "stop";
}