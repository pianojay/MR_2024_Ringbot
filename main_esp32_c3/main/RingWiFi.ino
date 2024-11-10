/*
  #################################################################
          RingWiFi.ino: for Ringbot main.ino
          
          It handles:
            - Wifi connection
            - Server hosting; HTTP with HTML
            - handle/execute commands, write response

          Put this in a same folder with main.ino
  #################################################################
*/

namespace RingWiFi {


// WiFi credentials
const char* ssid = "ESP_test";
const char* password = "ESP_test";

// Static IP configuration
IPAddress staticIP(192, 168, 1, 100);  // However, 192.168.4.1 for some reason (I really don't know why)
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
        body { 
            font-family: Arial; 
            text-align: center; 
            margin: 0px auto; 
            padding: 10px; 
            max-width: 360px;
        }
        h1 {
            font-size: 20px;
            margin: 10px 0;
        }
        .button {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 12px 12px;
            text-decoration: none;
            font-size: 18px;
            margin: 2px;
            cursor: pointer;
            border-radius: 4px;
            user-select: none;
            touch-action: none;
            min-width: 44px;
        }
        .button:active {
            background-color: #45a049;
        }
        .controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 5px;
            max-width: 180px;
            margin: 10px auto;
        }
        .pid-controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 5px;
            max-width: 360px;
            margin: 10px auto;
        }
        .pid-group {
            border: 1px solid #ccc;
            padding: 5px;
            border-radius: 4px;
        }
        .pid-group h3 {
            margin: 0 0 5px 0;
            font-size: 14px;
        }
        .pid-button {
            font-size: 14px;
            padding: 8px 4px;
        }
        .utility-buttons {
            display: flex;
            gap: 10px;
            justify-content: center;
            margin: 10px auto;
        }
        .stop-button {
            background-color: #ff4444;
            font-size: 18px;
            padding: 12px 24px;
        }
        .refresh-button {
            background-color: #2196F3;
            font-size: 18px;
            padding: 12px 24px;
        }
        #response {
            margin-top: 10px;
            padding: 8px;
            background-color: #f0f0f0;
            border-radius: 4px;
            font-family: monospace;
            font-size: 12px;
            white-space: pre-line;
            overflow-x: auto;
        }
    </style>
</head>
<body>
    <h1>ESP32 Control Panel</h1>
    
    <!-- Movement Controls -->
    <div class="controls">
        <div></div>
        <button class="button" id="up">W</button>
        <div></div>
        <button class="button" id="left">A</button>
        <button class="button" id="down">S</button>
        <button class="button" id="right">D</button>
    </div>

    <!-- Utility Buttons -->
    <div class="utility-buttons">
        <button class="button stop-button" id="stop">STOP</button>
        <button class="button refresh-button" id="refresh">↻</button>
    </div>

    <!-- PID Controls -->
    <div class="pid-controls">
        <div class="pid-group">
            <h3>Kp Control</h3>
            <button class="button pid-button" id="kp_up">+</button>
            <button class="button pid-button" id="kp_down">-</button>
        </div>
        <div class="pid-group">
            <h3>Ki Control</h3>
            <button class="button pid-button" id="ki_up">+</button>
            <button class="button pid-button" id="ki_down">-</button>
        </div>
        <div class="pid-group">
            <h3>Kd Control</h3>
            <button class="button pid-button" id="kd_up">+</button>
            <button class="button pid-button" id="kd_down">-</button>
        </div>
    </div>

    <div id="response">Response will appear here</div>

    <script>
        let activeButton = null;
        let commandInterval = null;
        const COMMAND_INTERVAL = 100; // Send command every 100ms

        const buttons = {
            'up': 'w',
            'down': 's',
            'left': 'a',
            'right': 'd',
            'stop': '0',
            'refresh': 'r',
            'kp_up': 'z',
            'kp_down': 'x',
            'ki_up': 'c',
            'ki_down': 'v',
            'kd_up': 'b',
            'kd_down': 'n'
        };

        // Continuous movement buttons
        const movementButtons = ['up', 'down', 'left', 'right'];

        async function sendCommand(cmd) {
            try {
                const response = await fetch(`/?cmd=${cmd}`);
                const data = await response.text();
                document.getElementById('response').innerText = data;
            } catch (error) {
                console.error('Error:', error);
                document.getElementById('response').innerText = 'Error sending command';
            }
        }

        function handleButtonPress(buttonId) {
            // Clear any existing interval
            if (commandInterval) {
                clearInterval(commandInterval);
                commandInterval = null;
            }

            // Send initial command immediately
            sendCommand(buttons[buttonId]);

            // For movement buttons, set up interval for continuous sending
            if (movementButtons.includes(buttonId)) {
                activeButton = buttonId;
                commandInterval = setInterval(() => {
                    sendCommand(buttons[buttonId]);
                }, COMMAND_INTERVAL);
            }
        }

        function handleButtonRelease() {
            // Clear interval and active button state
            if (commandInterval) {
                clearInterval(commandInterval);
                commandInterval = null;
            }
            activeButton = null;
        }

        // Add event listeners for all buttons
        Object.keys(buttons).forEach(buttonId => {
            const button = document.getElementById(buttonId);
            if (!button) return;

            // Mouse events
            button.addEventListener('mousedown', (e) => {
                e.preventDefault();
                handleButtonPress(buttonId);
            });

            // Touch events
            button.addEventListener('touchstart', (e) => {
                e.preventDefault();
                handleButtonPress(buttonId);
            });

            // Only add release events for movement buttons
            if (movementButtons.includes(buttonId)) {
                button.addEventListener('mouseup', handleButtonRelease);
                button.addEventListener('mouseleave', handleButtonRelease);
                button.addEventListener('touchend', handleButtonRelease);
                button.addEventListener('touchcancel', handleButtonRelease);
            }
        });

        // Keyboard controls
        const keyMap = {
            'w': 'up',
            's': 'down',
            'a': 'left',
            'd': 'right',
            'ArrowUp': 'up',
            'ArrowDown': 'down',
            'ArrowLeft': 'left',
            'ArrowRight': 'right',
            '0': 'stop',
            'r': 'refresh',
            'z': 'kp_up',
            'x': 'kp_down',
            'c': 'ki_up',
            'v': 'ki_down',
            'b': 'kd_up',
            'n': 'kd_down'
        };

        document.addEventListener('keydown', (e) => {
            const buttonId = keyMap[e.key];
            if (buttonId && !e.repeat) {
                handleButtonPress(buttonId);
            }
        });

        document.addEventListener('keyup', (e) => {
            const buttonId = keyMap[e.key];
            if (buttonId && movementButtons.includes(buttonId)) {
                handleButtonRelease();
            }
        });

        // Handle cases where mouse/touch leaves the window or page loses focus
        document.addEventListener('visibilitychange', handleButtonRelease);
        window.addEventListener('blur', handleButtonRelease);
    </script>
</body>
</html>
)rawliteral";

String command = "_";  // fetched at server.on
String response = "";  // send from server.on


void RingWiFisetup() {
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
  server.on("/", HTTP_GET, []() {   // Called when there is a request
    if (server.hasArg("cmd")) {     // commands: w, s, a, d, 0, z, x, c, v, b, n, r
      command = server.arg("cmd");  // This has to be global variable.
      // response is decided and modified at main.ino
      switch (command.charAt(0)) {
        case 'w':  // Forward acceleration
          ringvs += 5;
          break;
        case 's':  // Backward acceleration
          ringvs -= 5;
          break;
        case 'a':  // Roll Left
          rolls -= 5;
          break;
        case 'd':  // Roll Right
          rolls += 5;
          break;
        case '0':  // Reset: Full Stop
          ringvs = 0;
          rolls = 0;
          break;
        case 'z':  // Kp
          armkp += 5;
          break;
        case 'x':  // Kp
          armkp -= 5;
          break;
        case 'c':  // Ki
          armki += 5;
          break;
        case 'v':  // Ki
          armki -= 5;
          break;
        case 'b':  // Kd
          armkd += 5;
          break;
        case 'n':  // Kd
          armkd -= 5;
          break;
        default:  // (refresh)
          break;
      }
      response = "ringv=" + String(ringv, 2) + "ringvs=" + String(ringvs, 2);
      response += "\n";
      response += "rolli=" + String(rolli, 2) + "armao=" + String(armao, 2) + "rolls=" + String(rolls, 2);
      response += "\n";
      response += "Kp=" + String(armkp) + " Ki=" + String(armki) + " Kd=" + String(armkd);
      server.send(200, "text/plain", response);
    } else {
      server.send(200, "text/html", index_html);
    }
  });

  server.begin();  // HTTP server started
  Serial.println("HTTP server started");
}

void RingWiFiloop() {
  server.handleClient();
}

}  // End of namespace RingWiFi