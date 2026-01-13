// Include WiFi library for wireless communication
#include <WiFi.h>
// Include WebServer library for HTTP server functionality
#include <WebServer.h>

// ===== WiFi Access Point Configuration =====
// SSID (network name) that will be broadcast by ESP32
const char* ap_ssid = "ESP32-S3-N16R8";      // WiFi network name
// Password for connecting to the ESP32 WiFi network
const char* ap_password = "12345678";        // WiFi password (min 8 chars)
// Local IP address of the ESP32 access point
const IPAddress local_ip(192, 168, 4, 1);
// Gateway IP address (same as local_ip for AP mode)
const IPAddress gateway(192, 168, 4, 1);
// Subnet mask for the WiFi network
const IPAddress subnet(255, 255, 255, 0);

// ===== Web Server Instance =====
// Create WebServer object on port 80 (HTTP default port)
WebServer server(80);

// ===== Timing Variables =====
// Tracks the last time sensor data was updated
unsigned long lastSendTime = 0;
// Interval between sensor data updates in milliseconds (100ms)
const unsigned long sendInterval = 100;  // Send every 100ms

// ===== Data Storage =====
// Stores the last prepared sensor data as JSON string
String lastSensorData = "";

// ===== Keyboard State Tracking =====
// Stores the currently active/pressed key
String activeKey = "";
// Timestamp of when the key was last pressed
unsigned long lastKeyTime = 0;
// Time in milliseconds before key resets to idle (200ms)
const unsigned long keyResetTime = 200;  // Reset key after 200ms if no new input

// ===== Key Press Logging =====
// Stores all key press logs as a string with timestamps
String keyPressLog = "";
// Maximum number of log lines to keep in memory
const int maxLogLines = 50;  // Keep last 50 log entries

// ===== SETUP FUNCTION (runs once on startup) =====
void setup() {
  // Initialize serial communication at 115200 baud rate for debugging
  Serial.begin(115200);
  // Wait 2 seconds for serial monitor to initialize
  delay(2000);
  
  // Print startup message to serial monitor
  Serial.println("\n\nESP32-S3 WiFi Access Point");
  // Print separator line
  Serial.println("===========================");
  
  // Initialize ESP32 as WiFi access point
  startAccessPoint();
  
  // Setup all HTTP server routes and endpoints
  setupWebServer();
}

// ===== MAIN LOOP FUNCTION (runs continuously) =====
void loop() {
  // Handle incoming client requests to web server
  server.handleClient();
  
  // Check if it's time to update sensor data (every 100ms)
  if (millis() - lastSendTime >= sendInterval) {
    // Update timestamp of last sensor data update
    lastSendTime = millis();
    
    // Prepare sensor data JSON string
    lastSensorData = prepareSensorData();
    
    // Send data update to serial monitor for debugging
    Serial.println("Data updated: " + lastSensorData);
  }
  
  // Small delay to prevent watchdog timer from resetting ESP32
  delay(10);  // Small delay to prevent watchdog issues
}

// ===== START ACCESS POINT FUNCTION =====
void startAccessPoint() {
  // Print status message to serial
  Serial.println("Starting WiFi Access Point...");
  
  // Set ESP32 to Access Point mode (creates WiFi network)
  WiFi.mode(WIFI_AP);
  // Configure the Access Point with IP, gateway, and subnet
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  // Create the WiFi network with SSID and password
  boolean apStatus = WiFi.softAP(ap_ssid, ap_password);
  
  // Check if access point started successfully
  if (apStatus) {
    // Print success message
    Serial.println("Access Point Started!");
    // Print network name
    Serial.print("SSID: ");
    // Display the SSID being broadcast
    Serial.println(ap_ssid);
    // Print password label
    Serial.print("Password: ");
    // Display the password required to connect
    Serial.println(ap_password);
    // Print IP address label
    Serial.print("IP Address: ");
    // Display the IP address of the ESP32
    Serial.println(WiFi.softAPIP());
  } else {
    // Print failure message
    Serial.println("Failed to start Access Point");
  }
}

// ===== SETUP WEB SERVER ROUTES FUNCTION =====
void setupWebServer() {
  // ===== ROUTE 1: GET /data =====
  // Route to retrieve current sensor data in JSON format
  server.on("/data", HTTP_GET, []() {
    // Send sensor data as JSON response
    server.send(200, "application/json", lastSensorData);
  });
  
  // ===== ROUTE 2: POST /keypress =====
  // Route to receive keyboard input from Mac via WiFi
  server.on("/keypress", HTTP_POST, []() {
    // Check if request body exists
    if (server.hasArg("plain")) {
      // Get the request body containing JSON data
      String body = server.arg("plain");
      
      // Parse JSON and look for "key" field
      if (body.indexOf("\"key\"") != -1) {
        // Find the position after the colon and quote
        int keyStart = body.indexOf(":") + 2;
        // Find the closing quote position
        int keyEnd = body.indexOf("\"", keyStart);
        
        // Validate that positions are correct
        if (keyStart > 0 && keyEnd > keyStart) {
          // Extract the key character from JSON
          activeKey = body.substring(keyStart, keyEnd);
          // Update timestamp of last key press
          lastKeyTime = millis();
          
          // Create a log entry with timestamp
          String logEntry = "[" + String(millis() / 1000) + "s] KEY PRESSED: " + activeKey + " activated\n";
          // Add log entry to key press log
          keyPressLog += logEntry;
          
          // Count total lines in log
          int lineCount = 0;
          // Loop through log to count newlines
          for (int i = 0; i < keyPressLog.length(); i++) {
            // Increment count when newline found
            if (keyPressLog[i] == '\n') lineCount++;
          }
          // Check if log exceeds maximum allowed lines
          if (lineCount > maxLogLines) {
            // Find position of first newline
            int firstNewline = keyPressLog.indexOf('\n') + 1;
            // Remove oldest log entry by trimming from start
            keyPressLog = keyPressLog.substring(firstNewline);
          }
          
          // Create JSON response confirming key was received
          String response = "{\"status\":\"" + activeKey + " activated\",\"received\":true}";
          // Send response back to client with success code
          server.send(200, "application/json", response);
          
          // Also print to serial monitor for debugging
          Serial.println(logEntry);
          // Exit function after processing
          return;
        }
      }
    }
    
    // Send error response if JSON parsing failed
    server.send(400, "application/json", "{\"error\":\"Invalid key data\"}");
  });
  
  // ===== ROUTE 3: GET /logs =====
  // Route to retrieve the complete key press log
  server.on("/logs", HTTP_GET, []() {
    // Send log data as JSON response
    server.send(200, "application/json", "{\"logs\":\"" + keyPressLog + "\"}");
  });
  
  // ===== ROUTE 4: GET /status =====
  // Route to get status information about ESP32
  server.on("/status", HTTP_GET, []() {
    // Start building JSON status object
    String status = "{";
    // Add SSID (network name) to response
    status += "\"ssid\":\"" + String(ap_ssid) + "\",";
    // Add number of connected WiFi clients
    status += "\"connected_clients\":" + String(WiFi.softAPgetStationNum()) + ",";
    // Add uptime in seconds since startup
    status += "\"uptime\":" + String(millis() / 1000) + "";
    // Close JSON object
    status += "}";
    // Send status as JSON response
    server.send(200, "application/json", status);
  });
  
  // ===== ROUTE 5: GET / (root HTML page) =====
  // Route to serve the main web interface for keyboard monitoring
  server.on("/", HTTP_GET, []() {
    // Start building HTML document
    String html = "<html><head><title>ESP32-S3 Keyboard Monitor</title>";
    // Start CSS styling section
    html += "<style>";
    // Reset all default margins and padding
    html += "* { margin: 0; padding: 0; box-sizing: border-box; }";
    // Style body: dark terminal theme with green text
    html += "body { font-family: 'Courier New', monospace; background: #0a0a0a; color: #00ff00; height: 100vh; display: flex; justify-content: center; align-items: center; }";
    // Style main container: full height flex column
    html += ".container { width: 100%; height: 100%; display: flex; flex-direction: column; }";
    // Style header: dark gradient background with green border
    html += ".header { background: linear-gradient(135deg, #1a1a1a 0%, #0a0a0a 100%); padding: 20px; border-bottom: 3px solid #00ff00; text-align: center; }";
    // Style header title: larger bold text
    html += ".header h1 { font-size: 24px; letter-spacing: 2px; }";
    // Style content area: centered flex layout with scrolling
    html += ".content { flex: 1; display: flex; flex-direction: column; justify-content: center; align-items: center; padding: 30px; overflow-y: auto; }";
    // Style key display section: centered text area
    html += ".key-display { text-align: center; }";
    // Style key label text: small cyan text
    html += ".key-label { font-size: 14px; color: #00aaff; margin-bottom: 10px; letter-spacing: 1px; }";
    // Style key display box: large glowing yellow box with border
    html += ".key-box { font-size: 100px; font-weight: bold; color: #ffff00; text-shadow: 0 0 20px #ffff00; line-height: 1; min-height: 120px; display: flex; align-items: center; justify-content: center; border: 3px solid #00ff00; padding: 15px; border-radius: 10px; background: rgba(0, 255, 0, 0.05); width: 180px; }";
    // Style idle state: green text instead of yellow
    html += ".key-box.idle { color: #00ff00; text-shadow: 0 0 10px #00ff00; font-size: 35px; }";
    // Style button section: centered area for test buttons
    html += ".buttons-section { margin-top: 30px; text-align: center; }";
    // Style button label: cyan small text
    html += ".buttons-label { font-size: 12px; color: #00aaff; margin-bottom: 12px; letter-spacing: 1px; }";
    // Style button row: horizontal flex layout with gap
    html += ".button-row { display: flex; gap: 10px; justify-content: center; margin-bottom: 10px; }";
    // Style buttons: green border with hover effect
    html += ".btn { padding: 12px 20px; font-size: 16px; font-weight: bold; background: #0a0a0a; color: #00ff00; border: 2px solid #00ff00; cursor: pointer; border-radius: 5px; font-family: 'Courier New', monospace; transition: all 0.1s; letter-spacing: 1px; }";
    // Style button hover: fill with green background
    html += ".btn:hover { background: #00ff00; color: #0a0a0a; box-shadow: 0 0 20px #00ff00; }";
    // Style button active: scale down when clicked
    html += ".btn:active { transform: scale(0.95); }";
    // Style verification box: orange border box
    html += ".verification-box { margin-top: 20px; padding: 12px; border: 2px solid #ff6600; background: rgba(255, 102, 0, 0.1); border-radius: 5px; min-height: 25px; }";
    // Style verification label: small orange text
    html += ".verification-label { font-size: 11px; color: #ff6600; margin-bottom: 5px; }";
    // Style verification text: bold orange text
    html += ".verification-text { font-size: 14px; color: #ff6600; font-weight: bold; }";
    // Style success verification: green text and border
    html += ".verification-text.success { color: #00ff00; border-color: #00ff00; }";
    // Style log section: container for key press log
    html += ".logs-section { margin-top: 25px; width: 100%; max-width: 700px; }";
    // Style log title: cyan small text
    html += ".logs-title { font-size: 12px; color: #00aaff; margin-bottom: 8px; letter-spacing: 1px; }";
    // Style monitor: scrollable log display area
    html += ".monitor { background: #0a0a0a; border: 2px solid #00ff00; padding: 12px; height: 140px; overflow-y: auto; font-size: 10px; line-height: 1.7; }";
    // Style log lines: with left border indicator
    html += ".log-line { margin: 2px 0; color: #00ff00; padding: 2px 5px; border-left: 2px solid transparent; }";
    // Style verified log lines: green left border
    html += ".log-line.verified { border-left-color: #00ff00; }";
    // Style timestamp in log: cyan color
    html += ".log-line .time { color: #00aaff; }";
    // Style key in log: yellow color
    html += ".log-line .key { color: #ffff00; font-weight: bold; }";
    // Style verification icon: green checkmark
    html += ".log-line .verify-icon { color: #00ff00; }";
    // Style footer: border top and small text
    html += ".footer { background: #0a0a0a; padding: 10px; border-top: 2px solid #00ff00; text-align: center; font-size: 11px; color: #00aaff; }";
    // End CSS styling section
    html += "</style></head><body>";
    // Start main container div
    html += "<div class='container'>";
    // Add header with title
    html += "<div class='header'><h1>⌨️ MAC → ESP32 → MAC</h1></div>";
    
    // Start content area
    html += "<div class='content'>";
    // Start key display section
    html += "<div class='key-display'>";
    // Add key label
    html += "<div class='key-label'>CURRENT KEY</div>";
    // Add key display box (starts as IDLE)
    html += "<div class='key-box idle' id='keyBox'>IDLE</div>";
    // Close key display section
    html += "</div>";
    
    // Start button section
    html += "<div class='buttons-section'>";
    // Add button section label
    html += "<div class='buttons-label'>SEND KEY TO ESP32</div>";
    // Start button row
    html += "<div class='button-row'>";
    // Add W button (onclick calls sendKey function)
    html += "<button class='btn' onclick='sendKey(\"w\")'>W</button>";
    // Add A button
    html += "<button class='btn' onclick='sendKey(\"a\")'>A</button>";
    // Add S button
    html += "<button class='btn' onclick='sendKey(\"s\")'>S</button>";
    // Add D button
    html += "<button class='btn' onclick='sendKey(\"d\")'>D</button>";
    // Close button row
    html += "</div>";
    
    // Start verification box
    html += "<div class='verification-box'>";
    // Add verification label
    html += "<div class='verification-label'>ESP32 VERIFICATION:</div>";
    // Add verification text (updates dynamically)
    html += "<div class='verification-text' id='verificationText'>Waiting for key press...</div>";
    // Close verification box
    html += "</div>";
    // Close button section
    html += "</div>";
    
    // Start log section
    html += "<div class='logs-section'>";
    // Add log section title
    html += "<div class='logs-title'>KEY PRESS LOG (MAC ↔ ESP32 ↔ MAC)</div>";
    // Add log display area (updated by JavaScript)
    html += "<div class='monitor' id='monitor'></div>";
    // Close log section
    html += "</div>";
    // Close content area
    html += "</div>";
    
    // Add footer explaining the WiFi communication flow
    html += "<div class='footer'>WiFi Communication: MAC sends key → ESP32 receives & verifies → MAC receives confirmation</div>";
    // Close main container
    html += "</div>";
    
    // ===== JAVASCRIPT CODE SECTION =====
    // Start JavaScript section
    html += "<script>";
    // Variable to track the last displayed key
    html += "let lastKey = '';";
    // Variable to track log length for update detection
    html += "let lastLogLength = 0;";
    // Variable to track if verification is pending
    html += "let verificationPending = false;";
    
    // Function to send a key to ESP32
    html += "function sendKey(key) {";
    // Set verification pending flag
    html += "  verificationPending = true;";
    // Show "sending" message with loading icon
    html += "  document.getElementById('verificationText').innerText = '⏳ Sending ' + key.toUpperCase() + ' to ESP32...';";
    // Reset text color to default (orange)
    html += "  document.getElementById('verificationText').className = 'verification-text';";
    // Blank line for readability
    html += "  ";
    // Fetch (send POST request) to /keypress endpoint
    html += "  fetch('/keypress', {";
    // Specify POST method
    html += "    method: 'POST',";
    // Set content type header
    html += "    headers: {'Content-Type': 'application/json'},";
    // Send key as JSON in request body
    html += "    body: JSON.stringify({key: key})";
    // Close fetch options
    html += "  })";
    // Parse response as JSON
    html += "  .then(r => r.json())";
    // Handle successful response
    html += "  .then(data => {";
    // Update verification text with ESP32 response
    html += "    document.getElementById('verificationText').innerText = '✓ ESP32: ' + data.status;";
    // Change text color to green (success)
    html += "    document.getElementById('verificationText').className = 'verification-text success';";
    // Log response to browser console
    html += "    console.log('Verification received from ESP32:', data);";
    // Clear pending flag
    html += "    verificationPending = false;";
    // Close success handler
    html += "  })";
    // Handle errors
    html += "  .catch(e => {";
    // Show error message
    html += "    document.getElementById('verificationText').innerText = '✗ Error: ' + e;";
    // Keep text color as default (orange)
    html += "    document.getElementById('verificationText').className = 'verification-text';";
    // Log error to console
    html += "    console.error('Error:', e);";
    // Clear pending flag
    html += "    verificationPending = false;";
    // Close error handler
    html += "  });";
    // Close sendKey function
    html += "}";
    
    // Function to update current key status
    html += "function updateStatus() {";
    // Fetch current data from ESP32
    html += "  fetch('/data')";
    // Parse response as JSON
    html += "    .then(r => r.json())";
    // Handle response data
    html += "    .then(data => {";
    // Get reference to key display box
    html += "      let keyBox = document.getElementById('keyBox');";
    // Get current active key from data
    html += "      let currentKey = data.activeKey || '';";
    // Check if key changed
    html += "      if (currentKey && currentKey !== lastKey) {";
    // Update box text with new key
    html += "        keyBox.innerText = currentKey.toUpperCase();";
    // Change class to active state (yellow)
    html += "        keyBox.className = 'key-box';";
    // Remember current key
    html += "        lastKey = currentKey;";
    // Close key changed check
    html += "      } else if (!currentKey && lastKey) {";
    // If key released, show IDLE
    html += "        keyBox.innerText = 'IDLE';";
    // Change class back to idle state (green)
    html += "        keyBox.className = 'key-box idle';";
    // Clear last key variable
    html += "        lastKey = '';";
    // Close key released check
    html += "      }";
    // Close data handler
    html += "    })";
    // Handle errors in status update
    html += "    .catch(e => console.error('Status update error:', e));";
    // Close updateStatus function
    html += "}";
    
    // Function to update key press log display
    html += "function updateLogs() {";
    // Fetch log data from ESP32
    html += "  fetch('/logs')";
    // Parse response as JSON
    html += "    .then(r => r.json())";
    // Handle response data
    html += "    .then(data => {";
    // Get reference to log display area
    html += "      let monitor = document.getElementById('monitor');";
    // Get logs string from response
    html += "      let logs = data.logs.trim();";
    // Check if logs were updated
    html += "      if (logs.length > lastLogLength) {";
    // Split logs into individual lines
    html += "        let lines = logs.split('\\n').filter(l => l.length > 0);";
    // Get only the last 10 log lines
    html += "        let recentLines = lines.slice(-10);";
    // Clear previous log display
    html += "        monitor.innerHTML = '';";
    // Loop through each recent log line
    html += "        recentLines.forEach(line => {";
    // Create new div for log entry
    html += "          let div = document.createElement('div');";
    // Set verified class for styling
    html += "          div.className = 'log-line verified';";
    // Regex to extract time and key from log format
    html += "          let match = line.match(/\\[(.*?)\\] KEY PRESSED: (.*?) activated/);";
    // Check if regex matched successfully
    html += "          if (match) {";
    // Extract timestamp
    html += "            let time = match[1];";
    // Extract key and convert to uppercase
    html += "            let key = match[2].toUpperCase();";
    // Build HTML with formatted log entry
    html += "            div.innerHTML = '<span class=\"time\">[' + time + ']</span> <span class=\"key\">' + key + '</span> activated <span class=\"verify-icon\">✓</span>';";
    // Close regex match check
    html += "          }";
    // Add log entry div to monitor
    html += "          monitor.appendChild(div);";
    // Close forEach loop
    html += "        });";
    // Auto-scroll to bottom of log
    html += "        monitor.scrollTop = monitor.scrollHeight;";
    // Update log length tracker
    html += "        lastLogLength = logs.length;";
    // Close update check
    html += "      }";
    // Close data handler
    html += "    })";
    // Handle errors in log update
    html += "    .catch(e => console.error('Log update error:', e));";
    // Close updateLogs function
    html += "}";
    
    // Call updateStatus every 50 milliseconds for real-time display
    html += "setInterval(updateStatus, 50);";
    // Call updateLogs every 100 milliseconds for real-time log updates
    html += "setInterval(updateLogs, 100);";
    // End JavaScript section
    html += "</script>";
    // Close HTML body
    html += "</body></html>";
    
    // Send complete HTML page to client
    server.send(200, "text/html", html);
  });
  
  // Start the web server to begin listening for client requests
  server.begin();
  // Print success message with the web server URL
  Serial.println("Web server started on http://192.168.4.1");
}

// ===== PREPARE SENSOR DATA FUNCTION =====
String prepareSensorData() {
  // Check if active key should be reset due to timeout
  if (millis() - lastKeyTime > keyResetTime && activeKey.length() > 0) {
    // Clear active key after 200ms of no new input
    activeKey = "";
  }
  
  // Start building JSON payload string
  String json = "{";
  
  // Add custom identifier field
  json += "\"KAYRA\":\"ESP32-S3-Data\",";
  
  // Add current timestamp in milliseconds
  json += "\"timestamp\":" + String(millis()) + ",";
  
  // Add keyboard input status (check if key is active)
  if (activeKey.length() > 0) {
    // If key is active, add the key character
    json += "\"activeKey\":\"" + activeKey + "\",";
    // Add key status message
    json += "\"keyStatus\":\"" + activeKey + " activated\",";
  } else {
    // If no key active, empty key field
    json += "\"activeKey\":\"\",";
    // Set status to idle
    json += "\"keyStatus\":\"idle\",";
  }
  
  // Add ESP32 heap (RAM) information
  // Add free heap memory available
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  // Add total heap memory size
  json += "\"totalHeap\":" + String(ESP.getHeapSize()) + ",";
  // Add heap usage percentage
  json += "\"heapUsagePercent\":" + String((100 * (ESP.getHeapSize() - ESP.getFreeHeap())) / ESP.getHeapSize()) + ",";
  
  // Add temperature reading from internal sensor
  json += "\"temperature\":" + String(temperatureRead(), 2) + ",";
  
  // Add WiFi connection information
  json += "\"connectedClients\":" + String(WiFi.softAPgetStationNum()) + ",";
  
  // Add chip identifier (last field, no trailing comma)
  json += "\"chip\":\"ESP32-S3-N16R8\"";
  
  // Close JSON object
  json += "}";
  
  // Return the complete JSON string
  return json;
}
