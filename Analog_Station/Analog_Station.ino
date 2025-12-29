/* THE ANALOG STATION! - An ESP32-C3 based Audio & Environmental Monitor
 * Created in 2025 by Dave Elfving in San Francisco for teaching and learning.
 * 
 * A dual-mode monitoring station combining vintage analog aesthetics with modern digital processing.
 * 
 * MODES:
 * 1. AUDIO SPECTRUM ANALYZER
 *    - Real-time FFT analysis of audio frequencies (Low/Mid/High).
 * 
 * 2. ENVIRONMENTAL MONITOR (BME280)
 *    - Displays Temperature, Humidity, and Barometric Pressure.
 * 
 * CONTROLS:
 * - Toggle Switch: Switch between Audio and Sensor modes.
 * - Web Interface: Calibrate sensor readings and audio scaling via a built-in web server.
 */

// Required libraries
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <arduinoFFT.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>

// Hardware pin assignments
#define MIC_PIN 4           
#define TOGGLE_PIN 3        
#define METER_HIGH 6        
#define METER_MID 7         
#define METER_LOW 10        
#define LED_HIGH 1          
#define LED_MID 0           
#define LED_LOW 5           
#define SDA_PIN 20          
#define SCL_PIN 21          
#define BUILTIN_LED_C3 8    

// Audio processing configuration
#define SAMPLES 128         
#define SAMPLE_RATE 4000    
#define PWM_FREQ 500        
#define PWM_RESOLUTION 8    

// Global objects and variables
Adafruit_BME280 bme;
Preferences preferences;
WebServer server(80); 
char ssid[32]; // Global to store unique network name
bool sensorMode = false;
bool lastToggleState = HIGH;

// Forward declarations
void setupWiFi();
void handleRoot();

// FFT audio analysis
float vReal[SAMPLES];
float vImag[SAMPLES];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLE_RATE);

// --- CALIBRATION VARIABLES ---
// Data Calibration: Adjusts the reported value (Serial & Web) AND the meter.
// Stored in Fahrenheit for Temp.
float calTemp = 0.0;
float calPress = 0.0;
float calHum = 0.0;

// Visual Nudge: Adjusts ONLY the analog meter needle position.
// Stored in Fahrenheit for Temp.
float nudgeTemp = 0.0;
float nudgePress = 0.0;
float nudgeHum = 0.0;
bool calibrationValid = false;

// Audio Calibration (Noise Floor & Scaling)
float noiseFloorL = 800.0;
float noiseFloorM = 800.0;
float noiseFloorH = 800.0;

// Audio Scaling Factors (adjustable via web interface)
float scalingL = 45.0;
float scalingM = 20.0;
float scalingH = 5.0;

// Calibration adjustment mode variables (Sensor mode)
bool calibrationAdjustMode = false;

// Auto-save variables
unsigned long lastAdjustmentTime = 0;
bool calibrationChanged = false;
#define AUTO_SAVE_DELAY 3000 

// Setup - runs once at startup
void setup() {
  Serial.begin(115200);
  
  // Wait for serial to connect (needed for native USB boards like C3)
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000);
  delay(1000);

  Serial.println("\n\n--- Analog Station Booting ---");

  // Initialize I2C and BME280 sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!bme.begin(0x76, &Wire)) {
    Serial.println("BME280 initialization failed - check wiring");
  } else {
    Serial.println("BME280 initialized successfully on ESP32-C3");
  }

  // Configure GPIO pins
  pinMode(TOGGLE_PIN, INPUT_PULLUP);
  pinMode(LED_LOW, OUTPUT);
  pinMode(LED_MID, OUTPUT);
  pinMode(LED_HIGH, OUTPUT);
  pinMode(BUILTIN_LED_C3, OUTPUT);

  // Configure ADC for microphone (12-bit, 0-3.3V)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Configure PWM for analog meters and onboard LED (500Hz, 8-bit)
  ledcAttach(METER_LOW, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(METER_MID, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(METER_HIGH, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(BUILTIN_LED_C3, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(BUILTIN_LED_C3, 127);

  // Turn on LEDs
  digitalWrite(LED_LOW, HIGH);
  digitalWrite(LED_MID, HIGH);
  digitalWrite(LED_HIGH, HIGH);

  // Load saved settings from non-volatile storage
  preferences.begin("analog-station", false);
  loadCalibration();
  preferences.end(); 
  
  // Initialize WiFi and Web Server
  setupWiFi();

  Serial.println("ESP32-C3 Controller Ready");
  Serial.printf("Connect to WiFi: '%s' (IP: %s)\n", ssid, WiFi.softAPIP().toString().c_str());
  
  delay(2000);
}

// Main loop - runs continuously
void loop() {
  server.handleClient(); // Handle web requests
  
  // Check toggle switch for mode changes
  bool currentToggleState = digitalRead(TOGGLE_PIN);
  if (currentToggleState != lastToggleState) {
    sensorMode = (currentToggleState == LOW);
    lastToggleState = currentToggleState;
    Serial.printf("Mode: %s (via toggle switch)\n", sensorMode ? "Sensor" : "Audio");
  }
  
  // Process audio or sensor mode
  sensorMode ? processSensorMode() : processAudioMode();
  
  // Auto-save calibration if changed and idle
  if (calibrationChanged && (millis() - lastAdjustmentTime > AUTO_SAVE_DELAY)) {
    saveCalibration();
    calibrationChanged = false;
    Serial.println("(Auto-saved changes)");
  }
  
  delay(20);
}

// Sensor mode - reads BME280 and drives meters (updates every 10 seconds)
void processSensorMode() {
  static unsigned long lastSensorRead = 0;
  unsigned long currentTime = millis();
  
  // Rate limit to 10 seconds (or 0.5 seconds in adjustment mode for live feedback)
  unsigned long readInterval = calibrationAdjustMode ? 500 : 10000;
  if (currentTime - lastSensorRead < readInterval) {
    return;
  }
  lastSensorRead = currentTime;
  
  // Read sensor values
  float tempC = bme.readTemperature();
  float pressureHPa = bme.readPressure() / 100.0F;
  float humidity = bme.readHumidity();
  
  // Validate readings
  if (isnan(tempC) || isnan(pressureHPa) || isnan(humidity)) {
    Serial.println("BME280 read failed - check wiring");
    return;
  }

  // Convert Temp to Fahrenheit
  float tempF = (tempC * 1.8f) + 32.0f;

  // 1. Apply DATA Calibration (Affects Serial & Meter)
  float finalTemp = tempF + calTemp;
  float finalPress = pressureHPa + calPress;
  float finalHum = humidity + calHum;

  // 2. Apply VISUAL Nudge (Affects Meter ONLY)
  float meterTemp = finalTemp + nudgeTemp;
  float meterPress = finalPress + nudgePress;
  float meterHum = finalHum + nudgeHum;

  // Scale to meter range (0-255)
  // Temp Range: 32F - 100F
  int pwmTemp = constrain((int)((meterTemp - 32.0f) * (255.0f / 68.0f)), 0, 255);
  
  // Pressure Range: 980 - 1030 hPa
  int pwmPress = constrain((int)((meterPress - 980.0f) * (255.0f / 50.0f)), 0, 255);
  
  // Humidity Range: 0 - 100%
  int pwmHum = constrain((int)(meterHum * (255.0f / 100.0f)), 0, 255);

  // Drive meters
  ledcWrite(METER_LOW, pwmTemp);
  ledcWrite(METER_MID, pwmHum);
  ledcWrite(METER_HIGH, pwmPress);
  ledcWrite(BUILTIN_LED_C3, 127);

  // Display readings (Data Calibrated, NOT Nudged)
  if (!calibrationAdjustMode) {
    Serial.printf("T: %.1f°F (PWM %d) | H: %.1f%% (PWM %d) | P: %.1fhPa (PWM %d)\n",
                  finalTemp, pwmTemp, finalHum, pwmHum, finalPress, pwmPress);
  }
}

// Audio mode - Simplified FFT-based spectrum analysis
void processAudioMode() {
  unsigned long sampling_period_us = round(1000000.0 * (1.0 / SAMPLE_RATE));
  unsigned long newTime;

  // 1. Sample Audio
  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) { /* Wait */ }
  }

  // 2. Process FFT
  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  // 3. Calculate Band Levels (Average of magnitudes)
  // Sample Rate: 4000Hz, Samples: 128 -> Bin size: ~31.25Hz
  // Max Frequency: 2000Hz (Nyquist)
  float levelLow = 0, levelMid = 0, levelHigh = 0;
  int countLow = 0, countMid = 0, countHigh = 0;

  // Low: ~90Hz - 250Hz (Bins 3-8)
  for (int i = 3; i <= 8; i++) {
    levelLow += vReal[i];
    countLow++;
  }
  
  // Mid: ~250Hz - 800Hz (Bins 9-25)
  for (int i = 9; i <= 25; i++) {
    levelMid += vReal[i];
    countMid++;
  }

  // High: ~800Hz - 2000Hz (Bins 26-63)
  for (int i = 26; i < 64; i++) {
    levelHigh += vReal[i];
    countHigh++;
  }
  
  // Normalize (Average) to prevent wider bands from dominating
  if (countLow > 0) levelLow /= countLow;
  if (countMid > 0) levelMid /= countMid;
  if (countHigh > 0) levelHigh /= countHigh;

  // 5. Compute Meter PWM (0-255) with Smoothing
  static float smoothLow = 0, smoothMid = 0, smoothHigh = 0;
  const float SMOOTHING_L = 0.85; // Higher = Slower/Smoother for Bass
  const float SMOOTHING_MH = 0.7; // Standard for Mid/High

  // Helper lambda for processing
  auto processBand = [](float level, float floor, float scale, float &smoothed, float smoothFactor) -> int {
    // Hard Noise Gate
    if (level < floor) {
      level = 0;
    } else {
      level = (level - floor);
    }
    
    // Smoothing
    smoothed = (smoothed * smoothFactor) + (level * (1.0 - smoothFactor));
    
    // Scaling
    int output = (int)(smoothed / scale);
    return constrain(output, 0, 255);
  };

  int pwmL = processBand(levelLow, noiseFloorL, scalingL, smoothLow, SMOOTHING_L);
  int pwmM = processBand(levelMid, noiseFloorM, scalingM, smoothMid, SMOOTHING_MH);
  int pwmH = processBand(levelHigh, noiseFloorH, scalingH, smoothHigh, SMOOTHING_MH);

  // 6. Output to Meters
  ledcWrite(METER_LOW, pwmL);
  ledcWrite(METER_MID, pwmM);
  ledcWrite(METER_HIGH, pwmH);
  
  // Brightness of built-in LED tied to mid level
  ledcWrite(BUILTIN_LED_C3, 255 - pwmM);

  // Debug Output (Visible in Serial Monitor)
  static int skip = 0;
  if (skip++ > 10) { // Slow down prints
    skip = 0;
    Serial.printf("RAW: L=%.0f M=%.0f H=%.0f  ->  PWM: %d %d %d\n", 
      levelLow, levelMid, levelHigh, pwmL, pwmM, pwmH);
  }
}

// Sensor calibration functions
void loadCalibration() {
  calibrationValid = preferences.getBool("calValid", false);
  
  noiseFloorL = preferences.getFloat("noiseL", 500.0);
  noiseFloorM = preferences.getFloat("noiseM", 500.0);
  noiseFloorH = preferences.getFloat("noiseH", 500.0);

  scalingL = preferences.getFloat("scaleL", 45.0);
  scalingM = preferences.getFloat("scaleM", 20.0);
  scalingH = preferences.getFloat("scaleH", 5.0);

  if (calibrationValid) {
    // Load Data Offsets
    calTemp = preferences.getFloat("calTemp", 0.0);
    calPress = preferences.getFloat("calPress", 0.0);
    calHum = preferences.getFloat("calHum", 0.0);

    // Load Visual Nudges
    nudgeTemp = preferences.getFloat("nudgeTemp", 0.0);
    nudgePress = preferences.getFloat("nudgePress", 0.0);
    nudgeHum = preferences.getFloat("nudgeHum", 0.0);
    
    Serial.println("\nLoaded calibration:");
    Serial.printf("  Temp: Offset=%.2fF, Nudge=%.2fF\n", calTemp, nudgeTemp);
  } else {
    Serial.println("\nNo sensor calibration data found.");
  }
  Serial.printf("Loaded Noise Floor: L=%.0f M=%.0f H=%.0f\n", noiseFloorL, noiseFloorM, noiseFloorH);
  Serial.printf("Loaded Scaling: L=%.1f M=%.1f H=%.1f\n", scalingL, scalingM, scalingH);
}

void saveCalibration() {
  preferences.begin("analog-station", false);
  
  preferences.putFloat("calTemp", calTemp);
  preferences.putFloat("calPress", calPress);
  preferences.putFloat("calHum", calHum);

  preferences.putFloat("nudgeTemp", nudgeTemp);
  preferences.putFloat("nudgePress", nudgePress);
  preferences.putFloat("nudgeHum", nudgeHum);
  
  preferences.putFloat("scaleL", scalingL);
  preferences.putFloat("scaleM", scalingM);
  preferences.putFloat("scaleH", scalingH);

  preferences.putFloat("noiseL", noiseFloorL);
  preferences.putFloat("noiseM", noiseFloorM);
  preferences.putFloat("noiseH", noiseFloorH);
  
  preferences.putBool("calValid", true);
  preferences.end();
  calibrationValid = true;
  Serial.println("Calibration saved.");
}

// ==========================================
// Wi-Fi & Web Server Functions
// ==========================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Analog Station Config</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin:0; padding:20px; background:#222; color:#eee; }
    h2 { color: #ffcc00; }
    .card { background: #333; padding: 20px; margin: 10px auto; max-width: 500px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.3); }
    .row { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; padding: 5px; background: #444; border-radius: 4px; }
    .label { width: 60px; font-weight: bold; text-align: left; }
    .val { width: 80px; color: #ffcc00; font-family: monospace; }
    input[type=number] { width: 70px; padding: 5px; border: 1px solid #555; background: #222; color: #fff; border-radius: 4px; }
    button { cursor: pointer; border: none; border-radius: 4px; padding: 5px 10px; font-weight: bold; }
    .btn-cal { background: #007acc; color: white; }
    .btn-nudge { background: #555; color: white; width: 30px; }
    .btn-nudge:hover { background: #777; }
    .section-title { text-align: left; color: #aaa; border-bottom: 1px solid #555; margin-bottom: 10px; padding-bottom: 5px; }
  </style>
  <script>
    function nudge(id, dir) {
      fetch('/nudge?id=' + id + '&dir=' + dir)
        .then(res => res.text())
        .then(val => { 
          console.log('Nudged', id, val);
          if(id.startsWith('scale') || id.startsWith('noise')) {
             document.getElementById(id).value = val;
          }
        });
    }
    function calibrate(id) {
      let refVal = document.getElementById('ref_' + id).value;
      if(!refVal) return;
      fetch('/calibrate?id=' + id + '&value=' + refVal)
        .then(res => res.text())
        .then(val => { 
          document.getElementById('cur_' + id).innerText = val; 
          alert('Calibrated ' + id + ' to ' + val);
        });
    }
  </script>
</head>
<body>
  <h2>Analog Station</h2>
  
  <div class="card">
    <div class="section-title">SENSOR CALIBRATION</div>
    <div style="font-size: 0.8em; color: #aaa; margin-bottom: 10px;">
      <b>Ref:</b> Enter actual value to calibrate data.<br>
      <b>Nudge:</b> Adjust meter needle only.
    </div>

    <!-- Temperature -->
    <div class="row">
      <span class="label">Temp</span>
      <span class="val" id="cur_temp">%TEMP% F</span>
      <input type="number" id="ref_temp" placeholder="Ref F" step="0.1">
      <button class="btn-cal" onclick="calibrate('temp')">Set</button>
      <div style="display:flex; flex-direction:column; gap:2px;">
        <button class="btn-nudge" onclick="nudge('temp', 1)">+</button>
        <button class="btn-nudge" onclick="nudge('temp', -1)">-</button>
      </div>
    </div>

    <!-- Pressure -->
    <div class="row">
      <span class="label">Press</span>
      <span class="val" id="cur_press">%PRESS% hPa</span>
      <input type="number" id="ref_press" placeholder="Ref hPa" step="0.1">
      <button class="btn-cal" onclick="calibrate('press')">Set</button>
      <div style="display:flex; flex-direction:column; gap:2px;">
        <button class="btn-nudge" onclick="nudge('press', 1)">+</button>
        <button class="btn-nudge" onclick="nudge('press', -1)">-</button>
      </div>
    </div>

    <!-- Humidity -->
    <div class="row">
      <span class="label">Hum</span>
      <span class="val" id="cur_hum">%HUM% %</span>
      <input type="number" id="ref_hum" placeholder="Ref %" step="0.1">
      <button class="btn-cal" onclick="calibrate('hum')">Set</button>
      <div style="display:flex; flex-direction:column; gap:2px;">
        <button class="btn-nudge" onclick="nudge('hum', 1)">+</button>
        <button class="btn-nudge" onclick="nudge('hum', -1)">-</button>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="section-title">AUDIO SETTINGS</div>
    
    <div style="font-size: 0.8em; color: #aaa; margin-bottom: 5px; margin-top: 10px;">
      <b>SCALING:</b> Higher = Less Sensitive
    </div>
    <form action="/save_audio" method="POST">
      <div class="row">
        <span class="label">Low</span>
        <input type="number" step="0.1" name="scaleL" id="scaleL" value="%SCALEL%">
        <div style="display:flex; flex-direction:column; gap:2px;">
          <button type="button" class="btn-nudge" onclick="nudge('scaleL', 1)">+</button>
          <button type="button" class="btn-nudge" onclick="nudge('scaleL', -1)">-</button>
        </div>
      </div>
      <div class="row">
        <span class="label">Mid</span>
        <input type="number" step="0.1" name="scaleM" id="scaleM" value="%SCALEM%">
        <div style="display:flex; flex-direction:column; gap:2px;">
          <button type="button" class="btn-nudge" onclick="nudge('scaleM', 1)">+</button>
          <button type="button" class="btn-nudge" onclick="nudge('scaleM', -1)">-</button>
        </div>
      </div>
      <div class="row">
        <span class="label">High</span>
        <input type="number" step="0.1" name="scaleH" id="scaleH" value="%SCALEH%">
        <div style="display:flex; flex-direction:column; gap:2px;">
          <button type="button" class="btn-nudge" onclick="nudge('scaleH', 1)">+</button>
          <button type="button" class="btn-nudge" onclick="nudge('scaleH', -1)">-</button>
        </div>
      </div>

      <input type="submit" value="Save Audio Settings" style="width:100%; background:#ffcc00; border:none; padding:10px; margin-top:10px; cursor:pointer;">
    </form>
  </div>
  
  <p><a href="/" style="color:#aaa">Refresh Page</a></p>
</body>
</html>
)rawliteral";

void handleRoot() {
  // Get fresh readings for display
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0F;
  float h = bme.readHumidity();
  
  float curTemp = (t * 1.8f + 32.0f) + calTemp;
  float curPress = p + calPress;
  float curHum = h + calHum;

  String s = index_html;
  s.replace("%TEMP%", String(curTemp, 1));
  s.replace("%PRESS%", String(curPress, 1));
  s.replace("%HUM%", String(curHum, 1));
  s.replace("%SCALEL%", String(scalingL));
  s.replace("%SCALEM%", String(scalingM));
  s.replace("%SCALEH%", String(scalingH));
  server.send(200, "text/html", s);
}

void handleNudge() {
  if (!server.hasArg("id") || !server.hasArg("dir")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }
  
  String id = server.arg("id");
  int dir = server.arg("dir").toInt(); 
  float newVal = 0;
  
  if (id == "temp") nudgeTemp += (dir * 0.5); // 0.5 F steps
  else if (id == "press") nudgePress += (dir * 0.5);
  else if (id == "hum") nudgeHum += (dir * 0.5);
  
  // Audio Scaling Nudges
  else if (id == "scaleL") { scalingL += (dir * 1.0); newVal = scalingL; }
  else if (id == "scaleM") { scalingM += (dir * 1.0); newVal = scalingM; }
  else if (id == "scaleH") { scalingH += (dir * 0.5); newVal = scalingH; }

  calibrationAdjustMode = true;
  calibrationChanged = true;
  lastAdjustmentTime = millis();
  
  server.send(200, "text/plain", String(newVal));
}

void handleCalibrate() {
  if (!server.hasArg("id") || !server.hasArg("value")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }

  String id = server.arg("id");
  float target = server.arg("value").toFloat();
  float newVal = target;

  // Read current raw values to calculate offset
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0F;
  float h = bme.readHumidity();

  if (id == "temp") {
    float rawF = (t * 1.8f) + 32.0f;
    calTemp = target - rawF;
  } else if (id == "press") {
    calPress = target - p;
  } else if (id == "hum") {
    calHum = target - h;
  }

  calibrationChanged = true;
  lastAdjustmentTime = millis();
  
  server.send(200, "text/plain", String(newVal, 1));
}

void handleSaveAudio() {
  if (server.hasArg("scaleL")) scalingL = server.arg("scaleL").toFloat();
  if (server.hasArg("scaleM")) scalingM = server.arg("scaleM").toFloat();
  if (server.hasArg("scaleH")) scalingH = server.arg("scaleH").toFloat();
  
  saveCalibration();
  server.sendHeader("Location", "/");
  server.send(303);
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  
  uint8_t mac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
  
  if (ret != ESP_OK) {
    Serial.println("Failed to read MAC address, using random fallback.");
    randomSeed(analogRead(MIC_PIN));
    mac[4] = random(255);
    mac[5] = random(255);
  }

  WiFi.mode(WIFI_AP);
  
  // Create SSID using the last 2 bytes of the STATION MAC
  // (These are unique to each chip)
  snprintf(ssid, sizeof(ssid), "Analog Station %02X%02X", mac[4], mac[5]);

  WiFi.softAP(ssid, ""); // Open network
  IPAddress IP = WiFi.softAPIP();
  
  Serial.print("AP Started: ");
  Serial.println(ssid);
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.on("/", handleRoot);
  server.on("/nudge", handleNudge);
  server.on("/calibrate", handleCalibrate);
  server.on("/save_audio", HTTP_POST, handleSaveAudio);
  server.begin();
  Serial.println("HTTP server started");
}
