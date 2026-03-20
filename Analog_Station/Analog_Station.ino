/* THE ANALOG STATION - An audio & environmental monitor built with an ESP32-C3.
Created in San Francisco for teaching and learning. 2025 */

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
#include <LittleFS.h>

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

// System Mode State
// "normal" (Respects Toggle Switch & sensorMode) or "calibrate" (Respects calTab)
String systemMode = "normal";
String calTab = "sensor";

// Forward declarations
void setupWiFi();
void handleRoot();
void calibrateNoiseFloor();

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

// Audio Calibration (Noise Floor & Gain)
float noiseFloorL = 500.0;
float noiseFloorM = 500.0;
float noiseFloorH = 500.0;

// Audio Sensitivity (1-100) - controls Gain
int sensL = 17;
int sensM = 23;
int sensH = 49;

// Current Audio Levels (for Web Interface)
int webAudioL = 0;
int webAudioM = 0;
int webAudioH = 0;

// Audio Band Config (BINS)
// Bins are roughly 31.25Hz apart (4000Hz / 128)
int bandSplitLowMid = 11;
int bandSplitMidHigh = 49;

// Calibration adjustment mode variables (Sensor mode)
bool calibrationAdjustMode = false;

// Auto-save variables
unsigned long lastAdjustmentTime = 0;
bool calibrationChanged = false;
#define AUTO_SAVE_DELAY 3000 

unsigned long lastSensorRead = 0; // Global for instant switching

// Setup - runs once at startup
void setup() {
  Serial.begin(115200);
  
  // Wait shortly for serial, but don't block long if not connected
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000); 
  delay(1000);

  Serial.println("\n\n--- Analog Station Booting ---");

  // Initialize I2C and BME280 sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Checking BME280 status...");
  unsigned status = bme.begin(0x76, &Wire);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
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
  loadCalibration();
  
  // Initialize File System
  if(!LittleFS.begin(true)){
    Serial.println("An Error has occurred while mounting LittleFS");
  }

  // Initialize WiFi and Web Server
  setupWiFi();

  Serial.println("ESP32-C3 Controller Ready");
  Serial.printf("Connect to WiFi: '%s' (IP: %s)\n", ssid, WiFi.softAPIP().toString().c_str());
}

// Main loop - runs continuously
void loop() {
  server.handleClient(); // Handle web requests
  
  // Check toggle switch for mode changes
  bool currentToggleState = digitalRead(TOGGLE_PIN);
  
  // Logic: Detect CHANGE in switch position (Edge Detection).
  // This allows the web app to set the mode freely, but if the user physically
  // flips the switch, the device responds immediately.
  if (currentToggleState != lastToggleState) {
      if (systemMode == "normal") {
          sensorMode = (currentToggleState == LOW);
      }
      lastToggleState = currentToggleState;
  }
  
  bool activeSensorMode = sensorMode;
  
  // Logic: "The device should reflect which ever tab is active in calibration mode"
  if (systemMode == "calibrate") {
      activeSensorMode = (calTab == "sensor");
  } else {
      // Normal mode uses the sensorMode (controlled by switch/web)
      activeSensorMode = sensorMode;
  }
  
  // Process audio or sensor mode
  activeSensorMode ? processSensorMode() : processAudioMode();
  
  // Auto-save calibration if changed and idle
  if (calibrationChanged && (millis() - lastAdjustmentTime > AUTO_SAVE_DELAY)) {
    saveCalibration();
    calibrationChanged = false;
  }
  
  delay(20);
}

// Handle calibration
void calibrateNoiseFloor() {
  delay(1000);
  
  float maxL = 0, maxM = 0, maxH = 0;
  unsigned long start = millis();
  unsigned long sampling_period_us = round(1000000.0 * (1.0 / SAMPLE_RATE));
  
  while (millis() - start < 2000) {
    // Sample
    for (int i = 0; i < SAMPLES; i++) {
      unsigned long newTime = micros();
      vReal[i] = analogRead(MIC_PIN);
      vImag[i] = 0;
      while ((micros() - newTime) < sampling_period_us) { /* Wait */ }
    }
    
    // FFT
    FFT.dcRemoval();
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    
    // Sum Bands
    float l = 0, m = 0, h = 0;
    int cL = 0, cM = 0, cH = 0;
    
    for (int i = 3; i <= bandSplitLowMid; i++) { l += vReal[i]; cL++; }
    for (int i = bandSplitLowMid + 1; i <= bandSplitMidHigh; i++) { m += vReal[i]; cM++; }
    for (int i = bandSplitMidHigh + 1; i < 64; i++) { h += vReal[i]; cH++; }
    
    // Normalize
    if (cL > 0) l /= cL;
    if (cM > 0) m /= cM;
    if (cH > 0) h /= cH;
    
    if (l > maxL) maxL = l;
    if (m > maxM) maxM = m;
    if (h > maxH) maxH = h;
  }
  
  // Set noise floor to max measured value (no buffer)
  noiseFloorL = maxL;
  noiseFloorM = maxM;
  noiseFloorH = maxH;
  
  // Ensure minimums (Lowered for averaged values)
  if (noiseFloorL < 100) noiseFloorL = 100;
  if (noiseFloorM < 100) noiseFloorM = 100;
  if (noiseFloorH < 100) noiseFloorH = 100;
  
  saveCalibration();
  
  sensorMode = false;
}

// 
// Sensor mode - reads BME280 and drives meters (updates every 10 seconds)
void processSensorMode() {
  unsigned long currentTime = millis();
  
  // Rate limit to 3 seconds (or 0.5 seconds in adjustment mode for live feedback)
  unsigned long readInterval = calibrationAdjustMode ? 500 : 3000;
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
  // Temp Range: 20F - 120F
  int pwmTemp = constrain((int)((meterTemp - 20.0f) * (255.0f / 100.0f)), 0, 255);
  
  // Pressure Range: 980 - 1030 hPa
  int pwmPress = constrain((int)((meterPress - 980.0f) * (255.0f / 50.0f)), 0, 255);
  
  // Humidity Range: 0 - 100%
  int pwmHum = constrain((int)(meterHum * (255.0f / 100.0f)), 0, 255);

  // Drive meters
  ledcWrite(METER_LOW, pwmTemp);
  ledcWrite(METER_MID, pwmHum);
  ledcWrite(METER_HIGH, pwmPress);
  ledcWrite(BUILTIN_LED_C3, 127);
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

  // 3. Calculate Band Levels (Max Peak Detection)
  // Max Frequency: 2000Hz (Nyquist)
  float levelLow = 0, levelMid = 0, levelHigh = 0;

  // Low Band
  for (int i = 3; i <= bandSplitLowMid; i++) {
    if (vReal[i] > levelLow) levelLow = vReal[i];
  }
  
  // Mid Band
  for (int i = bandSplitLowMid + 1; i <= bandSplitMidHigh; i++) {
     if (vReal[i] > levelMid) levelMid = vReal[i];
  }

  // High Band
  for (int i = bandSplitMidHigh + 1; i < 64; i++) {
     if (vReal[i] > levelHigh) levelHigh = vReal[i];
  }
  
  // 5. Compute Meter PWM (0-255) with Smoothing
  static float smoothLow = 0, smoothMid = 0, smoothHigh = 0;
  
  // Calculate Gain from Sensitivity (0.0001 to ~0.5)
  // Square curve gives better low-volume control
  // sens=50 -> 2500 * 0.00005 = 0.125
  // sens=100 -> 10000 * 0.00005 = 0.5
  const float GAIN_FACTOR = 0.00005; 
  float gainL = (sensL * sensL) * GAIN_FACTOR;
  float gainM = (sensM * sensM) * GAIN_FACTOR;
  float gainH = (sensH * sensH) * GAIN_FACTOR;

  const float SMOOTHING_L = 0.85; // Higher = Slower/Smoother for Bass
  const float SMOOTHING_MH = 0.7; // Standard for Mid/High

  // Helper lambda for processing
  auto processBand = [](float level, float floor, float gain, float &smoothed, float smoothFactor) -> int {
    // Hard Noise Gate
    if (level < floor) {
      level = 0;
    } else {
      level = (level - floor);
    }
    
    // Smoothing
    smoothed = (smoothed * smoothFactor) + (level * (1.0 - smoothFactor));
    
    // Gain
    int output = (int)(smoothed * gain);
    return constrain(output, 0, 255);
  };

  int pwmL = processBand(levelLow, noiseFloorL, gainL, smoothLow, SMOOTHING_L);
  int pwmM = processBand(levelMid, noiseFloorM, gainM, smoothMid, SMOOTHING_MH);
  int pwmH = processBand(levelHigh, noiseFloorH, gainH, smoothHigh, SMOOTHING_MH);

  // Update globals for web
  webAudioL = pwmL;
  webAudioM = pwmM;
  webAudioH = pwmH;

  // 6. Output to Meters
  ledcWrite(METER_LOW, pwmL);
  ledcWrite(METER_MID, pwmM);
  ledcWrite(METER_HIGH, pwmH);
  
  // Brightness of built-in LED tied to mid level
  ledcWrite(BUILTIN_LED_C3, 255 - pwmM);
}

// Sensor calibration functions
void loadCalibration() {
  preferences.begin("analog_stn", true);
  calibrationValid = preferences.getBool("calValid", false);
  
  noiseFloorL = preferences.getFloat("noiseL", 500.0);
  noiseFloorM = preferences.getFloat("noiseM", 500.0);
  noiseFloorH = preferences.getFloat("noiseH", 500.0);

  sensL = preferences.getInt("sensL", 17);
  sensM = preferences.getInt("sensM", 23);
  sensH = preferences.getInt("sensH", 49);

  bandSplitLowMid = preferences.getInt("split1", 11);
  bandSplitMidHigh = preferences.getInt("split2", 49);

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
  Serial.printf("Loaded Sensitivity: L=%d M=%d H=%d\n", sensL, sensM, sensH);
  preferences.end();
}

void saveCalibration() {
  preferences.begin("analog_stn", false);
  
  preferences.putFloat("calTemp", calTemp);
  preferences.putFloat("calPress", calPress);
  preferences.putFloat("calHum", calHum);

  preferences.putFloat("nudgeTemp", nudgeTemp);
  preferences.putFloat("nudgePress", nudgePress);
  preferences.putFloat("nudgeHum", nudgeHum);
  
  preferences.putInt("sensL", sensL);
  preferences.putInt("sensM", sensM);
  preferences.putInt("sensH", sensH);

  preferences.putInt("split1", bandSplitLowMid);
  preferences.putInt("split2", bandSplitMidHigh);

  preferences.putFloat("noiseL", noiseFloorL);
  preferences.putFloat("noiseM", noiseFloorM);
  preferences.putFloat("noiseH", noiseFloorH);
  
  preferences.putBool("calValid", true);
  preferences.end();
  calibrationValid = true;
}

// ==========================================
// Wi-Fi & Web Server Functions
// ==========================================

// ==========================================
// Wi-Fi & Web Server Functions
// ==========================================

void handleRoot() {
   File file = LittleFS.open("/index.html", "r");
   if (!file) {
      server.send(404, "text/plain", "File Not Found. Please upload LittleFS data.");
      return;
   }
   server.streamFile(file, "text/html");
   file.close();
}

void handleStatusJson() {
  float t = bme.readTemperature();
  float p = bme.readPressure() / 100.0F;
  float h = bme.readHumidity();
  
  float curTemp = (t * 1.8f + 32.0f) + calTemp;
  float curPress = p + calPress;
  float curHum = h + calHum;

  String json = "{";
  json += "\"temp\":" + String(curTemp, 1) + ",";
  json += "\"press\":" + String(curPress, 1) + ",";
  json += "\"hum\":" + String(curHum, 1) + ",";
  json += "\"sensL\":" + String(sensL) + ",";
  json += "\"sensM\":" + String(sensM) + ",";
  json += "\"sensH\":" + String(sensH) + ",";
  json += "\"nudgeTemp\":" + String(nudgeTemp, 1) + ",";
  json += "\"nudgePress\":" + String(nudgePress, 1) + ",";
  json += "\"nudgeHum\":" + String(nudgeHum, 1) + ",";
  
  // Include real-time audio levels
  json += "\"audioL\":" + String(webAudioL) + ",";
  json += "\"audioM\":" + String(webAudioM) + ",";
  json += "\"audioH\":" + String(webAudioH) + ",";
  
  // Send "calibrate" if that is the system mode, otherwise send "audio" or "sensor"
  String reportedMode = (systemMode == "calibrate") ? "calibrate" : (sensorMode ? "sensor" : "audio");
  
  json += "\"mode\":\"" + reportedMode + "\",";
  json += "\"calTab\":\"" + calTab + "\"";
  json += "}";
  
  server.send(200, "application/json", json);
}

void handleSetNudge() {
  if (!server.hasArg("id") || !server.hasArg("val")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }
  
  String id = server.arg("id");
  float val = server.arg("val").toFloat();
  
  if (id == "temp") nudgeTemp = val;
  else if (id == "press") nudgePress = val;
  else if (id == "hum") nudgeHum = val;
  
  calibrationAdjustMode = true;
  calibrationChanged = true;
  lastAdjustmentTime = millis();
  
  server.send(200, "text/plain", String(val));
}

void handleSwitchMode() {
  if (!server.hasArg("mode")) {
    server.send(400, "text/plain", "Missing mode arg");
    return;
  }
  
  String mode = server.arg("mode");
  if (mode == "sensor") {
    systemMode = "normal";
    sensorMode = true;
    calibrationAdjustMode = false;
    lastSensorRead = 0; // Force immediate update
    server.send(200, "text/plain", "Switched to Sensor Mode");
  } else if (mode == "audio") {
    systemMode = "normal";
    sensorMode = false;
    calibrationAdjustMode = false;
    server.send(200, "text/plain", "Switched to Audio Mode");
  } else if (mode == "calibrate") {
    systemMode = "calibrate";
    calibrationAdjustMode = true; // Use faster updates?
    server.send(200, "text/plain", "Switched to Calibration Mode");
  } else {
    server.send(400, "text/plain", "Invalid mode");
  }
}

void handleSetCalTab() {
  if (!server.hasArg("tab")) {
    server.send(400, "text/plain", "Missing tab arg");
    return;
  }
  String tab = server.arg("tab");
  if (tab == "audio" || tab == "sensor") {
      calTab = tab;
      server.send(200, "text/plain", "Tab Set");
  } else {
      server.send(400, "text/plain", "Invalid Tab");
  }
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

void handleSetAudio() {
  if (!server.hasArg("id") || !server.hasArg("val")) {
    server.send(400, "text/plain", "Missing args");
    return;
  }
  
  String id = server.arg("id");
  int val = server.arg("val").toInt();
  
  // Set Sensitivity Directly (1-100)
  if (id == "sensL") sensL = val;
  else if (id == "sensM") sensM = val;
  else if (id == "sensH") sensH = val;
  
  // Set Frequency Splits
  else if (id == "split1") {
    bandSplitLowMid = constrain(val, 3, bandSplitMidHigh - 1);
  }
  else if (id == "split2") {
    bandSplitMidHigh = constrain(val, bandSplitLowMid + 1, 60);
  }
  
  calibrationChanged = true;
  lastAdjustmentTime = millis();
  
  server.send(200, "text/plain", String(val));
}

void handleCalibrateNoise() {
  calibrateNoiseFloor();
  server.send(200, "text/plain", "Noise floor calibrated successfully!");
}

void handleRestoreDefaults() {
  sensL = 17;
  sensM = 23;
  sensH = 49;
  bandSplitLowMid = 11;
  bandSplitMidHigh = 49;
  saveCalibration();
  server.send(200, "text/plain", "Audio settings restored to defaults.");
}

void handleRestoreSensorDefaults() {
  calTemp = 0.0;
  calPress = 0.0;
  calHum = 0.0;
  nudgeTemp = 0.0;
  nudgePress = 0.0;
  nudgeHum = 0.0;
  saveCalibration();
  server.send(200, "text/plain", "Sensor calibration reset to raw values.");
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
  server.on("/status.json", handleStatusJson);
  server.on("/nudge", handleNudge);
  server.on("/set_nudge", handleSetNudge);
  server.on("/switch_mode", handleSwitchMode);
  server.on("/set_cal_tab", handleSetCalTab);
  server.on("/calibrate", handleCalibrate);
  server.on("/calibrate_noise", handleCalibrateNoise);
  server.on("/restore_defaults", handleRestoreDefaults);
  server.on("/restore_sensor_defaults", handleRestoreSensorDefaults);
  server.on("/set_audio", handleSetAudio);
  
  // Fallback for other files
  server.serveStatic("/", LittleFS, "/");

  server.begin();
  Serial.println("HTTP server started");
}
