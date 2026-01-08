/* THE ANALOG STATION! - An audio & environmental monitor built with an ESP32-C3.
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
  while (!Serial && millis() - start < 500); 

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
  loadCalibration();
  
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
  if (currentToggleState != lastToggleState) {
    sensorMode = (currentToggleState == LOW);
    lastToggleState = currentToggleState;
  }
  
  // Process audio or sensor mode
  sensorMode ? processSensorMode() : processAudioMode();
  
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

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Analog Station</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; text-align: left; margin:0; padding:20px; background:#121212; color:#e0e0e0; }
    h2 { color: #ffc107; letter-spacing: 1px; margin-bottom: 20px; font-weight: 300; text-align: center; }
    
    /* Card Design */
    .card { background: #1e1e1e; padding: 25px; margin: 20px auto; max-width: 420px; border-radius: 12px; box-shadow: 0 4px 20px rgba(0,0,0,0.5); border: 1px solid #333; }
    
    /* Section Headers */
    .section-title { text-align: left; color: #ffc107; text-transform: uppercase; font-size: 0.85em; letter-spacing: 1.5px; margin-bottom: 20px; border-bottom: 1px solid #333; padding-bottom: 8px; font-weight: 600; }
    
    /* Rows & Inputs */
    .row { display: flex; align-items: center; justify-content: space-between; margin-bottom: 12px; padding: 10px; background: #2c2c2c; border-radius: 8px; } /* Medium Grey */
    .val { width: 80px; color: #ffc107; font-family: monospace; font-size: 1.2em; text-align: right; margin-right: 15px; }
    
    input[type=number] { width: 90px; padding: 10px; border: 1px solid #444; background: #252525; color: #fff; border-radius: 6px; font-size: 1em; outline: none; transition: border-color 0.2s; }
    input[type=number]:focus { border-color: #ffc107; }

    input[type=text] { width: 90px; padding: 10px; border: 1px solid #444; background: #252525; color: #fff; border-radius: 6px; font-size: 1em; outline: none; transition: border-color 0.2s; }
    input[type=text]:focus { border-color: #ffc107; }
    
    input[type=range] { width: 100%; margin-top: 10px; accent-color: #ffc107; height: 6px; border-radius: 3px; background: #444; }
    
    /* Buttons */
    button { cursor: pointer; border: none; border-radius: 6px; padding: 12px 15px; font-weight: 600; font-size: 0.9em; transition: all 0.2s; text-transform: uppercase; letter-spacing: 0.5px; }
    
    .btn-mode { width:48%; background: #333; color: #aaa; }
    .btn-mode.active { background: #ffc107; color: #121212; }
    
    .btn-cal { background: #ffc107; color: #121212; padding: 10px 15px; }
    .btn-cal:hover { background: #e0a800; transform: translateY(-1px); }
    
    .btn-nudge { background: #333; color: #ccc; width: 48%; padding: 10px; font-size: 0.85em; border: 1px solid #444; }
    .btn-nudge:hover { background: #444; color: #fff; border-color: #555; }
    
    .btn-ghost { background: transparent; color: #777; font-size: 0.8em; margin-top:15px; width:100%; text-decoration: underline; }
    .btn-ghost:hover { color: #aaa; }

    /* Sliders Container */
    .slider-container { text-align: left; margin-bottom: 20px; padding: 15px; background: #252525; border-radius: 8px; border: 1px solid #333; }
    .slider-label { display: flex; justify-content: space-between; font-size: 0.9em; color: #ddd; margin-bottom: 5px; font-weight: 500; }
    .sub-text { font-size: 0.75em; color: #888; margin-top: 10px; margin-bottom: 10px; line-height: 1.4; }
  </style>
  <script>
    function nudge(id, dir) {
      fetch('/nudge?id=' + id + '&dir=' + dir)
        .then(res => res.text())
        .then(val => { 
          console.log('Nudged', id, val);
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
    function calibrateNoise() {
      document.getElementById('btn_noise').innerText = "Calibrating...";
      fetch('/calibrate_noise')
        .then(res => res.text())
        .then(msg => { 
          alert(msg);
          document.getElementById('btn_noise').innerText = "Calibrate Noise Floor";
        });
    }
    function setAudio(id, val) {
        // Update display immediately
        document.getElementById('disp_' + id).innerText = val;
        // Debounce or just send? Sending on change (release) is fine. 
        // For 'input' event (dragging), we might flood. 
        // Using 'onchange' triggers on release. 'oninput' triggers on drag.
        // Let's use fetch on change.
        fetch('/set_audio?id=' + id + '&val=' + val);
    }
    function switchMode(mode) {
      fetch('/switch_mode?mode=' + mode)
        .then(res => res.text())
        .then(msg => location.reload());
    }
    function restoreDefaults() {
      if(!confirm('Restore default audio settings?')) return;
      fetch('/restore_defaults')
        .then(res => res.text())
        .then(msg => { alert(msg); location.reload(); });
    }
    function restoreSensorDefaults() {
      if(!confirm('Restore default sensor calibration (raw values)?')) return;
      fetch('/restore_sensor_defaults')
        .then(res => res.text())
        .then(msg => { alert(msg); location.reload(); });
    }
  </script>
</head>
<body>
  <h2>The Analog Station!</h2>
  
  <div class="card">
    <div style="display:flex; gap:10px; justify-content:flex-start;">
      <button class="btn-mode %BTN_SENSOR_ACTIVE%" onclick="switchMode('sensor')">Sensor</button>
      <button class="btn-mode %BTN_AUDIO_ACTIVE%" onclick="switchMode('audio')">Audio</button>
    </div>
  </div>
  
  <div class="card">
    <div class="section-title">Sensor Calibration</div>
    <div class="sub-text">Set reference values to match your sensor an outside source. You can also nudge the needle for each meter to ensure accurate display.</div>
    <div style="background:#252525; padding:15px; border-radius:8px; margin-bottom:15px; border:1px solid #333;">
        <div class="row" style="margin-bottom:10px; padding:0; background:none;">
          <div style="flex:1;">
            <div style="font-size:1.1em; text-align:left; color:#ccc; font-weight:500;">Temp</div>
            <div class="val" id="cur_temp" style="text-align:left; color:#ffc107;">%TEMP% F</div>
          </div>
          <input type="text" id="ref_temp" placeholder="Ref Temp">
          <button class="btn-cal" style="margin-left:10px;" onclick="calibrate('temp')">SET</button>
        </div>
        <div style="border-top:1px solid #333; margin:10px 0;"></div>
        <div style="display:flex; justify-content:space-between; width:100%; gap:10px;">
          <button class="btn-nudge" onclick="nudge('temp', -1)">- Nudge</button>
          <button class="btn-nudge" onclick="nudge('temp', 1)">+ Nudge</button>
        </div>
    </div>

    <div style="background:#252525; padding:15px; border-radius:8px; margin-bottom:15px; border:1px solid #333;">
        <div class="row" style="margin-bottom:10px; padding:0; background:none;">
          <div style="flex:1;">
            <div style="font-size:1.1em; text-align:left; color:#ccc; font-weight:500;">Humidity</div>
            <div class="val" id="cur_hum" style="text-align:left; color:#ffc107;">%HUM% %</div>
          </div>
          <input type="text" id="ref_hum" placeholder="Ref Hum">
          <button class="btn-cal" style="margin-left:10px;" onclick="calibrate('hum')">SET</button>
        </div>
        <div style="border-top:1px solid #333; margin:10px 0;"></div>
        <div style="display:flex; justify-content:space-between; width:100%; gap:10px;">
          <button class="btn-nudge" onclick="nudge('hum', -1)">- Nudge</button>
          <button class="btn-nudge" onclick="nudge('hum', 1)">+ Nudge</button>
        </div>
    </div>

    <div style="background:#252525; padding:15px; border-radius:8px; margin-bottom:15px; border:1px solid #333;">
        <div class="row" style="margin-bottom:10px; padding:0; background:none;">
          <div style="flex:1;">
            <div style="font-size:1.1em; text-align:left; color:#ccc; font-weight:500;">Pressure</div>
            <div class="val" id="cur_press" style="text-align:left; color:#ffc107;">%PRESS% hPa</div>
          </div>
          <input type="text" id="ref_press" placeholder="Ref hPa">
          <button class="btn-cal" style="margin-left:10px;" onclick="calibrate('press')">SET</button>
        </div>
        <div style="border-top:1px solid #333; margin:10px 0;"></div>
        <div style="display:flex; justify-content:space-between; width:100%; gap:10px;">
          <button class="btn-nudge" onclick="nudge('press', -1)">- Nudge</button>
          <button class="btn-nudge" onclick="nudge('press', 1)">+ Nudge</button>
        </div>
    </div>
    
    <button class="btn-ghost" onclick="restoreSensorDefaults()">Restore Raw Sensor Values</button>
  </div>

  <div class="card">
    <div class="section-title">Audio Sensitivity</div>
    <div class="sub-text">Adjust the sensitivity of your meters. The defaults work well for ambient music.</div>
    <div class="slider-container">
      <div class="slider-label"><span>Low</span> <span id="disp_sensL">%SENSL%</span></div>
      <div class="sub-text">90Hz - 340Hz</div>
      <input type="range" min="1" max="100" value="%SENSL%" onchange="setAudio('sensL', this.value)" oninput="document.getElementById('disp_sensL').innerText=this.value">
    </div>

    <div class="slider-container">
      <div class="slider-label"><span>Medium</span> <span id="disp_sensM">%SENSM%</span></div>
      <div class="sub-text">375Hz - 1.5kHz</div>
      <input type="range" min="1" max="100" value="%SENSM%" onchange="setAudio('sensM', this.value)" oninput="document.getElementById('disp_sensM').innerText=this.value">
    </div>

    <div class="slider-container">
      <div class="slider-label"><span>High</span> <span id="disp_sensH">%SENSH%</span></div>
      <div class="sub-text">1.5kHz - 2kHz</div>
      <input type="range" min="1" max="100" value="%SENSH%" onchange="setAudio('sensH', this.value)" oninput="document.getElementById('disp_sensH').innerText=this.value">
    </div>
    
    <button class="btn-ghost" onclick="restoreDefaults()">Restore Default Settings</button>
  </div>
  
  <div class="card">
    <div class="section-title">Calibration</div>
    <div class="sub-text">If your meters seem to bounce during silence, this can help. Ensure the room is completely silent, then tap below to set the noise floor.</div>
    <div style="color:#aaa; font-size:0.9em; margin-bottom:15px; line-height:1.4;">
    </div>
    <button id="btn_noise" style="width:100%; background:#333; color:#ffc107; border:1px solid #ffc107; padding:15px;" onclick="calibrateNoise()">CALIBRATE NOISE FLOOR</button>
  </div>
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
  
  s.replace("%SENSL%", String(sensL));
  s.replace("%SENSM%", String(sensM));
  s.replace("%SENSH%", String(sensH));
  
  // Update button colors based on current mode
  if (sensorMode) {
    s.replace("%BTN_SENSOR_ACTIVE%", "active");
    s.replace("%BTN_AUDIO_ACTIVE%", "");
  } else {
    s.replace("%BTN_SENSOR_ACTIVE%", "");
    s.replace("%BTN_AUDIO_ACTIVE%", "active");
  }
  
  server.send(200, "text/html", s);
}

void handleSwitchMode() {
  if (!server.hasArg("mode")) {
    server.send(400, "text/plain", "Missing mode arg");
    return;
  }
  
  String mode = server.arg("mode");
  if (mode == "sensor") {
    sensorMode = true;
    calibrationAdjustMode = false;
    lastSensorRead = 0; // Force immediate update
    server.send(200, "text/plain", "Switched to Sensor Mode");
  } else if (mode == "audio") {
    sensorMode = false;
    calibrationAdjustMode = false;
    server.send(200, "text/plain", "Switched to Audio Mode");
  } else {
    server.send(400, "text/plain", "Invalid mode");
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
  server.on("/nudge", handleNudge);
  server.on("/switch_mode", handleSwitchMode);
  server.on("/calibrate", handleCalibrate);
  server.on("/calibrate_noise", handleCalibrateNoise);
  server.on("/restore_defaults", handleRestoreDefaults);
  server.on("/restore_sensor_defaults", handleRestoreSensorDefaults);
  server.on("/set_audio", handleSetAudio);
  server.begin();
  Serial.println("HTTP server started");
}
