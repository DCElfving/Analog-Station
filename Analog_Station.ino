/* THE ANALOG STATION - ESP32-C3 Audio & Environmental Monitor
 * Created in 2025 by Dave Elfving in San Francisco, using Claude 4.5.
 * 
 * Dual-mode monitoring station with analog meters and a vintage glow! 
 * - AUDIO MODE: Low, mid, and high frequency display
 * - SENSOR MODE: Temperature, humidity, and barometric pressure
 * 
 * Switch modes with toggle switch or serial commands.
 * 
 * SERIAL COMMANDS:
 * 's' - Switch to Sensor mode
 * 'a' - Switch to Audio mode
 * 'c' - Calibrate sensors (follow prompts to enter reference values)
 */

// Required libraries
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>

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
#define SAMPLES 256         
#define SAMPLE_RATE 20000   
#define SAMPLE_DELAY 50     
#define PWM_FREQ 500        
#define PWM_RESOLUTION 8    

// Processing constants
#define SMOOTHING 0.3              
#define SMOOTHING_INV (1.0 - SMOOTHING)  
#define TEMP_SCALE 1.8              
#define TEMP_OFFSET 32.0         

// Global objects and variables
Adafruit_BME280 bme;
Preferences preferences;
bool sensorMode = false;
bool lastToggleState = HIGH;
float lowFreqLevel = 0;
float midFreqLevel = 0;
float highFreqLevel = 0;
float tempOffset = 0.0;
float pressureOffset = 0.0;
float humidityOffset = 0.0;
bool calibrationValid = false;
#define LOW_SENSITIVITY 5
#define MID_SENSITIVITY 5
#define HIGH_SENSITIVITY 5

// Setup - runs once at startup
void setup() {
  Serial.begin(115200);
  delay(1000);

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
  
  // Display available commands
  Serial.println("ESP32-C3 Controller Ready");
  Serial.println("Commands: 's' Sensor mode, 'a' Audio mode, 'c' Calibrate sensors");
  
  if (!calibrationValid) {
    Serial.println("\n*** SENSOR CALIBRATION REQUIRED ***");
    Serial.println("Press 'c' to calibrate sensors");
  }
  
  delay(2000);
}

// Main loop - runs continuously
void loop() {
  checkSerialCommands();
  
  // Check toggle switch for mode changes
  bool currentToggleState = digitalRead(TOGGLE_PIN);
  if (currentToggleState != lastToggleState) {
    sensorMode = (currentToggleState == LOW);
    lastToggleState = currentToggleState;
    Serial.printf("Mode: %s (via toggle switch)\n", sensorMode ? "Sensor" : "Audio");
  }
  
  // Process audio or sensor mode
  sensorMode ? processSensorMode() : processAudioMode();
  delay(20);
}

// Handle serial commands
void checkSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch (command) {
      case 's':
      case 'S':
        sensorMode = true;
        Serial.println("Mode: Sensor (via console command)");
        break;
        
      case 'a':
      case 'A':
        sensorMode = false;
        Serial.println("Mode: Audio (via console command)");
        break;
        
      case 'c':
      case 'C':
        runCalibration();
        break;
        
      default:
        break;
    }
    
    // Clear serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}

// Sensor mode - reads BME280 and drives meters (updates every 10 seconds)
void processSensorMode() {
  static unsigned long lastSensorRead = 0;
  unsigned long currentTime = millis();
  
  // Rate limit to 10 seconds
  if (currentTime - lastSensorRead < 10000) {
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

  // Apply calibration offsets
  tempC += tempOffset;
  pressureHPa += pressureOffset;
  humidity += humidityOffset;

  // Convert to Fahrenheit
  float tempF = tempC * TEMP_SCALE + TEMP_OFFSET;

  // Scale to meter range (0-255)
  int tempMeter = constrain((int)((tempF - 32.0f) * (255.0f / 68.0f)), 0, 255);
  int pressureMeter = constrain((int)((pressureHPa - 980.0f) * (255.0f / 50.0f)), 0, 255);
  int humidityMeter = constrain((int)(humidity * (255.0f / 100.0f)), 0, 255);

  // Drive meters
  ledcWrite(METER_LOW, tempMeter);
  ledcWrite(METER_MID, humidityMeter);
  ledcWrite(METER_HIGH, pressureMeter);
  ledcWrite(BUILTIN_LED_C3, 127);

  // Display readings
  Serial.printf("%.1f°F(%d) %.1fhPa(%d) %.1f%%(%d)\n",
                tempF, tempMeter, pressureHPa, pressureMeter, humidity, humidityMeter);
}

// Audio mode - analyzes frequency spectrum and drives meters
void processAudioMode() {
  static int samples[SAMPLES];
  long sum = 0;

  // Collect 256 audio samples at 20kHz
  for (int i = 0; i < SAMPLES; i++) {
    samples[i] = analogRead(MIC_PIN);
    sum += samples[i];
    delayMicroseconds(SAMPLE_DELAY);
  }

  // Calculate DC offset
  int dcOffset = sum / SAMPLES;
  
  // Frequency detection using zero-crossing rate and amplitude in specific bands
  float lowEnergy = 0;
  float midEnergy = 0;
  float highEnergy = 0;
  
  for (int i = 1; i < SAMPLES; i++) {
    int current = samples[i] - dcOffset;
    int previous = samples[i-1] - dcOffset;
    
    // Detect zero crossings
    if ((previous < 0 && current >= 0) || (previous >= 0 && current < 0)) {
      float period = 0;
      // Estimate period from this crossing
      int lookback = 1;
      while (i - lookback > 0 && lookback < 1000) {
        int check = samples[i - lookback] - dcOffset;
        if ((previous < 0 && check >= 0) || (previous >= 0 && check < 0)) {
          period = lookback;
          break;
        }
        lookback++;
      }
      
      // 20kHz sample rate: period in samples = 20000 / frequency
      // 50-250Hz: periods of 80-400 samples
      // 300-650Hz: periods of 31-67 samples  
      // 800-2000Hz: periods of 10-25 samples
      
      float amplitude = abs(current) / 2048.0;
      
      if (period >= 80 && period <= 400) {
        lowEnergy += amplitude;
      } else if (period >= 31 && period < 67) {
        midEnergy += amplitude;
      } else if (period >= 10 && period < 25) {
        highEnergy += amplitude;
      }
    }
    
    // Also accumulate general amplitude in bands based on recent zero-crossing rate
    if (i >= 20) {
      int recentCrossings = 0;
      for (int j = 1; j < 20; j++) {
        int c = samples[i-j] - dcOffset;
        int p = samples[i-j-1] - dcOffset;
        if ((p < 0 && c >= 0) || (p >= 0 && c < 0)) {
          recentCrossings++;
        }
      }
      
      float amp = abs(current) / 2048.0;
      // Map crossing rate to frequency bands - broader ranges for music
      if (recentCrossings <= 2) {
        lowEnergy += amp * 0.5;
      } else if (recentCrossings <= 6) {
        midEnergy += amp * 0.5;
      } else {
        highEnergy += amp * 0.5;
      }
      if (recentCrossings <= 2) {
        lowEnergy += amp * 0.5;
      } else if (recentCrossings <= 10) {
        midEnergy += amp * 0.5;
      } else {
        highEnergy += amp * 0.5;
      }
    }
  }
  
  // Normalize by sample count
  lowEnergy = lowEnergy / SAMPLES;
  midEnergy = midEnergy / SAMPLES;
  highEnergy = highEnergy / SAMPLES;

  // Apply exponential smoothing for fluid motion
  lowFreqLevel = lowFreqLevel * SMOOTHING_INV + lowEnergy * SMOOTHING;
  midFreqLevel = midFreqLevel * SMOOTHING_INV + midEnergy * SMOOTHING;
  highFreqLevel = highFreqLevel * SMOOTHING_INV + highEnergy * SMOOTHING;

  // Scale to meter range (0-255) with fixed sensitivity
  // Different scale factors to normalize response across frequency bands
  // Higher frequencies need more gain because they're detected less frequently
  int lowMeter = constrain((int)(lowFreqLevel * 120.0 * LOW_SENSITIVITY), 0, 255);
  int midMeter = constrain((int)(midFreqLevel * 600.0 * MID_SENSITIVITY), 0, 255);
  int highMeter = constrain((int)(highFreqLevel * 2000.0 * HIGH_SENSITIVITY), 0, 255);

  // Drive meters
  ledcWrite(METER_LOW, lowMeter);
  ledcWrite(METER_MID, midMeter);
  ledcWrite(METER_HIGH, highMeter);
  ledcWrite(BUILTIN_LED_C3, 255 - midMeter);

  // Output to serial plotter
  Serial.printf("%d,%d,%d\n", lowMeter, midMeter, highMeter);
}

// Sensor calibration functions
// Load calibration offsets
void loadCalibration() {
  calibrationValid = preferences.getBool("calValid", false);
  
  if (calibrationValid) {
    tempOffset = preferences.getFloat("tempOffset", 0.0);
    pressureOffset = preferences.getFloat("pressOffset", 0.0);
    humidityOffset = preferences.getFloat("humidOffset", 0.0);
    
    Serial.println("\nLoaded calibration offsets:");
    Serial.printf("  Temperature: %.2f °C\n", tempOffset);
    Serial.printf("  Pressure: %.2f hPa\n", pressureOffset);
    Serial.printf("  Humidity: %.2f %%\n", humidityOffset);
  }
}

// Save calibration offsets
void saveCalibration() {
  preferences.putFloat("tempOffset", tempOffset);
  preferences.putFloat("pressOffset", pressureOffset);
  preferences.putFloat("humidOffset", humidityOffset);
  preferences.putBool("calValid", true);
  calibrationValid = true;
  Serial.println("Calibration saved to memory.");
}

// Interactive calibration routine
void runCalibration() {
  Serial.println("\n=== SENSOR CALIBRATION ===");
  Serial.println("Reading current sensor values...");
  delay(1000);
  
  // Read raw sensor values
  float rawTempC = bme.readTemperature();
  float rawPressureHPa = bme.readPressure() / 100.0F;
  float rawHumidity = bme.readHumidity();
  
  if (isnan(rawTempC) || isnan(rawPressureHPa) || isnan(rawHumidity)) {
    Serial.println("ERROR: Cannot read sensor. Check connections.");
    return;
  }
  
  // Display current readings
  Serial.println("\nCurrent sensor readings:");
  Serial.printf("  Temperature: %.2f °C (%.1f °F)\n", rawTempC, rawTempC * 1.8 + 32);
  Serial.printf("  Pressure: %.2f hPa\n", rawPressureHPa);
  Serial.printf("  Humidity: %.1f %%\n", rawHumidity);
  Serial.println("\nEnter reference values from a reference source:");
  
  // Collect reference values and calculate offsets
  Serial.print("\nReference Temperature (°F): ");
  float refTempF = waitForFloatInput();
  float refTempC = (refTempF - 32.0) / 1.8;
  tempOffset = refTempC - rawTempC;
  Serial.printf("Temperature offset: %.2f °C (%.2f °F)\n", tempOffset, tempOffset * 1.8);

  Serial.print("\nReference Pressure (hPa): ");
  float refPressure = waitForFloatInput();
  pressureOffset = refPressure - rawPressureHPa;
  Serial.printf("Pressure offset: %.2f hPa\n", pressureOffset);
  
  Serial.print("\nReference Humidity (%%): ");
  float refHumidity = waitForFloatInput();
  humidityOffset = refHumidity - rawHumidity;
  Serial.printf("Humidity offset: %.2f %%\n", humidityOffset);
  
  saveCalibration();
  Serial.println("\n=== CALIBRATION COMPLETE ===");
  Serial.println("Calibrated values will be used from now on.");
  Serial.println("Press 'c' anytime to recalibrate.\n");
}

// Wait for user input from serial monitor
float waitForFloatInput() {
  while (Serial.available() > 0) {
    Serial.read();
  }
  
  String input = "";
  while (true) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (input.length() > 0) {
          float value = input.toFloat();
          Serial.println(value);
          return value;
        }
      } else {
        input += c;
        Serial.print(c);
      }
    }
  }
}
