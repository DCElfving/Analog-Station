# Analog Station - Calibration Guide
Welcome to the Analog Station! Follow these steps to connect to your device, calibrate the sensors, and tune the audio response.

## 1. Power Up & Identify
1.  Connect your ESP32-C3 module to power (USB).
2.  Wait a few seconds for the device to start.
3.  Open your computer or phone's Wi-Fi settings.
4.  Look for a network named **"Analog Station XXXX"** (where XXXX are 4 unique characters, e.g., "A1B2").
    *   *Note: If you see multiple "Analog Station" networks, check the label that came with your device for its unique address.*

## 2. Connect
1.  Select the network **"Analog Station XXXX"**.
2.  There is **no password**.
3.  Once connected, your device may warn you that there is no internet access. This is normal; stay connected.

## 3. Configure & Calibrate
1.  Open a web browser (Chrome, Safari, etc.).
2.  Type **`192.168.4.1`** into the address bar and press Enter.
3.  You should see the **Analog Station** interface.

### Mode Selection (Top Navigation)
Use the navigation bar at the top of the screen to switch views:
*   **AUDIO**: Displays a real-time spectrum analyzer (Bass, Mid, Treble) on the meters.
*   **SENSOR**: Displays digital readouts of Temperature, Humidity, and Pressure.
*   **SETTINGS**: Access calibration controls for both sensors and audio.

### Settings & Calibration
Tap the **Settings** icon (gear/tools) in the top navigation. Then use the tabs at the top of the panel ("Sensor" / "Audio") to switch between categories.

#### Sensor Settings (Top Tab "SENSOR")
Use this section to align the analog meter needles with the digital readings.
1.  **Meter Offset Sliders:**
    *   Drag the slider left or right to **Adjust temperature meter offset** (or Humidity/Pressure).
    *   The digital reading is shown in red next to the slider header.
    *   Use this if the needle position doesn't perfectly match the digital value shown.
2.  **Diagnostics:**
    *   View raw sensor data at the bottom of the page verify sensor health.
3.  **Advanced:**
    *   **Reference Calibration:** If needed, calibrate sensors with a known reference source.
    *   **Restore Default Calibration:** Resets all sensor offsets to defaults.

#### Audio Settings (Top Tab "AUDIO")
Use this section to tune how the meters react to sound and music.
1.  **Sensitivity Sliders:**
    *   **Bass / Mid / Treble Sensitivity:** Adjusts the gain for each frequency band.
    *   **Higher number (Right)** = More sensitive (Meter moves more).
    *   **Lower number (Left)** = Less sensitive (Meter moves less).
2.  **Noise Floor:**
    *   **Calibrate Noise Floor:** If the meters are bouncing or jittery when the room is quiet, click this button.
    *   Keep the room silent for 2 seconds while it measures background noise.
3.  **Restore Defaults:** Resets sensitivity to factory presets.

*Note: All settings are saved automatically.*
