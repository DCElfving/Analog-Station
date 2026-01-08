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

### Mode Selection
At the top of the page, use the buttons to switch between modes:
*   **SENSOR**: Displays Temperature, Humidity, and Pressure on the meters.
*   **AUDIO**: Displays a real-time spectrum analyzer (Low, Mid, High frequencies).

### Sensor Calibration
This section allows you to calibrate the environmental sensors. Each sensor (Temp, Humidity, Pressure) has its own card.

*   **Reference Value (Ref):**
    *   *Optional:* This allows you to calibrate the BME280 sensor to known correct value (e.g., from a trusted thermometer) in the input box labeled **"Ref"**.
    *   Click **SET**. This calibrates the internal data offset. 
*   **Nudge (+ / -):**
    *   If needed, use the **"- Nudge"** and **"+ Nudge"** buttons to physically move the needle so it points to the correct value.

### Audio Sensitivity
Adjust the sliders to control how responsive the meters are to sound.
*   **Sensitivity Sliders:**
    *   **Higher number (Right)** = More sensitive (Meter moves more).
    *   **Lower number (Left)** = Less sensitive (Meter moves less).
    *   *Default values:* Low: `17`, Mid: `23`, High: `49`.
*   **Saving Changes:**
    *   Changes are **saved automatically** a few seconds after you stop adjusting the sliders.
*   **Restore Defaults:**
    *   Click **"Restore Default Settings"** to reset the sensitivity to the default values.

### Noise Calibration
*   **Noise Floor Calibration:**
    *   If the meters are bouncing or jittery when the room is quiet, click the **"CALIBRATE NOISE FLOOR"** button.
    *   You will be asked to keep the room silent for 2 seconds.
    *   The device will measure the background noise and set a threshold to eliminate the jitter.

## 4. Troubleshooting
If you cannot connect via Wi-Fi:
1.  Open the Arduino IDE or a Serial Terminal (Baud Rate: 115200).
2.  Reset the device.
3.  The Serial Monitor will display the **Wi-Fi Name** and **IP Address** during boot.
