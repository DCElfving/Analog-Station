from http.server import SimpleHTTPRequestHandler, HTTPServer
import json
import urllib.parse
import os
import random
import math
import time

# Configuration
PORT = 8000
# Since we are running inside Analog_Station, data is just 'data'
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')

# Internal State (Mocking the ESP32 Variables)
state = {
    "temp": 72.4,
    "press": 1013.2,
    "hum": 45.0,
    "sensL": 17,
    "sensM": 23,
    "sensH": 49,
    "nudgeTemp": 0.0,
    "nudgePress": 0.0,
    "nudgeHum": 0.0,
    "mode": "sensor", # 'audio', 'sensor', 'calibrate'
    "calTab": "sensor" # 'audio', 'sensor'
}

class MockESP32Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        # Parse query params
        parsed_path = urllib.parse.urlparse(self.path)
        path = parsed_path.path
        query = urllib.parse.parse_qs(parsed_path.query)

        # 1. API: STATUS
        if path == '/status.json':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            
            # Simulate sensor fluctuation
            resp_data = state.copy()
            # jitter sensors slightly to look alive
            resp_data['temp'] += round(random.uniform(-0.1, 0.1), 1)
            
            # Provide RAW values (simulating the raw sensor reading before nudge)
            resp_data['rawTemp'] = round(resp_data['temp'] - resp_data['nudgeTemp'], 1)
            resp_data['rawHum'] = round(resp_data['hum'] - resp_data['nudgeHum'], 1)
            resp_data['rawPress'] = round(resp_data['press'] - resp_data['nudgePress'], 1)
            
            # Simulate Audio Levels (Sine wave + random noise for dynamic effect)
            t = time.time()
            resp_data['audioL'] = int((math.sin(t * 2) + 1) * 50 + random.randint(0, 20)) # Bass pulses
            resp_data['audioM'] = int((math.sin(t * 5) + 1) * 40 + random.randint(0, 40)) # Mids faster
            resp_data['audioH'] = int((math.sin(t * 8) + 1) * 30 + random.randint(0, 60)) # Highs erratic
            
            # Clamp to 0-255
            resp_data['audioL'] = max(0, min(255, resp_data['audioL']))
            resp_data['audioM'] = max(0, min(255, resp_data['audioM']))
            resp_data['audioH'] = max(0, min(255, resp_data['audioH']))

            self.wfile.write(json.dumps(resp_data).encode())
            return

        # 2. SET NUDGE API
        if path == '/set_nudge':
            # ?id=temp&val=2.5
            start_id = query.get('id', [''])[0]
            val = query.get('val', ['0'])[0]
            
            if start_id == 'temp': state['nudgeTemp'] = float(val)
            elif start_id == 'hum': state['nudgeHum'] = float(val)
            elif start_id == 'press': state['nudgePress'] = float(val)
            
            self.send_text_response(val)
            return

        # 3. SET AUDIO API
        if path == '/set_audio':
            # ?id=sensL&val=50
            audi_id = query.get('id', [''])[0]
            val = query.get('val', ['0'])[0]
            
            if audi_id in state:
                state[audi_id] = int(val)
                
            self.send_text_response(val)
            return

        # 4. SWITCH MODE API
        if path == '/switch_mode':
            mode = query.get('mode', ['sensor'])[0]
            if mode in ['audio', 'sensor', 'calibrate']:
                state['mode'] = mode
            self.send_text_response(f"Switched to {mode}")
            return

        # 5. SET CAL TAB API
        if path == '/set_cal_tab':
            tab = query.get('tab', ['sensor'])[0]
            if tab in ['audio', 'sensor']:
                state['calTab'] = tab
            self.send_text_response(f"CalTab set to {tab}")
            return
            
        # 6. RESTORE DEFAULTS
        if path == '/restore_sensor_defaults':
            state['nudgeTemp'] = 0.0
            state['nudgeHum'] = 0.0
            state['nudgePress'] = 0.0
            self.send_text_response("Defaults restored")
            return
        
        if path == '/restore_defaults':
             state['sensL'] = 17
             state['sensM'] = 23
             state['sensH'] = 49
             self.send_text_response("Audio Defaults restored")
             return

        if path == '/calibrate_noise':
             time.sleep(1)
             self.send_text_response("Noise floor calibrated")
             return

        # Static File Serving Logic
        
        # Rewrite root to index.html
        if path == '/' or path == '/index.html':
            self.path = '/data/index.html'
        
        # If the path doesn't start with /data, assume it's a static asset in /data
        # (e.g. browser asks for /style.css, we serve /data/style.css)
        elif not path.startswith('/data'):
             self.path = '/data' + path
            
        # print(f"Serving: {self.path} (mapped from {path})")
        return super().do_GET()

    def send_text_response(self, text):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        self.wfile.write(text.encode())

print(f"Starting Mock ESP32 Server at http://localhost:{PORT}")
print(f"Serving files from: {DATA_DIR}")
httpd = HTTPServer(('0.0.0.0', PORT), MockESP32Handler)
try:
    httpd.serve_forever()
except KeyboardInterrupt:
    print("\nServer stopped.")
