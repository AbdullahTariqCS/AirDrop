from flask import Flask, jsonify, request, render_template_string, Response
from collections import defaultdict
import threading
import time
import json
import requests

class CentralControlApplication:
    def __init__(self):
        self.app = Flask(__name__)
        self.registered_uavs = {}
        self.logs = defaultdict(list)
        self.telemetry = defaultdict(dict)
        self.landing_zones = [
            {"lat": 34.07020443923026, "lon": 72.64274724646779, "alt": 340},
            {"lat": 34.070754654891495, "lon": 72.64319147781445, "alt": 340}
        ]
        
        self.setup_routes()
        self.event_listeners = []
        
    def setup_routes(self):
        @self.app.route('/')
        def control_panel():
            return render_template_string('''
                <!DOCTYPE html>
                <html>
                <head>
                    <title>UAV Control Panel</title>
                    <style>
                        .uav-card { border: 1px solid #ccc; padding: 10px; margin: 10px; }
                        .log-box { height: 200px; overflow-y: scroll; border: 1px solid #ddd; padding: 5px; }
                    </style>
                </head>
                <body>
                    <h1>UAV Control Panel</h1>
                    <div id="uavs"></div>
                    <script>
                        const eventSource = new EventSource('/stream');
                        
                        eventSource.onmessage = function(e) {
                            const data = JSON.parse(e.data);
                            if (data.type === 'uav_update') {
                                updateUAVs(data.uavs);
                            }
                            if (data.type === 'telemetry_update') {
                                updateTelemetry(data.uav_id, data.telemetry);
                            }
                            if (data.type === 'log_update') {
                                updateLogs(data.uav_id, data.message);
                            }
                        };

                        function updateUAVs(uavs) {
                            const container = document.getElementById('uavs');
                            container.innerHTML = Object.keys(uavs).map(uavId => `
                                <div class="uav-card" id="uav-${uavId}">
                                    <h2>UAV ${uavId}</h2>
                                    <div class="telemetry">
                                        <h3>Telemetry</h3>
                                        <pre id="telemetry-${uavId}">${JSON.stringify(uavs[uavId].telemetry, null, 2)}</pre>
                                    </div>
                                    <div class="commands">
                                        <h3>Commands</h3>
                                        <button onclick="sendCommand(${uavId}, 'start_mission')">Start Mission</button>
                                        <button onclick="sendCommand(${uavId}, 'manual_override')">Manual Override</button>
                                        <button onclick="sendCommand(${uavId}, 'fly_to_landing_zone')">Go to Landing Zone</button>
                                        <button onclick="sendCommand(${uavId}, 'continue_mission')">Continue Mission</button>
                                    </div>
                              </div>
                            `).join('');
                        }

                        function updateTelemetry(uavId, telemetry) {
                            const element = document.getElementById(`telemetry-${uavId}`);
                            if (element) {
                                element.textContent = JSON.stringify(telemetry, null, 2);
                            }
                        }

                        function updateLogs(uavId, message) {
                            const logBox = document.getElementById(`logs-${uavId}`);
                            if (logBox) {
                                logBox.innerHTML += `<div>[${new Date().toLocaleTimeString()}] ${message}</div>`;
                                logBox.scrollTop = logBox.scrollHeight;
                            }
                        }

                        function sendCommand(uavId, command) {
                            fetch(`/command/${uavId}/${command}`, { method: 'POST' })
                                .then(response => response.json())
                                .then(data => console.log('Command response:', data));
                        }
                    </script>
                </body>
                </html>
            ''')

        @self.app.route('/stream')
        def stream():
            def event_stream():
                while True:
                    time.sleep(0.5)
                    yield "data: {}\n\n".format(json.dumps({
                        'type': 'uav_update',
                        'uavs': {k: {'telemetry': v} for k, v in self.telemetry.items()}
                    }))
            return Response(event_stream(), mimetype="text/event-stream")

        @self.app.route('/log', methods=['POST'])
        def receive_log():
            data = request.json
            self.logs[data['uav_id']].append(data['message'])
            return jsonify({"status": "logged"})

        @self.app.route('/telemetry/<int:uav_id>', methods=['POST'])
        def receive_telemetry(uav_id):
            self.telemetry[uav_id] = request.json
            return jsonify({"status": "telemetry updated"})

        @self.app.route('/command/<int:uav_id>/<command>', methods=['POST'])
        def forward_command(uav_id, command):
            if uav_id not in self.registered_uavs:
                return jsonify({"error": "UAV not registered"}), 404
                
            try:
                response = requests.post(
                    f"{self.registered_uavs[uav_id]}/command/{uav_id}",
                    json={"command": command}
                )
                return jsonify(response.json())
            except Exception as e:
                return jsonify({"error": str(e)}), 500

        @self.app.route('/register', methods=['POST'])
        def register_uav():
            data = request.json
            uav_id = data['uav_id']
            self.registered_uavs[uav_id] = data['endpoint']
            self.telemetry[uav_id] = {}
            
            
            return jsonify({
                "status": "registered",
                "landing_zones": self.landing_zones  # Include converted data
            })

    def run(self, port=8000):
        self.app.run(host="0.0.0.0", port=port, debug=False, threaded=True)

def run_control_center():
    # Start central application
    control_center = CentralControlApplication()
    control_center.run()

if __name__ == "__main__":
    run_control_center()