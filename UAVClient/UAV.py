from dronekit import connect, Vehicle, VehicleMode, LocationGlobalRelative, Command
from enum import Enum
from pymavlink import mavutil
import time
import math
from flask import Flask, jsonify, request
import threading
import socket
import requests
from converters import *
from werkzeug.serving import make_server

class MissionStatus(Enum):
    NOT_READY = 0
    READY_FOR_MISSION = 1
    IN_MISSION = 2
    IN_MANUAL_OVERRIDE = 3
    IN_COMMAND_OVERRIDE = 4



class UAV:
    def __init__(self, connection_string: str, sys_id: int, app_url: str, port=5000, telemetry_fps = 2):
        self.vehicle = connect(connection_string, wait_ready=True)
        self.mission_status = MissionStatus.NOT_READY
        self.next_wp = None
        self.command_override = False
        self.app_url = app_url
        self.sys_id = sys_id
        self.port = port
        self.script_msg = ""
        self.mav_msg = None
        self.telemetry_fps = telemetry_fps
        self.pickup_mission_index = -1
        self.dropoff_mission_index = -1
        
        # Initialize Flask server with dynamic port
        self.flask_app = Flask(__name__)
        self.setup_routes()
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        # Register with central application
        self.register_with_central_app()

        # Telemetry Stream
        self.telemetry_thread = threading.Thread(target=self.send_telemetry_updates)
        self.telemetry_thread.daemon = True
        self.telemetry_thread.start()
        
        # Register the listener for STATUSTEXT messages
        self.msg_lock = threading.Lock()
        self.vehicle.add_message_listener("STATUSTEXT", self._status_text_callback)

    def send_telemetry_updates(self):
        """Periodically send telemetry data to the central application."""
        while True:
            try:
                # with self.msg_lock: 
                mav_msg = self.mav_msg

                telemetry_data = {
                    "uav_id": self.sys_id,
                    "location": self.get_location(),
                    "attitude": self.get_attitude(),
                    "mission_status": self.get_mission_status(),
                    "script_message": self.script_msg,
                    "mav_message": mav_msg 
                }
                # Send POST request to central app's telemetry endpoint
                response = requests.post(
                    f"{self.app_url}/telemetry/{self.sys_id}",
                    json=telemetry_data
                )
                response.raise_for_status()

            except Exception as e:
                print(f"Failed to send telemetry: {str(e)}")
            time.sleep(1/self.telemetry_fps)  # Adjust interval as needed
        
    
    def setup_routes(self):
        @self.flask_app.route('/telemetry/<int:uav_id>', methods=['GET'])
        def get_telemetry(uav_id):
            if uav_id != self.sys_id:
                return jsonify({"error": "Unauthorized access"}), 403
             
            return jsonify({
                "uav_id": self.sys_id,
                "location": self.get_location(),
                "attitude": self.get_attitude(),
                "mission_status": self.get_mission_status(),
                "message": self.script_msg
            })

        @self.flask_app.route('/command/<int:uav_id>', methods=['POST'])
        def handle_command(uav_id):
            if uav_id != self.sys_id:
                return jsonify({"error": "Unauthorized command"}), 403

            data = request.get_json()
            command = data.get('command')
            params = data.get('params', {})
            
            response = {"uav_id": self.sys_id}
            
            try:
                if command == 'manual_override':
                    success = self.manual_override()
                    if success:  
                        response["status"] = "Manual override activated"
                    else: 
                        return jsonify({"error": "Unauthorized command"}), 403
                        
                
                elif command == 'fly_to_landing_zone':
                    self.fly_to_landing_zones()
                    response["status"] = "Flying to landing zone"

                elif command == 'upload_mission':
                    self.upload_mission(params['mission'])
                    response["status"] = "Mission uploaded"

                elif command == 'start_mission':
                    self.start_mission()
                    response["status"] = "Mission started"
                else:
                    response["error"] = "Unknown command"
                    return jsonify(response), 400
                    
                return jsonify(response)
                
            except Exception as e:
                return jsonify({"error": str(e)}), 500
            
        @self.flask_app.route('/mission/<int:uav_id>', methods=['GET'])
        def get_mission(uav_id):
            if uav_id != self.sys_id:
                return jsonify({"error": "Unauthorized command"}), 403
            return jsonify(self.get_mission())



    def run_server(self):
        self._flask_server = make_server('0.0.0.0', self.port, self.flask_app)
        self._flask_server.serve_forever()

    def cleanup(self):
        """Clean up resources including Flask server and vehicle connection."""
        try:
            # Shutdown Flask server
            if self._flask_server:
                self._flask_server.shutdown()
            
            # Wait for server thread to finish
            if self.server_thread.is_alive():
                self.server_thread.join(timeout=5)
            
            # Close vehicle connection
            if self.vehicle:
                self.vehicle.close()
            
            self.set_script_msg("Cleanup completed")
        except Exception as e:
            self.set_script_msg(f"Error during cleanup: {str(e)}")
        
    def register_with_central_app(self):
        registration_data = {
            "uav_id": self.sys_id,
            "endpoint": f"http://{self.get_ip()}:{self.port}"
        }


        try:
            response = requests.post(f"{self.app_url}/register", json=registration_data)
            response.raise_for_status()
            response_data = response.json()
            self.landing_zones = [
                LandingZoneConverter.from_json(lz_data) 
                for lz_data in response_data.get("landing_zones", [])
            ]
            print(self.landing_zones)
            self.set_script_msg("Registration successful")
            
        except requests.exceptions.RequestException as e:
            self.set_script_msg(f"Registration failed: {str(e)}")

    def get_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('10.255.255.255', 1))
            IP = s.getsockname()[0] 
        except Exception:
            IP = '127.0.0.1'
        finally:
            s.close()
        return IP


    def set_script_msg(self, msg, console = True):
        self.script_msg = msg
        if console: 
            print(msg)
        # try:
        #     requests.post(f"{self.app_url}/log", json={"uav_id": self.sys_id, "message": msg})
        # except Exception as e:
        #     print(f"Failed to send log: {e}")

    def get_location(self):
        location = self.vehicle.location.global_frame
        return LocationConverter.to_json(location)

    def get_attitude(self):
        attitude = self.vehicle.attitude
        return AttitudeConverter.to_json(attitude)

    def get_mission_status(self):
        if not self.check_flight_readiness():
            self.mission_status = MissionStatus.NOT_READY
            msg = self.get_mav_msg()
            self.set_script_msg(f"Vehicle error: {msg}", False)

        elif not self.vehicle.armed and not self.check_in_mission():
            self.mission_status = MissionStatus.READY_FOR_MISSION
            self.set_script_msg("Ready for mission", False)

        elif self.check_in_mission():
            self.mission_status = MissionStatus.IN_MISSION
            self.set_script_msg("In mission", False)

        elif self.check_manual_override():
            self.mission_status = MissionStatus.IN_MANUAL_OVERRIDE
            self.set_script_msg("Manual override active", False)

        elif self.command_override:
            self.mission_status = MissionStatus.IN_COMMAND_OVERRIDE
            self.set_script_msg("Command override active", False)
        
        return MissionStatusConverter.to_json(self.mission_status)

    def manual_override(self):
        if self.vehicle.channels <= 1300: 
            return False
        self.vehicle.mode = VehicleMode("LOITER")
        return True

    def fly_to_landing_zones(self):
        try:
            response = requests.get(f"{self.app_url}/landing_zones/nearest", params={"uav_id": self.sys_id})
            if response.status_code == 200:
                location_data = response.json()
                location = LocationConverter.from_json(location_data)
                self.command_override = True
                self.vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)
                self.vehicle.simple_goto(location)
                while self.get_distance_metres(location) > 2:
                    self.set_script_msg("En route to landing zone")
                    time.sleep(2)
                self.next_wp = self.vehicle.commands.next
                self.vehicle.mode = VehicleMode("LAND")
                while self.vehicle.armed:
                    self.set_script_msg("Landing...")
                    time.sleep(2)
            else:
                self.set_script_msg("Failed to fetch landing zone")
        except Exception as e:
            self.set_script_msg(f"Error: {str(e)}")

    def get_distance_metres(self, target_location):
        current = self.vehicle.location.global_relative_frame
        dlat = target_location.lat - current.lat
        dlon = target_location.lon - current.lon
        return math.sqrt((dlat**2 + dlon**2)) * 1.113195e5

    def get_mission(self): 
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        return MissionConverter().to_json(cmds, self.pickup_mission_index, self.dropoff_mission_index)


    def upload_mission(self, mission_json):
        cmds_arr, self.pickup_mission_index, self.dropoff_mission_index = MissionConverter.from_json(mission_json)
        cmds = self.vehicle.commands
        cmds.clear()
        time.sleep(1)
        for cmd in cmds_arr:
            cmds.add(cmd)
        cmds.upload()

    def start_mission(self):
        self.command_override = False
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
        self.vehicle.armed = True
        time.sleep(1)
        self.vehicle.mode = VehicleMode("AUTO")
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 1, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        def _execute_mission(): 
            while True: 
                if self.vehicle.commands.next in [self.pickup_mission_index, self.dropoff_mission_index]: 
                    self.vehicle.mode = VehicleMode("GUIDED")
                    time.sleep(1)
                    self.vehicle.groundspeed = 0
                    time.sleep(1)
                    #open servo to lower the payload containers
                    self.set_servo(5, 2000)
                    time_start = time.time()
                    #wait for 10 seconds
                    while time.time() - time_start >= 20: 
                        #for override commands
                        if self.vehicle.mode != "GUIDED": return
                        time.sleep(1)
                    
                    self.set_servo(5, 1000)
                    time_start = time.time()
                    #wait for 10 seconds
                    while time.time() - time_start >= 20: 
                        #for override commands
                        if self.vehicle.mode != "GUIDED": return
                        time.sleep(1)
                    #continues mission
                    break
                    
                time.sleep(1)
             
        threading.Thread(target=_execute_mission, daemon=True).start()



    def check_flight_readiness(self):
        # severity, _ = self.get_mav_msg()
        # return 0 <= severity < 7
        return Vehicle.is_armable

    def check_in_mission(self):
        return self.vehicle.armed and self.vehicle.mode.name in ["AUTO", "GUIDED"]

    def check_manual_override(self):
        return self.vehicle.armed and self.vehicle.mode.name in ["LOITER", "ALTHOLD", "STABILIZE", "MANUAL"]

    def _status_text_callback(self, vehicle, name, message):
        """Callback to store the latest STATUSTEXT message."""
        self.mav_msg = "Severity {}: {}".format(message.severity, message.text)
        print(self.mav_msg)

    def get_mav_msg(self):
        """Get the most recent STATUSTEXT message."""
        with self.msg_lock:
            if self.mav_msg:
                return self.mav_msg
            else:
                return "No recent messages"
        
        
    def _get_msg(self):
        """Get the latest status message from the vehicle."""
        try:
            # Try to get any pending messages first
            while True:
                msg = self.vehicle._master.recv_match(type='STATUSTEXT', blocking=False)
                if msg is None:
                    break
                last_msg = msg  # Store the last message we saw
            
            if hasattr(self, 'last_msg'):
                return last_msg.severity, last_msg.text
            return -1, "No recent messages"
            
        except Exception as e:
            print(f"Error getting messages: {e}")
            return -1, f"Error: {str(e)}"
    
    def set_servo(self, servo_number: int, pwm_value: int):
        """
        Sends a MAV_CMD_DO_SET_SERVO command to control a servo output.

        :param vehicle: Connected DroneKit Vehicle object
        :param servo_number: Servo output number (e.g., 9 for SERVO9)
        :param pwm_value: PWM value to set (usually between 1000 and 2000)
        """
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,       # confirmation
            servo_number,  # param1: servo number
            pwm_value,     # param2: PWM value
            0, 0, 0, 0, 0   # unused parameters
        )

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        print(f"Set SERVO {servo_number} to PWM {pwm_value}")


#----Example Usage-----
# class SwarmManager:
#     def __init__(self, app_url):
#         self.app_url = app_url
#         self.uavs = {}
        
#     def add_uav(self, connection_string, sys_id, port):
#         self.uavs[sys_id] = UAV(connection_string, sys_id, self.app_url, port)
        
#     def send_command_to_uav(self, sys_id, command, params=None):
#         if sys_id not in self.uavs:
#             return {"error": "UAV not found"}
            
#         uav = self.uavs[sys_id]
#         try:
#             response = requests.post(
#                 f"{uav.app_url}/command/{sys_id}",
#                 json={"command": command, "params": params}
#             )
#             return response.json()
#         except Exception as e:
#             return {"error": str(e)}

# # Example usage
# if __name__ == "__main__":
#     CENTRAL_APP_URL = "http://central-app:8000"
    
#     # Create swarm manager
#     swarm = SwarmManager(CENTRAL_APP_URL)
    
#     # Add UAVs with unique IDs and ports
#     swarm.add_uav('udp:127.0.0.1:14550', 1, 5001)
#     swarm.add_uav('udp:127.0.0.1:14551', 2, 5002)
    
#     # Send command to specific UAV
#     print(swarm.send_command_to_uav(1, "start_mission"))
    
#     # Keep the main thread alive
#     while True:
#         time.sleep(1)