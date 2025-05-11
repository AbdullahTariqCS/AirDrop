import subprocess
import time
from threading import Thread
import socket
from pymavlink import mavutil
import sys
from test_dronekit import fly_simple_mission

class ArduCopter:
    def __init__(self, lat, lon, alt, heading, sys_id, port):
        self.lat = lat
        self.lon = lon 
        self.alt = alt 
        self.heading = heading
        self.sys_id = sys_id
        self.port = port
        self.proc = None
        # Use sequential ports for MAVProxy outputs
        self.vehicle_port = 14550 + self.sys_id  # 14551, 14552, etc.
        self.connection_string = f"127.0.0.1:{self.vehicle_port}"
        print(f"Connection String: {self.connection_string}")

    def get_home_str(self):
        return f"{self.lat},{self.lon},{self.alt},{self.heading}"
    
    def start_instance(self):
        cmd = [
            "ArduCopter.exe",
            "--model", "copter",
            "--speedup", "1",
            "--home", self.get_home_str(),
            "--sysid", str(self.sys_id),
            "--base-port", str(self.port)
        ]
        try:
            print(f"Launching ArduCopter: sysid={self.sys_id}, port={self.port}")
            self.proc = subprocess.Popen(
                cmd, 
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
            )
            time.sleep(3)  # Increased startup delay

        except FileNotFoundError:
            raise Exception("ArduCopter.exe not found. Check PATH or filename.")

    def attach_script(self):
        print(f"Starting DroneKit script on {self.connection_string}")
        self.script_thread = Thread(target=fly_simple_mission, args=(self.connection_string, self.sys_id)).start()

    def kill_instance(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
        # self.script_thread.join()

class Sim:
    def __init__(self):
        self.copters = [
            ArduCopter(34.070338, 72.642536, 344.1282212, 0, 1, 5760),
            ArduCopter(34.070303, 72.642508, 344.0000000, 0, 2, 5770),
        ]
        self.mavproxy_proc = None

    def run_arducopter_instances(self):
        for copter in self.copters:
            copter.start_instance()
        time.sleep(5)  # Wait for all instances to initialize

    def run_mavproxy(self):
        cmd = [
            'start', 'cmd', '/k',
            "mavproxy.exe",
        ]
        
        for copter in self.copters:
            cmd.append(f"--master=tcp:127.0.0.1:{copter.port}")
        
        # for copter in self.copters:
            # cmd.append(f"--out=udp:127.0.0.1:{copter.vehicle_port}")
        for i, copter in enumerate(self.copters, 1):
            cmd += [
                "--master", f"tcp:127.0.0.1:{copter.port}",
                "--out", f"udp:127.0.0.1:{copter.vehicle_port}",
                "--link", f"{i}",
                "--target-system", f"{copter.sys_id}"
            ]

        
        cmd.append("--out=udp:127.0.0.1:14550")
        
        print("Starting MAVProxy with:", " ".join(cmd))
        self.mavproxy_proc = subprocess.Popen(
            cmd,
            shell=True,
            # creationflags=subprocess.CREATE_NEW_CONSOLE
        )
        time.sleep(5) 

    def start_dronekit_scripts(self):
        for copter in self.copters:
            copter.attach_script()
            time.sleep(1)  

    def kill_procs(self):
        for copter in self.copters:
            copter.kill_instance()

        if self.mavproxy_proc and self.mavproxy_proc.poll() is None:
            self.mavproxy_proc.terminate()

if __name__ == '__main__':
    sim = Sim()
    try:
        sim.run_arducopter_instances()
        sim.run_mavproxy()
        sim.start_dronekit_scripts()
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        sim.kill_procs()