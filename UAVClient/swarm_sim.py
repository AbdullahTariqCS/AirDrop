import subprocess
import os
import time
from typing import List, Tuple
from UAVClient.UAV import UAV
from pymavlink import mavutil
import socket
from test_dronekit import fly_simple_mission
from threading import Thread

class ArduCopter: 
    def __init__(self, lat, lon, alt, heading, sys_id, mavproxy_port): 
        self.lat = lat
        self.lon = lon 
        self.alt = alt 
        self.heading = heading
        self.sys_id = sys_id
        self.mavproxy_port = mavproxy_port
        self.proc: subprocess.Popen = None
        self.dronekit_script: UAV = None
        self.dronekit_port = 2 + mavproxy_port
        self.dronekit_connection_string: str = "tcp:127.0.0.1:{}".format(self.dronekit_port)
        print("Connection String: {}".format(self.dronekit_connection_string))


    def get_home_str(self): 
        return f"{self.lat},{self.lon},{self.alt},{self.heading}"
    
    def start_instance(self): 
        cmd = [
            "ArduCopter.exe",
            "--model", "copter",
            "--speedup", "1",
            "--home", str(self.get_home_str()),
            "--sysid", str(self.sys_id),
            "--base-port", str(self.mavproxy_port)
        ]
        try:
            print(f"Launching ArduCopter: sysid={self.sys_id}, port={self.mavproxy_port}")
            self.proc = subprocess.Popen(
                cmd, 
                stdout=subprocess.DEVNULL
            )

            time.sleep(1)
            if self.proc.poll() is not None:  # Process exited
                stderr_output = self.proc.stderr.read()
                raise Exception(f"[ERROR] ArduCopter sysid={self.sys_id} crashed:\n{stderr_output}")

        except FileNotFoundError:
            raise Exception(f"[ERROR] ArduCopter.exe not found. Check PATH or filename.")

    def attach_script(self): 
        # print(self.dronekit_connection_string)
        # Thread(target=fly_simple_mission, args=(self.dronekit_connection_string, self.sys_id)).start()
        self.dronekit_script = UAV(self.dronekit_connection_string, self.sys_id, "http://127.0.0.1:8000", 5000 + self.sys_id - 1)
        


    def kill_instance(self): 
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
        self.dronekit_script.cleanup()


class Sim: 
    def __init__(self, swarm_connection_string: str = "127.0.0.1:14550"):
        self.mavproxy_proc: subprocess.Popen = None
        self.copters: List[ArduCopter] = [
            ArduCopter(34.070338, 72.642536, 344.1282212, 0, 1, 5760),
            ArduCopter(34.070303, 72.642508, 344.0000000, 0, 2, 5770),
            # ArduCopter(34.06910982932786, 72.64367631675118, 344.0000000, 0, 3, 5780),
            # ArduCopter(34.06908983275423, 72.64377824069282, 344.0000000, 0, 4, 5790)
        ]
        self.num_copters = len(self.copters)
        self.swarm_connection_string = swarm_connection_string


    def run_arducopter_instances(self):
        for copter in self.copters:
            copter.start_instance()

    def run_mavproxy(self): 
        cmd = [
            'start', 'cmd', '/k',
            "mavproxy.exe", 
        ]
        for copter in self.copters: 
            cmd.append(f"--master=tcp:127.0.0.1:{copter.mavproxy_port}")
        
        cmd += ["--out=127.0.0.1:14550"]
        print(f"Launching MAVProxy:", (" ").join(cmd))
        self.mavproxy_proc = subprocess.Popen(cmd, shell=True)


    
    def wait(self): 
        if self.mavproxy_proc is not None: 
            self.mavproxy_proc.wait()

    def kill_procs(self): 
        for copter in self.copters: 
            copter.kill_instance()

        if self.mavproxy_proc is not None and self.mavproxy_proc.poll() is None: 
            self.mavproxy_proc.terminate()
        


if __name__ == '__main__': 
    try: 
        sim = Sim()
        sim.run_arducopter_instances()
        sim.run_mavproxy()
        time.sleep(5)
        # sim.initialize_mavutil_router()

        for copter in sim.copters: 
            print("Running Dronekit Server")
            copter.attach_script()
            time.sleep(1)

        # sim.run_mavutil_router()
        while True: time.sleep(1)

    except KeyboardInterrupt as k: 
        sim.kill_procs()
        raise k

    except Exception as e: 
        sim.kill_procs()
        raise e


