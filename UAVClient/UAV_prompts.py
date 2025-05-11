from dronekit import Vehicle, VehicleMode, LocationGlobalRelative, Command
from typing import List, Tuple
from pymavlink import mavutil
import time
import enum
import math



class MissionStatus(enum): 
    NOT_READY = 0
    READY_FOR_MISSION = 1
    IN_MISSION = 2
    IN_MANUAL_OVERRIDE = 3
    IN_COMMAND_OVERRIDE = 4


#convert the following class to a request handler class as well. the UAV requests the central application for connection and registrations. When connected the application requests information along with commands using http protocol
#also create a setter class for self.msg that sends the message to the application, along with printing to the terminal
class UAV: 
    def __init__(self, connection_string: str, sys_id: int): 
        self.vehicle = Vehicle(connection_string, sys_id=sys_id)
        self.mission_status = MissionStatus()
        self.next_wp = None
        self.command_override = False

    def get_location(self): 
        location = self.vehicle.location.global_frame 
        # construct response to return the locations. Need to make a connector class (interface) that translate Location() object to json object that application expects
        # return location.lat, location.lon, location.alt
    
    def get_attitude(self): 
        #construct a response using a converter class (interface) to return: angles, speed
        pass
    
    def set_msg(self, msg): 
        self.msg = msg
        print(msg)

    def get_mission_status(self): 
        
        if not self.check_flight_rediness(): 
            self.mission_status =  MissionStatus().NOT_READY
            _, msg = self.mission_status_util.check_flight_rediness()
            self.msg = "Vehicle got an error: " + msg

        elif not self.vehicle.armed and not self.check_in_mission(): 
            self.mission_status = MissionStatus().READY_FOR_MISSION
            self.msg = "Vehicle is ready to execute mission"

        elif self.check_in_mission(): 
            self.mission_status = MissionStatus().IN_MISSION
            self.msg = "Vehicle is currently in mission"

        elif self.check_manual_override():  
            self.mission_status = MissionStatus().IN_MANUAL_OVERRIDE
            self.msg = "Vehicle is currenlty being controlled by the operator"
        
        elif self.command_override: 
            self.mission_status = MissionStatus().IN_COMMAND_OVERRIDE
            self.msg = "Vehicle's mission is currenlty being overrided by operator command"

        #return the message in json structure (need an interface) that the application expects


    def get_mission_path(self): 
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        #need to define the structure of the mission in json format. also make a connector class (interface) that tranlates the json structure of the central application to python Mission object used in this client


    def manual_override(self): 
        self.vehicle.mode = VehicleMode("LOITER")

    def fly_to_landing_zones(self): 
        #get the landing zone location of the nearest from the central application, convert it to Location object using a connector class
        self.command_override = True
        location = LocationGlobalRelative()
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)
        self.vehicle.simple_goto(location)
        while self.get_distance_metres(location) <= 2: 
            self.msg = "Going to landing zone"
            time.sleep(2)
        
        self.next_wp = self.vehicle.commands.next
        self.vehicle.mode == VehicleMode("LAND")
        while self.vehicle.armed: 
            self.msg = "Vehicle is landing at the landing zone"
            time.sleep(2)


            

    
    def get_distance_metres(self, target_location: LocationGlobalRelative):
        current_location = self.vehicle.location.global_relative_frame
        dlat = current_location.lat - target_location.lat
        dlong = current_location.lon - target_location.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


    def upload_mission(self): 
        #again need a connector class (interaface) that converts the json object from the central applicatio into Command objects. 
        #these will include nav_waypoints, set speed commands to stop the drone, set_servo commands when the drone is stopped for pickup (lower the cargo container) and dropoff commands
        pass

    def start_mission(self): 
        #arm and send the start_mission command
        self.command_override = False
        self.msg = "Switching Mode to Guided"
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

        print("Arming")
        self.vehicle.armed = True
        time.sleep(1)

        self.vehicle.mode = VehicleMode("AUTO")
        print("Waiting for Mission Start")
        time.sleep(1)

        print("Starting Mission")
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0, 
            mavutil.mavlink.MAV_CMD_MISSION_START, 
            0, 0, 0, 1, 0, 0, 0, 0
        )

        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
    
    def fly_to_point_and_wait(self): 
        #get the location from application
        self.command_override = True
        location = LocationGlobalRelative()
        self.vehicle.simple_goto(location)
    
    def continue_mission(self): 
        self.command_override = True
        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not self.vehicle.armed: #indicating that the vehicle is landed
            self.start_mission()
            #fix this as well
            tkoff_cmd = cmds[0]
            while self.vehicle.commands.next == 1: 
                self.msg = "Taking off"
                time.sleep(1)
            
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
            self.vehicle.commands.next = self.next_wp
            time.sleep(1)
            self.next_wp = None
        self.vehicle.mode = VehicleMode("AUTO")
        
            


    def get_msg(self): 
        msg = self.vehicle._master.recv_match(type='STATUSTEXT', blocking=True)
        if msg: 
            return msg.severity, msg.text
        return -1, "Cannot connect to vehicle"

    def check_flight_rediness(self): 
        msg_severity, _ = self.get_msg()
        return 0 <= msg_severity < 7 

    def check_in_mission(self): 
        return self.vehicle.armed and self.vehicle.mode in ["AUTO", "GUIDED"]

    def check_manual_override(self): 
        return self.vehicle.armed and self.vehicle.mode in ["LOITER", "ALTHOLD", "STABILIZE", "MANUAL"]
    
    
