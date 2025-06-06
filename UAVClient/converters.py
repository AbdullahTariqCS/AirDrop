from dronekit import LocationGlobalRelative, Command, Vehicle, LocationGlobal
from pymavlink import mavutil


class LocationConverter:
    @staticmethod
    def to_json(location):
        return {
            "lat": location.lat,
            "lon": location.lon,
            "alt": location.alt
        }
    
    @staticmethod
    def from_json(data):
        return LocationGlobalRelative(data['lat'], data['lon'], data['alt'])

class AttitudeConverter:
    @staticmethod
    def to_json(attitude):
        return {
            "roll": attitude.roll,
            "pitch": attitude.pitch,
            "yaw": attitude.yaw,
        }

class MissionStatusConverter:
    @staticmethod
    def to_json(status):
        return {
            "status": status.name,
            "code": status.value
        }

    
    

class MissionConverter:
    @staticmethod
    def from_json(mission_json):
        commands = []
        takeoff_cmd = Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0,
            0, 0, 0, 0,
            0, 0, 40
        )
        commands.append(takeoff_cmd)

        for i, wp in enumerate(mission_json['waypoints']):
            cmd = Command(
                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                wp['lat'], wp['lon'], 40 
            )
            commands.append(cmd)
        
        rtl_cmd = Command(
            0, 0, 0, mavutil.mavlink.MAV_FRAME_MISSION,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        commands.append(rtl_cmd)

        return commands, mission_json['pickup_mission_index'], mission_json['dropoff_mission_index']
    
    @staticmethod
    def to_json(commands: Command, pickup_mission_index: int, dropoff_mission_index: int): 
        commands_arr = []
        for cmd in commands: 
            #only use nav_wapoints
            if cmd.command == 16: 
                commands_arr.append({'lat': cmd.x, 'lon': cmd.y, 'alt': cmd.z})
        
        mission_json = {
            "commands": commands_arr,
            "pickup_mission_index": pickup_mission_index, 
            "dropoff_mission_index": dropoff_mission_index
        }
        return mission_json

class LandingZoneConverter:
    @staticmethod
    def to_json(location: LocationGlobal):
        return {
            "lat": location.lat,
            "lon": location.lon,
            "alt": location.alt,
        }
    
    @staticmethod
    def from_json(data) -> LocationGlobal:
        return LocationGlobal(data['lat'], data['lon'], data['alt'])




