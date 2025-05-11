from dronekit import LocationGlobalRelative, Command, Vehicle
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
        for wp in mission_json['waypoints']:
            cmd = Command(
                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                wp['lat'], wp['lon'], wp['alt']
            )
            commands.append(cmd)
        return commands