from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math


def get_location_metres(original_location, dNorth, dEast):
    """Returns a LocationGlobalRelative object moved by dNorth/dEast meters."""
    earth_radius = 6378137.0  # Radius of "spherical" earth
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)


def fly_simple_mission(connection_string, sys_id):
    # Connect to the vehicle
    print(f"Connecting to {connection_string}...")
    # vehicle = connect(connection_string, wait_ready=True)
    vehicle = connect(
        connection_string,
        # source_system=255,  # Set controller sysid
        # target_system=sys_id,  # Explicit target
        wait_ready=True
    )
    
    print(sys_id, "Connected")
    # Wait for GPS and home lock
    while not vehicle.is_armable:
        print(sys_id, ": Waiting for vehicle to be armable...")
        time.sleep(1)

    print(sys_id, ": Waiting for ", sys_id * 10, "seconds")
    time.sleep(sys_id * 10)
    # Set mode and arm
    print(sys_id, ": Arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(sys_id, ": Waiting for arming...")
        time.sleep(1)


    # Take off
    target_altitude = 10
    print(f"{sys_id}: Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"{sys_id}: Altitude: {current_alt:.1f}")
        if current_alt >= target_altitude * 0.95:
            print(sys_id, ":Target altitude reached.")
            break
        time.sleep(1)

    # Move forward (North) 10 meters
    print(sys_id, ": Moving 10 meters forward (North)...")
    current_location = vehicle.location.global_relative_frame
    target_location = get_location_metres(current_location, dNorth=10, dEast=0)
    vehicle.simple_goto(target_location)

    time.sleep(10)  # Wait for movement

    # RTL
    print(sys_id, ": Returning to launch...")
    vehicle.mode = VehicleMode("RTL")

    # Close vehicle object before exiting
    time.sleep(5)
    vehicle.close()
    print(sys_id, ":Mission complete.")

# Example usage:
if __name__ == '__main__': 
    fly_simple_mission("tcp:127.0.0.1:5762", 1)
