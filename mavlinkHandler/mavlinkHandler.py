from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase
import time
import math
import geopy.distance
from dronekit import connect, VehicleMode, LocationGlobalRelative


class MAVLinkHandlerDronekit:
    """
    init by: mavlink_handler = MAVLinkHandler(f'127.0.0.1:{port}')
    """
    def __init__(self, connection_string, distance_radius=15):
        self.master = connect(connection_string, wait_ready=True)
        self.boot_time = time.time()
        self.radians_to_degrees = lambda radians: radians * (180.0 / math.pi)
        self.DISTANCE_RADIUS = distance_radius 


    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def set_target_attitude(self, roll = 0.0, pitch = 0.0,
                         yaw = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
        """
        use_yaw_rate: the yaw can be controlled using yaw OR yaw_rate.
                    When one is used, the other is ignored by Ardupilot.
        thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
                Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
                the code for maintaining current altitude.
        """
        if yaw is None:
            # this value may be unused by the vehicle, depending on use_yaw_rate
            yaw = self.master.attitude.yaw

        # Thrust >  0.5: Ascend
        # Thrust == 0.5: Hold the altitude
        # Thrust <  0.5: Descend
        msg = self.master.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            self.to_quaternion(roll, pitch, yaw), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            math.radians(yaw_rate), # Body yaw rate in radian/second
            thrust  # Thrust
        )
        
        self.master.send_mavlink(msg)

    def set_mode(self, mode_name):
        self.master.mode = VehicleMode(mode_name)

    def get_attitude(self):
        return self.radians_to_degrees(self.master.attitude.roll), self.radians_to_degrees(self.master.attitude.pitch), self.radians_to_degrees(self.master.attitude.yaw)
        

    def simple_go_to(self, lat, lon, alt):

        a_location = LocationGlobalRelative(lat, lon, alt)
        self.master.simple_goto(a_location)
        
        print("Going to:",lat,lon,alt)
        while True:
            vehicle_lat, vehicle_lon, vehicle_alt = self.get_location()
            point1 = (vehicle_lat, vehicle_lon)
            target = (lat, lon)
            distance = geopy.distance.geodesic(target, point1).meters
            if distance <= self.DISTANCE_RADIUS:
                break

    def return_master(self):
        return self.master




    def get_location(self):
        return self.master.location.global_frame.lat, self.master.location.global_frame.lon, self.master.location.global_frame.alt

    def get_mode(self):
        return self.master.mode.name
    
    def get_heading(self):
        return self.master.heading
    
    def get_air_speed(self):
        return self.master.airspeed
    
    def is_armed(self):
        return self.master.armed
    
    def is_armable(self):
        return self.master.is_armable

class MAVLinkHandlerPymavlink:
    def __init__(self, connection_string, distance_radius=15):
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        self.boot_time = time.time()
        self.radians_to_degrees = lambda radians: radians * (180.0 / math.pi)
        self.DISTANCE_RADIUS = distance_radius 

    def set_target_attitude(self, roll, pitch, yaw, thrust):
        self.master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self.boot_time)),
            self.master.target_system, self.master.target_component,
            0,
            QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
            0, 0, 0, thrust
        )

    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.set_mode(mode_id)

    def get_attitude(self):
        self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component,
                                         mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
        msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        if msg is not None:
            return self.radians_to_degrees(msg.pitch), self.radians_to_degrees(msg.roll), self.radians_to_degrees(msg.yaw)
        
    def simple_goto(self, lat , lon, alt):

        self.master.mav.mission_item_send(
            self.master.target_system, self.master.target_component,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2, 0, 0, 0, 0, 0,
            lat, lon, alt
        )

        while True:
            point1 = self.get_location()
            target = (lat, lon)
            distance = geopy.distance.geodesic(target, point1[:2]).meters
            time.sleep(0.3)
            if distance <= self.DISTANCE_RADIUS:
                break

    def get_location(self):
        self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component,
                                         mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 30, 1)
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg is not None:
            return msg.lat/1.0e7, msg.lon/1.0e7, msg.alt/1000.0

    def get_mode(self):
        message = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if message:
            mode = mavutil.mode_string_v10(message)
            return mode
        else:
            print("No HEARTBEAT message received")

    def arm_vehicle(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # 1 to arm, 0 to disarm
        )
        print(f"Vehicle armed")


if __name__ == '__main__':
    """Test"""
    mavlink_handler = MAVLinkHandler('127.0.0.1:14551')

    print("Change mode to GUIDED")
    mavlink_handler.set_mode('GUIDED')


    print("Location:", mavlink_handler.get_location())
    print("Mode:", mavlink_handler.get_mode())
    print("Heading:", mavlink_handler.get_heading())
    print("Air Speed:", mavlink_handler.get_air_speed())
    print("Armed:", mavlink_handler.is_armed())
    print("Armable:", mavlink_handler.is_armable())

        
        
        
    