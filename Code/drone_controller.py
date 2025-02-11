import time
import math
from pymavlink import mavutil

class DroneController:
    """
    A class to control the drone using MAVLink commands via pymavlink.
    Provides functions for connection, movement, navigation, landing, and servo control.
    """

    @staticmethod
    def connect_drone():
        """
        Establishes a connection to the drone via MAVLink over UDP.
        Waits for a heartbeat signal to confirm connection.
        """
        print("Connecting to the drone...")
        master = mavutil.mavlink_connection("udp:127.0.0.1:14550")
        master.wait_heartbeat()
        print("Drone connected.")
        return master

    @staticmethod
    def set_guided_mode(master):
        """
        Sets the drone's flight mode to GUIDED.
        The drone must be in GUIDED mode for autonomous commands to work.
        """
        print("Setting mode to GUIDED...")
        master.set_mode("GUIDED")
        master.mode_mapping()

    @staticmethod
    def arm_drone(master):
        """
        Sends a command to arm the drone, allowing motor operation.
        """
        print("Arming drone...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        master.motors_armed_wait()
        print("Drone armed.")

    @staticmethod
    def takeoff(master, altitude):
        """
        Commands the drone to take off to a specified altitude.
        Waits until the drone reaches approximately 95% of the target altitude.
        """
        print(f"Taking off to {altitude} meters.")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )

        while True:
            msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            current_alt = msg.relative_alt / 1000.0
            if current_alt >= altitude * 0.95:
                print(f"Reached target altitude: {current_alt:.2f} meters.")
                break
            print(f"Current altitude: {current_alt:.2f} meters, waiting to reach {altitude} meters...")
            time.sleep(1)

    @staticmethod
    def get_current_yaw(master):
        """
        Retrieves the current yaw (heading) of the drone in degrees.
        """
        msg = master.recv_match(type='ATTITUDE', blocking=True)
        if msg:
            yaw_rad = msg.yaw
            yaw_deg = math.degrees(yaw_rad)
            if yaw_deg < 0:
                yaw_deg += 360
            return yaw_deg
        else:
            raise RuntimeError("Failed to retrieve yaw from ATTITUDE message.")

    @staticmethod
    def set_velocity(master, vx_body, vy_body, vz=0):
        """
        Commands the drone to move with a specified velocity in body-relative coordinates.
        """
        yaw_deg = DroneController.get_current_yaw(master)
        yaw_rad = math.radians(yaw_deg)
        
        # Convert body frame velocity to global frame
        vx_global = vx_body * math.cos(yaw_rad) - vy_body * math.sin(yaw_rad)
        vy_global = vx_body * math.sin(yaw_rad) + vy_body * math.cos(yaw_rad)

        type_mask = 0b0000111111000111
        master.mav.set_position_target_local_ned_send(
            0, master.target_system, master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            vx_global, vy_global, vz,
            0, 0, 0,
            0, 0
        )
        time.sleep(0.5)

    @staticmethod
    def disarm_drone(master):
        """
        Disarms the drone, preventing motor operation.
        """
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # Disarm
            0, 0, 0, 0, 0, 0
        )
        master.motors_disarmed_wait()
        print("Drone disarmed.")
        time.sleep(1)

    @staticmethod
    def land_drone(master, timeout=60):
        """
        Commands the drone to land and waits until landing is confirmed or timeout occurs.
        """
        print("Landing drone...")

        # Send the land command
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                print("Landing timeout exceeded.")
                break

            msg = master.recv_match(type="HEARTBEAT", blocking=True)
            if msg and not msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Landing confirmed. Drone disarmed.")
                break

            time.sleep(1) 

    @staticmethod
    # Move the drone in a given direction (X, Y, or Z) for a specified distance
    def move_in_direction(master, velocity_x, velocity_y, distance):
        # Define the time required to travel the specified distance at the given velocity
        velocity = 0.5  # Speed in m/s for movement (you can adjust this)
        time_to_travel = distance / velocity

        # Send the movement commands
        for _ in range(int(time_to_travel * 10)):  # Loop for time duration with small sleep intervals
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                int(0b110111000111),  # Use local NED frame with position control
                0, 0, 0,  # Unused (x, y, z positions)
                velocity_x, velocity_y, 0,  # Velocity in X and Y directions (Z is 0)
                0, 0, 0,  # Unused (x, y, z accelerations)
                0, 0  # Unused (yaw, yaw_rate)
            ))
            time.sleep(0.1)  # Sleep for a short duration to control the movement

        # Stop the drone's movement after reaching the target distance
        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111000111),  # Use local NED frame with position control
            0, 0, 0,  # Unused (x, y, z positions)
            0, 0, 0,  # Set velocities to 0 to stop the movement
            0, 0, 0,  # Unused (x, y, z accelerations)
            0, 0  # Unused (yaw, yaw_rate)
        ))   

    @staticmethod
    def descend_to_altitude(master, target_altitude):
        print(f"Descending to {target_altitude} meters...")
        while True:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if not msg:
                continue
            current_alt = max(0, msg.relative_alt / 1000.0)  # Convert from mm to meters

            print(f"Current relative altitude: {current_alt:.2f} meters")

            if current_alt <= target_altitude or current_alt < 0.5:
                print(f"Drone has reached {target_altitude} meters")
                break

            # Send velocity command to descend
            master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                int(0b110111000111), 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0))
            
            time.sleep(0.5)

        master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111000111), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

        print(f"Drone descent to {target_altitude} meters complete.") 

    @staticmethod
    def move_forward(master, distance):
        """ Moves the drone forward by a specified distance. """
        print(f"Moving forward by {distance} meters...")
        DroneController.move_in_direction(master, 0.5, 0, distance)  # Positive X direction (forward)
        print(f"Moved forward by {distance} meters.")


    @staticmethod
    def move_backward(master, distance):
        """ Moves the drone backward by a specified distance. """
        print(f"Moving backward by {distance} meters...")
        DroneController.move_in_direction(master, -0.5, 0, distance)  # Negative X direction (backward)
        print(f"Moved backward by {distance} meters.")


    @staticmethod
    def move_right(master, distance):
        """ Moves the drone to the right by a specified distance. """
        print(f"Moving left by {distance} meters...")
        DroneController.move_in_direction(master, 0, -0.5, distance)  # Negative Y direction (left)
        print(f"Moved left by {distance} meters.")


    @staticmethod
    def move_left(master, distance):
        """ Moves the drone to the left by a specified distance. """
        print(f"Moving right by {distance} meters...")
        DroneController.move_in_direction(master, 0, 0.5, distance)  # Positive Y direction (right)
        print(f"Moved right by {distance} meters.")

    @staticmethod
    def set_yaw(master, target_yaw, direction):
        master.mav.command_long_send(
            master.target_system,  # target system
            master.target_component,  # target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            target_yaw,  # yaw angle
            0,  # yaw speed (not used)
            direction,  # direction (1: clockwise, -1: counterclockwise)
            0,  # relative (0: absolute, 1: relative)
            0, 0, 0  # unused params
        )
        print(f"Yaw set to {target_yaw} degrees.")

    @staticmethod
    def move_servo(master, servo_id, degrees):
        # Ensure the degree value is within the valid range (0 to 180)
        if degrees < 0 or degrees > 180:
            raise ValueError("Degrees should be in the range 0 to 180")
        
        # Convert degrees to PWM value (linear conversion)
        pwm_value = 1000 + (degrees / 180) * 1000
        
        # Send the MAVLink command to move the servo
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # Confirmation
            servo_id,  # Servo ID (e.g., 9 for SERVO9)
            pwm_value,  # PWM value for servo position (calculated from degrees)
            0, 0, 0, 0, 0  # Unused parameters
        )
        print(f"Servo moved to {degrees} degrees!")

