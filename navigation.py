import time
import math
from drone_controller import DroneController



def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing from the current position (lat1, lon1)
    to the target position (lat2, lon2).
    """

    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    delta_lon = lon2_rad - lon1_rad

    x = math.sin(delta_lon) * math.cos(lat2_rad)
    y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)

    bearing = math.atan2(x, y)
    bearing_deg = math.degrees(bearing)
    return (bearing_deg + 360) % 360  # Normalize to 0-360 degrees


def final_align(master, target_lat, target_lon):
    """
    Adjust the drone to hover directly over the target coordinates 
    before proceeding with landing.
    """
    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        print(f"Aligning: Lat: {current_lat:.7f}, Lon: {current_lon:.7f}")

        delta_lat = target_lat - current_lat
        delta_lon = target_lon - current_lon

        # If drone is close enough to target, stop adjustments
        if abs(delta_lat) < 0.000002 and abs(delta_lon) < 0.000002:
            print(f"Final alignment complete: Lat: {current_lat:.7f}, Lon: {current_lon:.7f}")
            break

        # Compute small velocity adjustments to refine alignment
        waypoint_vector_x, waypoint_vector_y = compute_waypoint_velocity(target_lat, target_lon, current_lat, current_lon)
        DroneController.set_velocity(master, waypoint_vector_x * 0.1, waypoint_vector_y * 0.1, 0)  # Slow adjustments
        time.sleep(0.2)


def navigate_with_obstacle_and_marker(master, target_lat, target_lon, target_alt, detector, estimator):
    """
    Navigate the drone to the target waypoint while avoiding obstacles. 
    If an obstacle is detected, attempt to find a clear path before continuing.
    """
    print(f"Navigating to waypoint: Lat: {target_lat}, Lon: {target_lon}, Alt: {target_alt}")

    while True:
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        current_lat = msg.lat / 1e7
        current_lon = msg.lon / 1e7
        print(f"Current position: Lat: {current_lat:.7f}, Lon: {current_lon:.7f}")

        # Check if the drone has reached the waypoint
        if abs(current_lat - target_lat) < 0.00005 and abs(current_lon - target_lon) < 0.00005:
            detector.obstacle_front = detector.obstacle_left = detector.obstacle_right = detector.obstacle_back = False

        # Handle obstacle avoidance
        if detector.obstacle_front or detector.obstacle_left or detector.obstacle_right or detector.obstacle_back:
            print("Obstacle detected! Evaluating free space...")

            # Decide movement direction based on obstacle location
            if detector.obstacle_front:
                if not detector.obstacle_left:
                    print("Front blocked. Moving left.")
                    DroneController.set_velocity(master, 0, -0.3, 0)  # Move left
                elif not detector.obstacle_right:
                    print("Front blocked. Moving right.")
                    DroneController.set_velocity(master, 0, 0.3, 0)  # Move right
                elif not detector.obstacle_back:
                    print("Front blocked. Moving backward.")
                    DroneController.set_velocity(master, -0.3, 0, 0)  # Move backward
                else:
                    print("All sides blocked! Holding position.")
                    DroneController.set_velocity(master, 0, 0, 0)  # Stop

            time.sleep(1)  # Allow movement to stabilize
            continue

        # If close to waypoint, initiate ARuco marker alignment for landing
        if abs(current_lat - target_lat) < 0.000005 and abs(current_lon - target_lon) < 0.000005:
            print(f"Close to waypoint: Lat: {current_lat:.7f}, Lon: {current_lon:.7f}")
            print("Performing ARuco marker alignment...")
            align_with_marker_and_land(master, target_lat, target_lon, estimator)
            break

        # Continue moving toward the waypoint
        waypoint_vector_x, waypoint_vector_y = compute_waypoint_velocity(target_lat, target_lon, current_lat, current_lon)
        DroneController.set_velocity(master, waypoint_vector_x, waypoint_vector_y, 0)
        time.sleep(0.5)


def align_with_marker_and_land(master, target_lat, target_lon, estimator):
    """
    Align the drone with the detected ARuco marker and perform a precise landing.
    """

    # Descend to a lower altitude to detect the marker
    DroneController.descend_to_altitude(master, 8.5)
    time.sleep(2)

    # Adjust yaw for better alignment
    DroneController.set_yaw(master, 0, 1)
    time.sleep(5)

    # Attempt to align based on ARuco marker detection
    max_attempts = 10  
    attempt = 0
    start_time = time.time()
    timeout = 30  
    previous_position = None
    stability_count = 0
    required_stability = 3  

    while attempt < max_attempts and time.time() - start_time < timeout and stability_count < required_stability:
        # Check if ARuco marker is detected
        if estimator.marker_position is not None:
            real_x, real_y, real_z = estimator.marker_position
            print(f"Detected marker at X: {real_x:.2f} m, Y: {real_y:.2f} m")

            # Check for stability in marker position
            if previous_position is not None:
                delta_x = abs(real_x - previous_position[0])
                delta_y = abs(real_y - previous_position[1])

                if delta_x < 0.01 and delta_y < 0.01:
                    stability_count += 1  
                else:
                    stability_count = 0  
            previous_position = (real_x, real_y)

            # Adjust position based on marker detection
            if abs(real_x) > 0.05:
                if real_x > 0:
                    DroneController.move_left(master, abs(real_x))
                else:
                    DroneController.move_right(master, abs(real_x))
            if abs(real_y) > 0.05:
                if real_y > 0:
                    DroneController.move_backward(master, abs(real_y))
                else:
                    DroneController.move_forward(master, abs(real_y))

            time.sleep(3)  
        else:
            print("No ARuco marker detected! Waiting for detection...")
            time.sleep(1)

        attempt += 1

    # Proceed with landing
    DroneController.land_drone(master)


def compute_waypoint_velocity(target_lat, target_lon, current_lat, current_lon):
    """
    Compute a velocity vector that directs the drone toward the target waypoint.
    The velocity scales based on distance to the target.
    """
    delta_lat = target_lat - current_lat
    delta_lon = target_lon - current_lon

    magnitude = math.sqrt(delta_lat**2 + delta_lon**2)
    if magnitude == 0:
        return 0, 0

    # Scale velocity based on distance to waypoint (closer -> slower movement)
    velocity_scale = max(0.1, min(1.0, magnitude / 0.0001))    
    return (delta_lat / magnitude) * velocity_scale, (delta_lon / magnitude) * velocity_scale
