#!/usr/bin/env python3
import rospy
from aruco_estimator import ArucoPoseEstimator
from obstacle_detector import ObstacleDetector
from drone_controller import DroneController
from navigation import navigate_with_obstacle_and_marker, align_with_marker_and_land
import time

def main():
    """
    Main function to control the drone's navigation through waypoints,
    perform obstacle avoidance, and land precisely using ARuco markers.
    """

    # Initialize the ROS node for drone navigation
    rospy.init_node("drone_navigation_with_obstacles", anonymous=True)
    
    # Initialize the ARuco marker pose estimator
    estimator = ArucoPoseEstimator()
    
    # Define the home location coordinates (latitude, longitude, altitude)
    home_lat = -35.3632622
    home_lon = 149.1652376
    home_alt = 10

    # List of waypoints for the drone to navigate through
    waypoints = [
        (-35.3631275, 149.1650502, 10),
        (-35.3628122, 149.1645427, 10),
        (-35.3626787, 149.1648526, 10),
        (-35.3625437, 149.1651273, 10),
    ]

    # Establish connection to the drone
    master = DroneController.connect_drone()

    # Initialize the obstacle detector
    detector = ObstacleDetector()

    # Iterate through each waypoint in the list
    for i, waypoint in enumerate(waypoints):
        lat, lon, alt = waypoint

        # Set drone to GUIDED mode
        DroneController.set_guided_mode(master)
        
        # Arm the drone
        DroneController.arm_drone(master)
        
        # Takeoff to the specified altitude
        DroneController.takeoff(master, alt)

        # Brief delay before proceeding
        time.sleep(1)

        # Set yaw orientation to 0 degrees (facing forward)
        DroneController.set_yaw(master, 0, 1)
        time.sleep(1)
        
        # Navigate to the waypoint while avoiding obstacles
        navigate_with_obstacle_and_marker(master, lat, lon, alt, detector, estimator)
        
        print(f"Performing precise landing at waypoint: Lat: {lat}, Lon: {lon}")
        
        # Align with the ARuco marker and perform precise landing
        align_with_marker_and_land(master, lat, lon, estimator)
        
        # Land the drone after alignment
        DroneController.land_drone(master)
        time.sleep(1)
        
        # Disarm the drone after landing
        DroneController.disarm_drone(master)
        time.sleep(1)

        # Simulate payload drop using a servo mechanism
        DroneController.move_servo(master, 9, 180)
        time.sleep(1)
        DroneController.move_servo(master, 9, 0)
        time.sleep(3)

    # Indicate mission completion
    print("Mission complete.")
    
    # Keep the ROS node running
    rospy.spin()

# Entry point for the script
if __name__ == "__main__":
    main()
    