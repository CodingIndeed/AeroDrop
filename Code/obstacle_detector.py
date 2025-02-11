import rospy
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    """
    A class to detect obstacles using LiDAR sensor data.
    It processes laser scan data to determine if obstacles are present in 
    front, left, right, or behind the drone.
    """

    def __init__(self):
        """
        Initializes the obstacle detector.
        - Defines the distance threshold for detecting obstacles.
        - Sets the angle ranges for different directions (front, left, right, back).
        - Subscribes to the LiDAR scan topic to receive obstacle data.
        """

        # Threshold distance to consider an object as an obstacle (in meters)
        self.obstacle_distance_threshold = 3.0  

        # Define angular ranges (in radians) for different directions
        self.front_min = 3.9611  # Minimum angle for detecting front obstacles
        self.front_max = 5.5319  # Maximum angle for detecting front obstacles

        self.left_min = 5.54935  # Minimum angle for detecting left obstacles
        self.left_max = 7.12015  # Maximum angle for detecting left obstacles

        self.right_min = 2.37285  # Minimum angle for detecting right obstacles
        self.right_max = 3.94365  # Maximum angle for detecting right obstacles

        self.back_min = 0.7854  # Minimum angle for detecting back obstacles
        self.back_max = 2.3554  # Maximum angle for detecting back obstacles

        # Flags to indicate whether an obstacle is detected in a particular direction
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_back = False

        # Subscribe to the LiDAR scan topic to receive sensor data
        self.lidar_sub = rospy.Subscriber("/lidar/scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        """
        Callback function that processes incoming LiDAR scan data.
        Updates obstacle detection flags based on the distance readings.

        :param msg: LaserScan message containing range data from the LiDAR.
        """

        # Reset obstacle detection flags before processing the new scan data
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_back = False

        # Iterate through all range readings from the LiDAR
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment  # Compute the angle of the current reading
            distance = msg.ranges[i]  # Get the distance at this angle

            # If the detected object is within a valid distance range, check its direction
            if 0.35 < distance < self.obstacle_distance_threshold:
                if self.front_min <= angle <= self.front_max:
                    self.obstacle_front = True  # Obstacle detected in front
                elif self.left_min <= angle <= self.left_max:
                    self.obstacle_left = True  # Obstacle detected on the left
                elif self.right_min <= angle <= self.right_max:
                    self.obstacle_right = True  # Obstacle detected on the right
                elif self.back_min <= angle <= self.back_max:
                    self.obstacle_back = True  # Obstacle detected behind
