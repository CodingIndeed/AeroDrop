import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
aruco = cv2.aruco  # Import OpenCV's Aruco marker module

class ArucoPoseEstimator:
    """
    A class to detect and estimate the pose of ARuco markers using OpenCV and ROS.
    This class subscribes to an image topic, processes frames, and extracts marker positions.
    """

    def __init__(self):
        """
        Initializes the ARuco marker pose estimator.
        - Subscribes to the camera image topic.
        - Loads camera calibration parameters.
        - Configures ARuco marker detection settings.
        """
        # Subscribe to the ROS topic that publishes camera images
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)

        # Initialize the OpenCV bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()

        # Load camera calibration parameters (intrinsic matrix and distortion coefficients)
        with np.load('camera_calib_live.npz') as X:
            self.mtx, self.dist = [X[i] for i in ('mtx', 'dist')]

        # Define the ARuco marker dictionary (6x6 markers, 250 possible IDs)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        # Set ARuco marker detection parameters
        self.parameters = aruco.DetectorParameters()

        # Define the size of the ARuco marker in meters
        self.marker_size = 1.0

        # Store the estimated marker position (x, y, z)
        self.marker_position = None

    def image_callback(self, data):
        """
        Callback function that processes the incoming camera image.
        Detects ARuco markers and estimates their position in the camera frame.

        :param data: ROS Image message containing the raw camera feed.
        """
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Convert the image to grayscale for ARuco detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ARuco markers in the image
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:  # If markers are detected
            # Draw detected ARuco markers on the image
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate the pose of each detected marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.mtx, self.dist)

            # Iterate through detected markers and extract position data
            for i in range(len(ids)):
                # Draw coordinate axes on each detected marker
                cv2.drawFrameAxes(cv_image, self.mtx, self.dist, rvecs[i], tvecs[i], 0.1)

                # Extract translation vector (marker position in camera frame)
                tvec = tvecs[i][0]
                real_x = tvec[0]  # X-coordinate in meters
                real_y = tvec[1]  # Y-coordinate in meters
                real_z = tvec[2]  # Z-coordinate in meters

                # Store the latest detected marker position
                self.marker_position = (real_x, real_y, real_z)

                # Print marker position in meters
                print(f"Marker ID {ids[i][0]} - X: {real_x:.2f} m, Y: {real_y:.2f} m, Z: {real_z:.2f} m")

                # Overlay marker position text on the image
                cv2.putText(cv_image, f"X: {real_x:.2f}m, Y: {real_y:.2f}m, Z: {real_z:.2f}m",
                            (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Display the processed image with detected markers
        cv2.imshow('ARuco Marker Detection', cv_image)

        # If 'q' is pressed, shutdown ROS node
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User exited.')
