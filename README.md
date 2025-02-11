# AeroDrop: Precision Drone Delivery with ARuco Marker Landing and Obstacle Avoidance

## Project Overview

AeroDrop is an autonomous drone delivery system designed to navigate through waypoints (lat and lon), avoid obstacles, and precisely land on an ARuco marker. The drone is controlled using MAVLink commands via `pymavlink`, while ARuco markers are detected using OpenCV. A LiDAR sensor is used for obstacle detection. The system is simulated in Gazebo with ArduPilot for real-world-like testing. The waypoints used in this project are actual latitude and longitude coordinates, making it applicable for real-world scenarios.

Additionally, since this project utilizes the ArduPilot plugin on Gazebo, the same control program can be used for a real-world drone with minimal modifications, enabling seamless transition from simulation to deployment. The precise landing system ensures that the drone lands with a maximum deviation of only 7 cm from the target ARuco marker, making it highly accurate for real-world applications.

The system also utilizes a **camera calibration file (`camera_calib_live.npz`)**, which contains the intrinsic matrix and distortion coefficients necessary for accurate ARuco marker detection and pose estimation.

## Features

- **Autonomous navigation**: The drone follows predefined waypoints.
- **Obstacle avoidance**: Uses a LiDAR sensor to detect and bypass obstacles.
- **Precise landing**: Detects and aligns with an ARuco marker before landing, with a maximum deviation of 7 cm.
- **MAVLink communication**: Controls the drone via `pymavlink`.
- **ROS Integration**: Uses ROS to manage communication between modules.
- **Camera Calibration**: Uses a precomputed calibration file (`camera_calib_live.npz`) to enhance marker detection accuracy.

## Technologies & Libraries Used

- **Gazebo**: Drone simulation environment.
- **ArduPilot**: Flight control software.
- **MAVProxy & MAVLink (`pymavlink`)**: Communicates with the drone.
- **OpenCV (`cv2`)**: ARuco marker detection.
- **ROS (Robot Operating System)**: Manages drone control and data flow.
- **Python**: Core language for the implementation.
- **LiDAR Sensor (`sensor_msgs/LaserScan`)**: Obstacle detection.

## Project Structure

```
├── aruco_estimator.py  # Detects ARuco markers and estimates position
├── drone_controller.py  # Handles drone movement using MAVLink
├── navigation.py  # Manages navigation and alignment for landing
├── obstacle_detector.py  # Processes LiDAR data to detect obstacles
├── main.py  # Main execution script orchestrating drone operations
├── camera_calib_live.npz  # Calibration file for accurate ARuco detection
```

## Setup Instructions

### Prerequisites

Ensure the following are installed on your system:

- **Ubuntu 20.04 (WSL or Native)**
- **ROS (Noetic recommended)**
- **Gazebo 11**
- **ArduPilot with SITL (Software In The Loop)**
- **Python3** with dependencies:
  ```sh
  pip install opencv-python numpy pymavlink rospkg catkin-tools
  ```
- **MAVProxy**:
  ```sh
  sudo apt-get install mavproxy
  ```

### Running the Simulation

Start the Gazebo environment with the drone:

```sh
roslaunch gazebo_ros iris_runway.launch
```

Launch ArduPilot SITL:

```sh
/usr/bin/python3 ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --out=udp:localhost:14540 --out=udp:localhost:14550
```

Start MAVProxy:

```sh
mavproxy.py --master=udp:127.0.0.1:14550
```

Run the precise landing and obstacle avoidance script:

```sh
cd ~/catkin_ws/src/scripts
/usr/bin/python3 main.py
```

### Expected Behavior

1. The drone takes off and follows the waypoints.
2. If obstacles are detected via LiDAR, the drone avoids them.
3. Upon reaching the final waypoint, the drone detects the ARuco marker.
4. The drone aligns itself with the marker and lands precisely, with a maximum deviation of 7 cm.
5. The mission completes, and the drone disarms.

## Future Enhancements

- **Enhancing obstacle avoidance** with advanced path planning algorithms.
- **Multi-drone coordination** for efficient deliveries.
- **Real-world deployment** with a physical drone and camera module.

## Contributors

- **Mathew Mathew**

## License

This project is open-source and available under the MIT License.s
