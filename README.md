# turtlebot4-traffic-light-detector-opencv-ros2

This ROS 2 project enables a TurtleBot4 to intelligently interact with its environment through:
-Traffic light detection (red, yellow, green) using an OAK-D Pro camera and OpenCV
-Real-time obstacle detection using a RPLIDAR
The goal is for the robot to stop at red lights, move at green, and safely avoid obstacles during navigation.

🚀 Features
-Reliable traffic light detection using the OAK-D Pro RGB stream
-360° obstacle detection with RPLIDAR
-Real-time image processing using OpenCV
-ROS 2 integration (nodes, topics)
-Visualization available with rqt_graph

🧰 Technologies Used
-ROS 2 Humble
-OpenCV (Python)
-OAK-D Pro 
-RPLIDAR A1M8
-TurtleBot4
-rclpy (Python ROS 2 client)

🧠 Robot Behavior
🔴 Red light detected → The robot stops
🟡 Yellow light detected → The robot slows down
🟢 Green light detected → The robot moves forward
🚧 Obstacle detected by LIDAR → The robot slows down, then stops
