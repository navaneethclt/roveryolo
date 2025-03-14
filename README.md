# Position Estimation of a Rover Robot using 1D Lidar and Computer Vision
This project involves developing a real-time position and yaw angle estimation system for a ROS-based rover, achieving 2 cm positional accuracy and 5-degree yaw angle accuracy using a low-cost camera and a 1D Garmin LIDAR-Lite-v3.

A YOLOv7-based object detection algorithm is implemented to enable active orientation control of the LIDAR, improving its measurement precision. The system processes real-time image data from a camera using OpenCV, detects objects with YOLOv7, and dynamically adjusts the LIDAR orientation based on detection results.

A low-pass Butterworth filter is applied to smooth sensor data, and serial communication with an external microcontroller is managed via the ArduSerial interface. The software operates with real-time constraints on a Windows system, utilizing high-priority scheduling and performance counter-based timing control for consistent execution.

The implementation includes:

-Efficient data filtering for stable sensor readings.

-Serial communication protocols for interfacing with the rover.

-YOLO-based detection pipeline for robust object recognition and orientation adjustment.

-Multi-threaded camera processing to optimize real-time performance.


This system enhances autonomous navigation capabilities by integrating machine learning-driven perception with sensor fusion for precise state estimation and control.
