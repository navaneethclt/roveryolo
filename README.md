# roveryolo
YOLO v7 Object Detection and Tracking  


This project integrates computer vision (YOLO model) and serial communication with an Arduino for robotic or sensor data collection and processing, particularly with a LIDAR sensor. It uses filters to smooth data inputs and outputs motor control commands based on sensor readings and object detection. The code also logs data and captures frames from a connected camera for further analysis.

Requirements
Software
Operating System: Windows
Libraries:
OpenCV (opencv2/opencv.hpp)
YOLO Model for object detection
Serial communication with Arduino (ArduSerial.h)
IIR Filter for signal processing (Iir.h)
Command-line argument parsing (cmdline.h)
Utility functions (utilsm.h)
Hardware
Arduino connected via serial communication
LIDAR sensor
Camera (used for YOLO object detection)
Code Components
Key Functionalities
Filtering:

Low-pass Butterworth filter (Iir::Butterworth::LowPass<order>) is used to smooth control inputs, reducing noise in the system.
Setup is defined by a cutoff frequency and sampling rate.
Object Detection:

YOLO object detection using a pre-trained ONNX model. The code detects objects in a camera feed and calculates control inputs based on the detected object’s position.
Serial Communication:

Interfacing with Arduino to send motor control commands (loop3) and read LIDAR sensor data (loop4read).
Data from the LIDAR is processed to find the median value of the readings.
Data Logging:

Logs theta values, filtered theta, LIDAR readings, and other sensor data into a CSV file (yolov7_ver78.csv).
Performance Handling:

Real-time performance with high-priority threading (SetPriorityClass).
Custom sleep function (sleepfor) to handle precise timing using Windows performance counters.
Main Loop
The main loop captures frames from the camera, applies YOLO detection, and computes control inputs based on the detected object’s position. The detected object's position is used to control a robotic system via Arduino. The filtered values of the control commands are sent to the Arduino for motor control, while the LIDAR readings are processed and logged.

Command-Line Arguments
--model_path: Path to the YOLO ONNX model file.
--image: Image source for detection.
--class_names: File with object class names (used by YOLO).
--gpu: Flag for enabling GPU-based inference.
Helper Functions
findMode: Finds the mode of a set of integer values.
findMedian: Finds the median value from a vector of integers.
type2str: Converts an OpenCV matrix type to a human-readable string format.
sleepfor: Pauses the execution until a specified amount of performance clicks have passed.
How to Use
Setup:

Ensure that OpenCV, YOLO, and other required libraries are installed.
Connect an Arduino and configure serial communication.
Attach a LIDAR sensor and camera.
Compile and Run:

Build the code in an environment that supports the necessary libraries (e.g., Visual Studio).
Run the program with the necessary command-line arguments:
css
Copy code
./program.exe --model_path yolov5.onnx --image input.jpg --class_names coco.names --gpu
Control:

The system will process the camera feed and send control commands to the Arduino based on detected objects and LIDAR readings.
Notes
The code expects to operate on a real-time system and may not perform as expected on lower-priority settings.
The LIDAR readings are processed to ensure smooth control of the system, while YOLO detects objects in real-time for decision-making.
