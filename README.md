# 3D Position Estimation Using ArUco Markers

---

**Project status**: Early development stage
Multiple student teams are collaboratively building a networked system for marker and camera pose estimation.

<p align="center">
  <img src="PosePositionDetection.jpg" alt="Detection of Marker with Camerar" width="50%">
</p>

## Project Goal


The project aims to develop a system for estimating the 2D positions and orientations (poses) of multiple cameras in a shared global coordinate system using ArUco markers. Each camera is equipped with four rigidly mounted ArUco markers and is capable of:

- Detecting other ArUco markers in a 3D environment
- Estimating its own pose relative to detected markers
- Communicating pose information with other cameras via MQTT

A fixed reference cube with four ArUco markers defines the origin and orientation of the global coordinate system. The global camera poses are computed through mutual marker detection and transformation logic.


### Global Coordinate System (Top View)

- **X+** points forward  
- **X−** points backward  
- **Y+** points right  
- **Y−** points left  

These directional labels are used **exclusively** for the global coordinate system. Each camera may be arbitrarily rotated within this system.


### Camera Marker Placement (relative to camera body)

- **Marker N0**: front side (near the lens), faces in the viewing direction of the camera  
- **Marker N1**: right side, faces to the right (relative to the camera)  
- **Marker N2**: back side, faces opposite to the viewing direction  
- **Marker N3**: left side, faces to the left (relative to the camera)


### Reference Cube at the Origin

- Contains four fixed ArUco markers with IDs `0`, `1`, `2`, `3`  
- **Marker 0**: rear side, visible from the global X+ direction  
- **Marker 1**: left side, visible from global Y+ direction  
- **Marker 2**: front side, visible from global X− direction  
- **Marker 3**: right side, visible from global Y− direction  


### Detection and Transformation Logic

Each camera can detect markers from other cameras or from the reference cube. From these detections (rotation vector and translation vector), a relative transformation matrix between the camera and the marker can be computed. By incorporating knowledge about the marker's position and orientation on its carrier object (camera or cube), the camera's position and orientation can be derived in the global coordinate system.


### Initialization and Requirements

- At least one camera must detect one of the reference markers (`0`–`3`) to initialize the global coordinate system.  
- Other cameras can be localized through transitive connections by detecting markers from already localized cameras.  
- The system must account for the fact that each camera has its own local coordinate system, which may differ in orientation from the global system.


Each team uses an independent setup consisting of an ESP32-CAM (with OV2640 camera) and an ESP32 microcontroller. This repository belongs to **one team** within the larger multi-team project.


## Technologies Used

- **Languages**: Python (image processing), C++/Arduino (ESP32)
- **Libraries**:
  - `OpenCV` with `cv2.aruco` for marker detection
  - `paho-mqtt` or `umqtt` for MQTT communication
- **Hardware**:
  - ESP32-CAM with OV2640 camera module
  - 6x6 ArUco markers (maximum marker size: 3 cm)


## ArUco Marker Basics

- Each marker encodes a unique ID in a 6x6 bit pattern.
- A solid black border improves edge detection.
- Pose (translation and rotation) is calculated using the marker's corners, known size, and intrinsic camera parameters.

## Output of cv2.aruco Marker Detection

When using ArUco markers for pose estimation, `tvecs` and `rvecs` provide the position and orientation of a marker relative to the camera.

- **`tvecs = [tx, ty, tz]` -> position relative to camera**  
  Represents the translation of the marker relative to the camera center
  - `tx`: translation along the camera's X-axis (sideways)  
  - `ty`: translation along the camera's Y-axis (upwards)  
  - `tz`: translation along the camera's Z-axis (forwards)  
  The unit depends on the calibration, typically meters or centimeters.  

- **`rvecs = [rx, ry, rz]` -> orientation relative to camera**  
  Describes the rotation of the marker relative to the camera using axis-angle representation 
  - The direction of the vector indicates the rotation axis.  
  - The magnitude ‖rvec‖ is the rotation angle **in radians**.  
  This can be converted into a 3×3 rotation matrix using the Rodrigues formula.


## Task for Each Team

- Detect and localize multiple ArUco markers in space.
- Estimate the camera's own pose.
- Share this data via MQTT with other teams.


## Contributors

Multiple teams from the university courses **"Optical Sensor Systems"** and **"Real-Time Systems"** are collaborating on this project.  
This repository documents the progress of **one** of these teams.
