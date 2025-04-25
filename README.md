# 3D Position Estimation Using ArUco Markers

**Project status**: Early development stage
Multiple student teams are collaboratively building a networked system for marker and camera pose estimation.

## Project Goal

This project aims to develop a system for **3D position and pose estimation of multiple cameras** using **ArUco markers**. Each camera unit should be able to:

- **Detect multiple ArUco markers in a 3D environment**
- **Estimate its own position and orientation (pose)**
- **Exchange pose data with other cameras via MQTT**

Each team uses an independent setup consisting of an ESP32-CAM (with OV2640 camera) and an ESP32 microcontroller. This repository belongs to **one team** within the larger multi-team project.

---

## Technologies Used

- **Languages**: Python (image processing), C++/Arduino (ESP32)
- **Libraries**:
  - `OpenCV` with `cv2.aruco` for marker detection
  - `paho-mqtt` or `umqtt` for MQTT communication
- **Hardware**:
  - ESP32-CAM with OV2640 camera module
  - 5x5 ArUco markers (maximum marker size: 3 cm)

---

## ArUco Marker Basics

- Each marker encodes a unique ID in a 5x5 bit pattern.
- A solid black border improves edge detection.
- **Pose (translation and rotation)** is calculated using the marker's corners, known size, and intrinsic camera parameters.

Example of a 5x5 ArUco marker:

![Example ArUco Marker]5x5ArucoMarker.jpeg

---

## Task for Each Team

- Detect and localize multiple ArUco markers in space.
- Estimate the camera's own pose.
- Share this data via MQTT with other teams.

---

## Contributors

Multiple teams from the university courses **"Optical Sensor Systems"** and **"Real-Time Systems"** are collaborating on this project.  
This repository documents the progress of **one** of these teams.
