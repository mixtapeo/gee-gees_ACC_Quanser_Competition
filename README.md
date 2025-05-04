# Quanser Self-Driving Car Competition - University of Ottawa Team

## Project Overview
This repository contains the code developed for Stage 1 of the 2025 Quanser American Control Conference Self-Driving Car Student Competition.

Our system uses a combination of computer vision techniques and PID control to navigate lanes, detect traffic lights, and handle intersections.

The required packages were installed by following the instructions from the repository provided by Quanser at https://github.com/quanser/ACC-Competition-2025/blob/main/Software_Guides/ACC%20Software%20Setup%20Instructions.md

Due to time and resource constraints, some trade-offs and limitations exist, which are detailed below.

---

## Accomplishments

We have successfully implemented the following features:
- **Lane Detection**
- **Traffic Light Detection**
- **Stop Sign Detection**
- **Pick-up and Drop-off Functionality**

---

## Important Notes and Known Issues

- **Yield Sign Detection**:
  - Due to time constraints, we decided to completely ignore yield sign detection.

- **Traffic Light Detection**:
  - The car stops briefly at red lights.
  - However, the traffic light detection system is not fully reliable. Sometimes, a red light may be misclassified as a yellow light.

- **Depth Camera and Stop Sign Behavior**:
  - Due to low Cycles Per Second (CPS) during some runs, simulation behavior may vary.
  - In low CPS cases, the depth camera fails to return accurate measurements, causing the car to occasionally miss stopping at stop signs.

- **PID Tuning Strategy**:
  - Car starts the first trip using:
    - **kp = 0.3**, **kd = 0.15**
  - After passing the stop sign:
    - **kp = 0.45**, **kd = 0.1**
  - When turning right at a red light:
    - The turn is controlled using a `while` loop.
  - Approaching the roundabout:
    - PID values are adjusted to **kp = 0.47**, **kd = 0.15** for smoother navigation.

---

## Repository Structure

- `lane_detection_pid.py` — Lane detection and car control using PID.
- `traffic_light_detection.py` — Detection of red, yellow, and green lights using OpenCV.
- `stop_sign_detection.py` — Stop sign detection and stopping logic.
- `pickup_dropoff.py` — Pick-up and drop-off navigation logic.
- `README.md` — Project overview and notes.

---

## Acknowledgements

We thank Quanser and the organizing team for providing this opportunity to learn and innovate with autonomous vehicles.

---

## Contact

For questions or collaboration:
- **Team Name**: Gee-Gees
- **University**: University of Ottawa
- **Team Leader**: Mohammad Ahsan Akhlaque
- **Email**: aakhl097@uottawa.ca

---
