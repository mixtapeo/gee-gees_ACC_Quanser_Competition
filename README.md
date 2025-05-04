# Quanser Self-Driving Car Competition - University of Ottawa Team

## Project Overview
This repository contains the code developed for Stage 1 of the 2025 Quanser American Control Conference Self-Driving Car Student Competition.

Our system uses a combination of computer vision techniques and PID control to navigate lanes, detect traffic lights, and handle intersections.

Due to time and resource constraints, some trade-offs and limitations exist, which are detailed below.


## Software Setup

The required packages were installed by following the instructions from the repository provided by Quanser at https://github.com/quanser/ACC-Competition-2025/blob/main/Software_Guides/ACC%20Software%20Setup%20Instructions.md

Download the 'mycar' zip folder in the directory path of ACC_Development/Development/ros2/src. Unzip the folder's contents after ensuring it's next to the 'qcar2_interfaces' and 'qcar2_nodes' packages. Please open up three terminals and enter the following commands.

- **First Terminal**

    - cd /home/$USER/Documents/ACC_Development/docker/virtual_qcar2
    - sudo docker run --rm -it --network host --name virtual-qcar2 quanser/acc2025-virtual-qcar2 bash
    - cd /home/qcar2_scripts/python
    - python3 Base_Scenarios_Python/Setup_Real_Scenario.py
    - Ensure the message 'Starting Traffic Light Sequence' pops up before executing the commands in the subsequent terminals. 

- **Second Terminal**

    - cd /home/$USER/Documents/ACC_Development/isaac_ros_common
    - ./scripts/run_dev.sh  /home/$USER/Documents/ACC_Development/Development
    - colcon build 
    - install/setup.bash
    - ros2 launch qcar2_nodes qcar2_virtual_launch.py

- **Third Terminal**

    - Copy and paste the first four commands from the second terminal in the third terminal. The third terminal is attached to the Isaac docker container running in the second terminal. 
    - Go to the path of the python script by entering 'cd ros2/src/mycar/src'.
    - Execute the python script by entering: 'python3 uottawa_qcar2_code.py'.

- **Important Notes**
 
     - It should be noted that the frame rate was set to 30 fps to ensure the camera always sends the best quality images. This contributed to a low CPS rate of only around 10. Due to low CPS, the algorithm may not work as expected multiple times. On our end, the simulation worked after around 7 failed trials. Occasionally, you may observe the following behaviours:
     - Car stopping at the stop sign a bit early or a bit far from the mark due to depth camera glitching. In other words, the car is not stopping near the mark the way it is in the video.
     - Car not stopping at the stop sign as programmed.
     - Car going straight after the stop sign instead of turning left.
     - Car turning right a bit early at the first traffic light. In other turns, the right turn is not smooth and the car may be seen being driven on white path.
     - Car rotating in circles at the beginning before the first yield sign.
     - Car trying to turn left at the second traffic light.
     - In all above cases, the simulation must be restarted. Alternatively, if the car's behaviour deviates from the behaviour shown in the video, then the simulation must be restarted.
     - The yield sign does get detected by the car and the message would pop up in the terminal. However, this depends on the angular view of the depth camera after the car turns right at the the first traffic light. 
---

## Accomplishments

We have successfully implemented the following features:
- **Lane Detection**
- **Traffic Light Detection**
- **Stop Sign Detection**
- **Pick-up and Drop-off Functionality**

---

## Tuning the Parameters of PD Controller
  - Car starts the first trip using:
    - **kp = 0.3**, **kd = 0.15**
  - After passing the stop sign:
    - **kp = 0.45**, **kd = 0.1**
  - When turning right at a red light:
    - The turn is controlled using a `while` loop.
  - Approaching the roundabout:
    - PD values are adjusted to **kp = 0.47**, **kd = 0.15** for smoother navigation.

---

## Repository Structure

- `uottawa_qcar2_code.py`
        -  Lane, stop sign, yield sign, and traffic light detection; passenger pickup and dropoff; PD steering control.
- `README.md` — Project overview and notes.

---

## Acknowledgements

We thank Quanser and the organizing team for providing this opportunity to learn and innovate with autonomous vehicles.

---

## Contact

For questions or collaboration:
- **Team Name**: Gee-Gees
- **University**: University of Ottawa
  
- **Team Captain**: Mohammad Ahsan Akhlaque
- **Email**: aakhl097@uottawa.ca

- **Team Member**: Kishita Pakhrani
- **Email**: kpakhran@uottawa.ca

- **Team Member**: Jonathan Horton
- **Email**: jhort062@uottawa.ca

---
