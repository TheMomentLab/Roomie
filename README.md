![Banner](https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/banner.png?raw=true)

<p align="center">
  <a href="https://www.apache.org/licenses/LICENSE-2.0">
    <img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg?style=for-the-badge" alt="License">
  </a>
  <a href="https://docs.google.com/presentation/d/1ov9HC-qspBt8EVuyY66NAY0iMZ1nOyS5X9acsZ7iivs/edit?usp=sharing">
    <img src="https://img.shields.io/badge/PRESENTATION-GoogleSlides-yellow?style=for-the-badge&logo=google-slides&logoColor=white" alt="Presentation Slides">
  </a>
  <a href="https://www.youtube.com/playlist?list=PLeVDEKHes6sHO5c1vp_Hu00HwNrdS69pk">
    <img src="https://img.shields.io/badge/DEMO-YouTube-red?style=for-the-badge&logo=youtube&logoColor=white" alt="Demo Video">
  </a>
</p>

# 📚 Table of Contents

- [1. Team](#1-team)
- [2. Project Overview](#2-project-overview)
- [3. Key Features](#3-key-features)
- [4. Core Technologies](#4-core-technologies)
- [5. Technical Challenges and Solutions](#5-technical-challenges-and-solutions)
- [6. System Design](#6-system-design)
- [7. Project Structure](#7-project-structure)
- [8. Tech Stack](#8-tech-stack)
- [9. Project Management](#9-project-management)
- [10. License](#10-license)

> 📄 Looking for the Korean version? See [`README.ko.md`](README.ko.md).

---

# 1. Team

## 🧑‍💼 Jinhyeok Jang [`@jinhyuk2me`](https://github.com/jinhyuk2me)
- Project planning and overall lead
- System architecture and ROS2 package design
- System scenario and FSM design
- Vision AI model development and Vision Service implementation
- ROS2 × PyQt-based Robot GUI implementation

## 🧑‍💼 Jiyeon Kim [`@heyjay1002`](https://github.com/heyjay1002)
- Backend development and database design
- micro-ROS-based IO controller implementation
- HTTP/WebSocket × PyQt Admin GUI
- Robot hardware fabrication

## 🧑‍💼 Jongmyeong Kim [`@jongbob1918`](https://github.com/jongbob1918)
- FreeRTOS-based robot arm control
- HTTP/WebSocket × HTML/JS/CSS Guest GUI
- HTTP/WebSocket × PyQt Staff GUI
- Jira schedule management
- Robot hardware fabrication

## 🧑‍💼 Hyojin Park [`@Park-hyojin`](https://github.com/Park-hyojin)
- Robot SLAM & navigation lead
- Static/dynamic obstacle-handling algorithm design and implementation
- Path creation and driving behaviors
- System integration plus delivery, guidance, and inter-floor travel features

---

# 2. Project Overview

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/intro_delivery_image.png?raw=true" height="200"><br>
        <sub>Room Service Delivery</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/intro_escort_image.png?raw=true" height="200"><br>
        <sub>Wayfinding Escort</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/intro_elevator_image.png?raw=true" height="200"><br>
        <sub>Inter-floor Travel</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/intro_admin_image.png?raw=true" height="200"><br>
        <sub>Admin Monitoring</sub>
      </td>
    </tr>
  </table>
</div>

- **Project Goal**
  - Let the robot autonomously take over repetitive hotel operations so that staff workload is reduced and guests receive a novel, convenient service.
  
- **Project Timeline**
  - July 7, 2025 – August 13, 2025 (38 days)

---

# 3. Key Features

## 🍽️ Room Service Delivery

<div align="center">
  <table>
    <tr>
      <th style="width:15%">Key Stages</th>
      <th style="width:80%">Description</th>
      <th style="width:30%">Media</th>
    </tr>
    <tr>
      <td valign="top">Order Placement</td>
      <td valign="top">
        ▪ Guests open the Guest GUI via the QR code placed in each room.<br>
        ▪ After reviewing the menu, the order is submitted → the Staff GUI receives the notification.<br>
        ▪ Once the dish is ready, the Staff GUI sends a “Pickup Request.”
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery-qr.gif?raw=true" width="130">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery-order.gif?raw=true" width="130">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Pickup & Loading</td>
      <td valign="top">
        ▪ Roomie drives to the restaurant pickup waypoint.<br>
        ▪ ArUco marker detection aligns the robot precisely with the pickup spot.<br>
        ▪ The Robot GUI shows the order list to prevent loading mistakes.<br>
        ▪ Drawer control includes door open/lock sensors and a load-presence sensor.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/pickup_new_1.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/pickup_new_2.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/pickup_new_3.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery-staffgui.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/pickup_new_4.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">In-room Delivery</td>
      <td valign="top">
        ▪ Nav2-based navigation drives the robot to the room entrance.<br>
        ▪ The destination ArUco marker confirms the exact door location.<br>
        ▪ Arrival notifications are sent through both the Guest GUI and the Robot GUI.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery_new_1.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery_new_2.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery-finalcheck.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Item Handover</td>
      <td valign="top">
        ▪ Guests operate the Robot GUI to unlock the drawer and take the order.<br>
        ▪ The robot returns to the standby area once the task is complete.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/delivery-return.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
  </table>
</div>

---

## 🧭 Guided Wayfinding Service

<div align="center">
  <table>
    <tr>
      <th style="width:15%">Key Stages</th>
      <th style="width:80%">Description</th>
      <th style="width:30%">Media</th>
    </tr>
    <tr>
      <td valign="top">Request & Destination Input</td>
      <td valign="top">
        ▪ Destination is auto-filled when the guest authenticates with the room card.<br>
        ▪ Manual input is also available from the Guest GUI or Robot GUI.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/escort_1.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Guest Identification</td>
      <td valign="top">
        ▪ The rear camera detects the guest to be escorted.<br>
        ▪ A DeepSORT-based target tracking algorithm follows the identified guest.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/escort_2.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Guided Escort</td>
      <td valign="top">
        ▪ The robot keeps a safe distance while guiding the guest to the destination.<br>
        ▪ If the guest leaves the field of view, the robot pauses.<br>
        ▪ The mission resumes automatically once the guest is detected again.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/escort_3.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
  </table>
</div>

---

## 🏢 Inter-floor Travel (Elevator Integration)

<div align="center">
  <table>
    <tr>
      <th style="width:15%">Key Stages</th>
      <th style="width:80%">Description</th>
      <th style="width:30%">Media</th>
    </tr>
    <tr>
      <td valign="top">Elevator Call</td>
      <td valign="top">
        ▪ The Vision Service extracts button coordinates.<br>
        ▪ The Arm Controller presses the lobby call button.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-first.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-alignbutton.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-pushouterbutton2.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Boarding & Interior Interaction</td>
      <td valign="top">
        ▪ The robot centers itself with the door before boarding.<br>
        ▪ Arm motion is driven by the size and coordinates of the floor buttons.<br>
        ▪ OCR on the overhead display confirms arrival at the target floor.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-afterpushing.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-opendoor.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator_entering.gif?raw=true" width="300"><br>
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-pushinnerbutton.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Exit</td>
      <td valign="top">
        ▪ After arriving at the destination floor, the robot centers itself and exits safely.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-last.gif?raw=true" width="300">
        </p>
      </td>
    </tr>
  </table>
</div>

---

## 📊 Admin Monitoring

<div align="center">
  <table>
    <tr>
      <th style="width:15%">Key Stages</th>
      <th style="width:80%">Description</th>
      <th style="width:30%">Media</th>
    </tr>
    <tr>
      <td valign="top">Dashboard</td>
      <td valign="top">
        ▪ Track the number of active jobs and robots in real time.<br>
        ▪ Draw robot positions on the 2D map.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/staffgui-dashboard.gif?raw=true" width="400">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Robot Management</td>
      <td valign="top">
        ▪ Monitor each robot’s position, task assignment, and battery level.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/staffgui-status.gif?raw=true" width="400">
        </p>
      </td>
    </tr>
    <tr>
      <td valign="top">Job History</td>
      <td valign="top">
        ▪ Review job lists and per-task logs.
      </td>
      <td valign="top">
        <p align="center">
          <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/staffgui-log.gif?raw=true" width="400">
        </p>
      </td>
    </tr>
  </table>
</div>

---

# 4. Core Technologies

## 1) Robot Arm Control

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/arm-principle.png?raw=true" height="200"><br>
        <sub>Robot arm control principle</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/elevator-pushouterbutton2.gif?raw=true" height="200"><br>
        <sub>Button press with the arm</sub>
      </td>
    </tr>
  </table>
</div>

<p align="center">
  <a href="ros2_ws/src/roomie_ac/README.md">View the detailed design</a>
</p>

- **Hardware**
  - Servo motors
  - 2D camera
  - Button-click end effector
- **Button detection**
  - Compute base → wrist → camera → button coordinates
- **Motion sequence**
  - Observation pose → pre-push pose → button press → confirmation
- **Control method**
  - Apply a Gaussian velocity/acceleration profile to minimize jitter

---

## 2) Path Planning and Navigation

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/slam_mapping.png?raw=true" height="200"><br>
        <sub>Path creation</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/mobile-staticobstacle.gif?raw=true" height="200"><br>
        <sub>Static obstacle avoidance</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/mobile-dynamicobstacle.gif?raw=true" height="200"><br>
        <sub>Dynamic obstacle avoidance</sub>
      </td>
    </tr>
  </table>
</div>

- **Nav2-based navigation**
  - Generate and follow global/local paths
- **Waypoint-driven path creation**
  - Match depth-camera obstacles to pre-defined waypoints
  - Use the **A\*** algorithm to compute the optimal route
- **Dynamic obstacle handling**
  - Detect obstacles via the depth camera in real time
  - Stop within a threshold distance and resume once the path clears
- **RTR motion (Rotate–Translate–Rotate)**
  - Provides precise alignment and backward motion inside elevators

---

## 3) Vision Perception

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/vision-obstacle.gif?raw=true" height="200"><br>
        <sub>Obstacle detection while driving</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/vision-elevator.gif?raw=true" height="200"><br>
        <sub>Elevator exterior perception</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/vision-tracking.gif?raw=true" height="200"><br>
        <sub>DeepSORT-based guest tracking</sub>
      </td>
    </tr>
  </table>
</div>

- **YOLOv8n object detection**
  - Obstacles: static, dynamic, glass doors
  - Elevators: buttons, displays, doors, direction indicators
- **Accuracy boosters**
  - CNN classifies detailed button types
  - EasyOCR reads the floor indicator
- **Person tracking**
  - YOLOv8n detects people
  - DeepSORT tracks a specific guest and publishes coordinates

---

## 4) micro-ROS IO Controller

<div align="center">
  <table>
    <tr>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/microros-2.jpg?raw=true" height="300"><br>
        <sub>Ultrasonic sensors for load and door detection</sub>
      </td>
      <td align="center">
        <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/dooropening.gif?raw=true" height="300"><br>
        <sub>Drawer control via micro-ROS</sub>
      </td>
    </tr>
  </table>
</div>

- **Drawer-door detection**
  - Measure the distance between the sensor and door
  - If the distance exceeds 5.0 cm, the drawer is considered open
- **Load detection**
  - Side-mounted sensors measure the internal width
  - If the distance is below 25.0 cm, cargo is detected
- **RFID card reader**
  - MFRC522 reads the UID of each RFID card
  - Interprets the 4-byte value stored in block 4 as `location_id`
  - Publishes `success=true` + the location value on success; `success=false`, `location_id=-1` otherwise
- **RGB LED status**
  - Control the LED color based on the `RobotState`
  - <details>
    <summary>💡 View the control logic</summary><br>

    | State ID | State name | RGB LED |
    |---|---|---|
    | 0 | `INITIAL` | Cyan |
    | 1, 2, 11, 13, 21, 23 | `CHARGING`, `WAITING`, `PICKUP_WAITING`, `DELIVERY_WAITING`, `GUIDE_WAITING`, `DESTINATION_SEARCHING` | Green |
    | 10, 12, 20, 22, 30, 31 | `PICKUP_MOVING`, `DELIVERY_MOVING`, `CALL_MOVING`, `GUIDE_MOVING`, `RETURN_MOVING`, `ELEVATOR_RIDING` | Blue |
    | 90 | `ERROR` | Red |

    </details>

---

# 5. Technical Challenges and Solutions

### 🤖 Arm Vibration
- **Problem**: Constant-velocity control created small jitter at the end effector.  
- **Solution**: Applied Gaussian velocity/acceleration profiles to smooth out the motion.  

### 🛣️ Path Planning in Narrow Indoor Spaces
- **Problem**: The robot could not re-route when corridors were narrow or blocked.  
- **Solution**: Added waypoint-based detours and ran the A\* algorithm to compute bypass paths in advance.  

### 🧠 Single YOLO Model Limitations

<div align="center">

<table>
  <tr>
    <td align="center">
      <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/vision-button.gif?raw=true" height="200"><br>
      <sub>YOLOv8n + CNN pipeline for buttons</sub>
    </td>
    <td align="center">
      <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/vison-elevator2.gif?raw=true" height="200"><br>
      <sub>YOLOv8n + EasyOCR pipeline for floors</sub>
    </td>
  </tr>
</table>

</div>

- **Problem**: Adding more classes to YOLO degraded accuracy.  
- **Solution**: YOLOv8n generates the ROI, a CNN classifies the button, and EasyOCR interprets the floor indicator.  

### 🏢 Precise Elevator Maneuvers
- **Problem**: Plain Nav2 navigation struggled with button pressing and precise alignment.  
- **Solution**: Introduced RTR (Rotate–Translate–Rotate) patterns to enable fine alignment and backward motion.  

---

# 6. System Design

<details>
<summary>User Requirements</summary>

```
[Priority Legend]
- `R` : Required implementation 
- `O` : Optional implementation  
```

| UR_ID | UR_NAME | UR Description | Condition | Required |
|-------|---------|----------------|-----------|----------|
| **Guest** |||||
| UR_01 | Call the robot | Request the robot to move to a specific location | Callable from:<br>▪ Lobby<br>▪ Guest room<br>▪ Restaurant | O |
| UR_02 | Guided escort | The robot guides the guest to a destination while carrying luggage | Supported locations:<br>▪ Guest room<br>▪ Lobby<br>▪ Restaurant | O |
| UR_03 | Personalized responses | Provide greetings in the guest’s preferred language | Triggered when:<br>▪ Guidance ends<br>▪ Delivery handover completes | O |
| UR_04 | Deliver amenities | Deliver requested items to the room | Items:<br>▪ **Food & beverage**: spaghetti, pizza<br>▪ **Supplies**: toothbrush, towel, bottled water, cutlery | R |
| UR_05 | Real-time progress tracking | Display the status of each requested job | Includes:<br>▪ Processing state<br>▪ Current position<br>▪ Estimated arrival time | R |
| UR_06 | Guest notifications | Notify the guest about job progress | Cases:<br>▪ Robot call: assigned, departed, arrived<br>▪ Guidance: started, finished<br>▪ Delivery: pickup arrival, pickup done, delivery arrival, received<br>▪ Failure alerts with reasons (blocked path, guest lost, collision, etc.) | R |
| **Administrator** |||||
| UR_07 | Job status management | Monitor every ongoing job | Includes:<br>▪ Current status<br>▪ Job ID<br>▪ Job type<br>▪ Failure indicator and reasons | O |
| UR_08 | Job history | Browse the entire job history | Filters:<br>▪ Job type<br>▪ Status<br>▪ Guest ID<br>▪ Room number | O |
| UR_09 | Job priority control | Reorder queued jobs | - | O |
| UR_10 | Robot information | Maintain robot-specific metadata | Fields:<br>▪ Robot ID<br>▪ Model name<br>▪ Manufacture date | O |
| UR_11 | Robot status | Track the current state of each robot | Fields:<br>▪ Location<br>▪ Battery level<br>▪ Charging state<br>▪ Assigned job ID<br>▪ System errors | O |

</details>

<details>
<summary>System Requirements</summary>

```
[Priority Legend]
- `R` : Required implementation 
- `O` : Optional implementation  
```

| SR_ID | SR_NAME | SR Description | Condition | Priority |
|-------|---------|----------------|-----------|----------|
| SR_01 | Robot call | Call the robot to a specific location | Available at:<br>- Room entrance (ROOM_XX)<br>- Restaurant (RES_2)<br>- Lobby (LOB_2) | R |
| SR_02 | Autonomous movement | Robots travel autonomously to execute or finish jobs | Job types:<br>- Call<br>- Guidance<br>- Delivery<br>- Food & beverage<br>- Amenities | R |
| SR_02_01 | Path creation | Robot generates its own route to the target | - | R |
| SR_02_02 | Obstacle avoidance | Detect and avoid obstacles while driving | Obstacles:<br>- Static: tables, chairs, trash bins<br>- Dynamic: people | R |
| SR_02_03 | Collision detection | Pause when a collision is detected | Determine collisions via IMU thresholds | R |
| SR_02_04 | Tip-over detection | Detect rollovers and alert an admin | Determine tip-over via IMU thresholds | O |
| SR_02_05 | Following confirmation | Make sure the guest is following during guidance | - | R |
| SR_03 | Inter-floor travel | Use the elevator by calling it and pushing buttons | - | R |
| SR_03_01 | Elevator call | Summon the elevator to the current floor | Methods:<br>- API call<br>- Physical manipulation with the arm | R |
| SR_03_02 | Floor selection | Select the target floor after boarding | Methods:<br>- API call<br>- Physical manipulation with the arm | R |
| SR_03_03 | Elevator boarding | Board when the elevator arrives | Factors:<br>- Direction<br>- Position<br>- Door state | R |
| SR_03_04 | Elevator exit | Exit when reaching the destination floor | Factors:<br>- Position<br>- Door state | R |
| SR_04 | In-job notifications | Notify guests while the job is running | Provide status updates per call/guidance/delivery and include failure reasons | R |
| SR_05 | Personalized responses | Play multilingual voice prompts at the start/end of tasks | Cases:<br>- Guidance start<br>- Call arrival<br>- Guidance end<br>- Delivery handover | R |
| SR_06 | Guidance request | Start guidance after reading the guest card key | Available at:<br>- Room entrance<br>- Restaurant<br>- Lobby | R |
| SR_06_01 | Guest appearance recognition | Detect the guest’s appearance for tracking | Use the camera | O |
| SR_06_02 | Destination input | Provide multiple destination input methods | In-room: auto-filled from card / manual / voice / touchscreen<br>Elsewhere: restaurant / lobby | O |
| SR_07 | Delivery request | Request item delivery from the room | Delivery types:<br>- Food (spaghetti, pizza)<br>- Amenities (toothbrush, towel, bottled water, cutlery) | O |
| SR_08 | Load items | Staff load items at the pickup station | Capacity: up to two rooms | O |
| SR_08_01 | Load confirmation | Verify the items before departure | Flow:<br>- IR sensor pre-check → staff “Load Confirm” → departure countdown | R |
| SR_09 | Delivery tracking | Provide real-time delivery status to guests | Includes:<br>- Progress stage<br>- Current position<br>- ETA | O |
| SR_10 | Job data management | Manage job types and statuses | Track call/guidance/delivery lifecycle | R |
| SR_10_01 | Job history lookup | Allow admins to query all jobs | Fields:<br>- ID<br>- Type<br>- Status | R |
| SR_10_02 | Job monitoring | Provide job information to staff in real time | - | R |
| SR_10_03 | Job reorder | Manually change the queue order | - | R |
| SR_10_04 | Auto dispatch | Auto-assign queued jobs to idle robots | - | O |
| SR_11 | Auto return | Return to the lobby after jobs | Conditions:<br>- Return on completion/cancellation<br>- Move to the charger when needed | O |
| SR_12 | Robot info management | Manage robot ID, model, manufacture date | - | R |
| SR_12_01 | Robot lookup | Let admins filter/search the robot list | - | R |
| SR_13 | Robot state management | Manage per-robot state | Position, battery, charging state, job ID, error | R |
| SR_13_01 | Robot state monitoring | Provide real-time state data to admins | - | R |
| SR_13_02 | Collision alerts | Notify admins when collisions occur | Linked to SR_02_03 | R |
| SR_13_03 | State history | Review charging and collision logs | Charging ID/time, collision location/time | R |
| SR_14 | Auto charging | Auto-dock based on battery level | Docking-station charging | R |
| SR_14_01 | Low-battery return | Return to standby when battery <20% | - | R |

</details>

<details>
<summary>System Scenario</summary>
  
📄 [View the scenario PDF](https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/docs/system_scenario.pdf)
  
</details>

<details>
<summary>System Architecture</summary>

<p align="center">
  <img src="https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/assets/images/systemarchtecture_diagram.png?raw=true" alt="System architecture diagram" width="80%"><br>
  <sub>An overview of robots, GUIs, servers, and their communication flows</sub>
</p>

```
Elevators are notorious for unstable or non-existent network connectivity.
To stay reliable, Roomie embeds the Vision Service on the robot (on-device AI) so that buttons, doors, and floor indicators can be recognized offline.
```
  
</details>

<details>
<summary>State Diagram</summary>

<p align="center">
  <img src="assets/images/state_diagram.png" alt="State diagram" width="80%"><br>
  <sub>The robot workflow modeled as state transitions</sub>
</p>
  
</details>

<details>
<summary>Interface Specification</summary>

📄 [Open the interface specification (PDF)](https://github.com/addinedu-ros-9th/ros-repo-2/blob/main/docs/interface_specification.pdf)
  
</details>

<details>
<summary>ER Diagram (Entity Relationship Diagram)</summary>

<p align="center">
  <img src="assets/images/erd.png" alt="ER diagram" width="80%"><br>
  <sub>Database tables and relationships used in the system</sub>
</p>
  
</details>

<details>
<summary>Test Map</summary>

<p align="center">
  <img src="assets/images/test_map.png" alt="Test map" width="80%"><br>
  <sub>Indoor map created for navigation and feature verification</sub>
</p>
  
</details>

---

# 7. Project Structure

```
Roomie/
├── ros2_ws/                            # Shared ROS2 workspace
│   ├── build/                          # Created by colcon build
│   ├── install/
│   ├── log/
│   └── src/
│       ├── micro_ros_setup/            # micro-ROS build tools
│       ├── roomie_msgs/                # Shared messages (msg/srv/action)
│       ├── roomie_rc/                  # Robot Controller node (RC)
│       ├── roomie_rgui/                # Robot GUI node (RGUI)
│       ├── roomie_vs/                  # Vision Service node (VS)
│       ├── roomie_rms/                 # Main server node (RMS)
│       ├── roomie_agui/                # Admin GUI node (AGUI)
│       ├── roomie_ac/                  # Arm Controller node (AC)
│       └── bringup/                    # Integrated launch files
│
├── esp32_firmware/                     # ESP32 firmware for micro-ROS
│   ├── arm_unit/                       # Servo control firmware for the arm
│   │   └── src/
│   └── io_controller/                  # Sensor, drawer, LED control
│       └── src/
│
├── gui/                                # Non-ROS GUI apps
│   ├── staff_gui/                      # Staff GUI
│   └── guest_gui/                      # Guest GUI
│
├── assets/                             # Images and resources
│   └── images/
│
├── docs/                               # Design documents
│   ├── architecture/                   # System architecture
│   ├── interface.md                    # Communication interface definitions
│   └── state_diagram/                  # State diagram
│
├── .gitignore
├── README.md
└── LICENSE
```

---

# 8. Tech Stack

| Category | Technologies |
|----------|--------------|
| **ML / DL** | [![PyTorch](https://img.shields.io/badge/PyTorch-E34A6F?style=for-the-badge&logo=pytorch&logoColor=white)](https://pytorch.org/) ![CNN](https://img.shields.io/badge/CNN-1E90FF?style=for-the-badge) [![YOLO](https://img.shields.io/badge/YOLO-FFB400?style=for-the-badge&logo=yolov5&logoColor=black)](https://github.com/AlexeyAB/darknet) [![DeepSORT](https://img.shields.io/badge/DeepSORT-800080?style=for-the-badge&logo=github&logoColor=white)](https://github.com/nwojke/deep_sort) ![OCR](https://img.shields.io/badge/OCR-4682B4?style=for-the-badge) [![OpenCV](https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/) |
| **GUI** | [![PyQt](https://img.shields.io/badge/PyQt-41CD52?style=for-the-badge&logo=qt&logoColor=white)](https://riverbankcomputing.com/software/pyqt/intro) [![JavaScript](https://img.shields.io/badge/JavaScript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black)](https://developer.mozilla.org/en-US/docs/Web/JavaScript) [![HTML](https://img.shields.io/badge/HTML-E34F26?style=for-the-badge&logo=html5&logoColor=white)](https://developer.mozilla.org/en-US/docs/Web/HTML) [![CSS](https://img.shields.io/badge/CSS-1572B6?style=for-the-badge&logo=css3&logoColor=white)](https://developer.mozilla.org/en-US/docs/Web/CSS) |
| **Network & Protocol** | [![UDP](https://img.shields.io/badge/UDP-0088cc?style=for-the-badge&logo=wifi&logoColor=white)](https://en.wikipedia.org/wiki/User_Datagram_Protocol) [![HTTP](https://img.shields.io/badge/HTTP-E34F26?style=for-the-badge&logo=fastapi&logoColor=white)](https://developer.mozilla.org/en-US/docs/Web/HTTP) [![WebSocket](https://img.shields.io/badge/WebSocket-3D9BE9?style=for-the-badge&logo=socketdotio&logoColor=white)](https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API) |
| **Robotics** | [![ROS2](https://img.shields.io/badge/ROS2-22314E?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/foxy/index.html) ![ikpy](https://img.shields.io/badge/ikpy-6A5ACD?style=for-the-badge) ![FreeRTOS](https://img.shields.io/badge/FreeRTOS-007ACC?style=for-the-badge) ![Nav2](https://img.shields.io/badge/Nav2-D33825?style=for-the-badge) |
| **Environment** | [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/) [![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)](https://isocpp.org/) [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/) [![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)](https://www.kernel.org/) |

---

# 9. Project Management

## 1. Schedule Management

<table>
  <tr>
    <td align="center" width="400" valign="top">
      <img src="assets/images/jira-1.gif" width="500"><br>
      <img src="assets/images/jira-2.gif" width="500">
    </td>
    <td align="left" valign="top">
      ▪ Managed via Jira with six sprints.<br>
      ▪ Organized the backlog with Epic → Task hierarchies.
    </td>
  </tr>
</table>

---

## 2. Documentation Management

<table>
  <tr>
    <td align="center" width="300" valign="top">
      <img src="assets/images/confluence.gif" width="300">
    </td>
    <td align="left" valign="top">
      ▪ Documented the workflow in Confluence across planning, design, research, implementation, and testing.<br>
      ▪ Logged progress at regular intervals.
    </td>
  </tr>
</table>

---

# 10. License

This project is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
Refer to the [`LICENSE`](./LICENSE) file for details.
