# PincherX 100 Teach Pendant GUI

A beginner-friendly graphical teach pendant for the **Interbotix PincherX 100**, built with Python, PyQt and ROS 2.

The goal of this project is simple: **make the PincherX 100 immediately approachable for students and first-time robotics users without requiring them to start by writing ROS 2 or Python code.**

![](https://github.com/jansistonen/pincher-gui/blob/main/pincherV4_1.gif)

---

## Why this project?

The PincherX 100 is a compact and useful educational robot arm. Interbotix provides a ROS 2 interface and a Python API, which make the robot highly programmable.

However, for a complete beginner, getting from:

**"Here is the robot"**

to:

**"I can move it, inspect its joints, teach positions and run a simple robot program"**

can require a surprisingly large amount of preparation.

Before the robot can be used programmatically, the user needs to become familiar with topics such as:

- Ubuntu and the Linux terminal
- ROS 2 workspaces
- sourcing ROS environments
- ROS 2 nodes, topics and services
- Interbotix packages
- Python APIs
- joint names and joint limits
- radians versus degrees
- forward and inverse kinematics

All of these are useful concepts to learn, but requiring them **before the student can meaningfully interact with the robot** can create an unnecessary barrier.

This project tries to reverse that learning order.

Instead of starting from the terminal, the student can first start from the robot.

---

## The idea

The interface is inspired by industrial robot **teach pendants**, such as the interface used with Universal Robots UR-series robots.

A typical industrial teach pendant allows the operator to:

1. Move the robot.
2. Observe its current position.
3. Save a position as a waypoint.
4. Move to another position.
5. Save additional waypoints.
6. Arrange the waypoints into a program.
7. Run the program.

This GUI applies the same basic idea to the PincherX 100.

It is **not intended to replace ROS 2 or the Interbotix Python API**. Instead, it acts as an accessible first layer on top of them.

The intended learning path is:

```text
Teach Pendant GUI
        ↓
Understand joints, poses and robot motion
        ↓
Explore ROS 2 topics and services
        ↓
Use the Interbotix Python API
        ↓
Write custom ROS 2 applications
```

This allows the robot to become a learning tool immediately, while the lower-level software can be introduced gradually.

---

## What can the GUI do?

The current version provides both **joint-space** and **Cartesian-style** robot control.

### Joint control

The user can view the actual joint angles of the PincherX 100 and command individual joints.

The interface provides controls for:

- Waist
- Shoulder
- Elbow
- Wrist angle
- Gripper

The displayed values are read from the actual ROS 2 joint state feedback.

This makes the interface useful not only for operating the robot, but also for learning how a multi-joint manipulator behaves.

> **Screenshot placeholder – Joint control**  
> Add a screenshot showing the joint sliders, actual angles and jog controls.

For example, a student can move only the shoulder joint and immediately observe how changing one joint affects the position and orientation of the end effector.

---

## Teaching positions

The current physical position of the robot can be stored as a **Pose**.

A basic workflow can therefore be:

```text
Move robot
    ↓
SAVE POSITION
    ↓
Move robot
    ↓
SAVE POSITION
    ↓
Move robot
    ↓
SAVE POSITION
```

The saved poses are displayed as a program sequence.

Each pose contains the joint positions and can also include parameters such as:

- movement time
- wait time
- gripper command

The order of the poses can be changed before running the program.

> **Screenshot placeholder – Program / saved poses**  
> Add a screenshot showing several stored poses in the program table.

---

## Running a robot program

After several positions have been taught, they can be executed sequentially using **RUN PROGRAM**.

For example:

```text
1. Home              Gripper: OPEN
2. Above object      Gripper: KEEP
3. Pick position     Gripper: CLOSE
4. Lift object       Gripper: KEEP
5. Place position    Gripper: KEEP
6. Release object    Gripper: OPEN
7. Home              Gripper: KEEP
```

During execution, the currently active command is visually highlighted.

The interface also shows the current execution phase, for example:

```text
▶ Step 3/7 — Pick position — MOVING ARM
▶ Step 3/7 — Pick position — GRIPPER CLOSE
▶ Step 3/7 — Pick position — WAIT 0.50 s
```

This makes program execution easier to follow and provides behavior similar to the visual program-step indication found in industrial robot interfaces.

> **Screenshot placeholder – Program running**  
> Add a screenshot where the currently executing row is highlighted.

---

## Gripper control

The gripper can be controlled directly from the interface.

The GUI currently supports commands such as:

- Open
- Close
- Stop gripper
- Adjustable gripping pressure

Gripper actions can also be included in stored program poses.

This allows simple pick-and-place programs to be created without writing code.

> **Screenshot placeholder – Gripper controls**

Future versions can also use the Interbotix linear-position mode to command a specific finger opening instead of only OPEN/CLOSE commands.

---

## Cartesian control and kinematics

The GUI also provides experimental Cartesian jogging for:

- X
- Y
- Z
- End-effector pitch

The actual Cartesian position is calculated from the measured joint states using **Forward Kinematics (FK)**.

A desired Cartesian movement is converted back into joint angles using **Inverse Kinematics (IK)**.

This provides an intuitive way to demonstrate the relationship:

```text
Joint angles
     ↓
Forward Kinematics
     ↓
X, Y, Z and orientation
```

and in the opposite direction:

```text
Desired X, Y, Z
     ↓
Inverse Kinematics
     ↓
Required joint angles
```

Because the PincherX 100 is a **4-DOF robot**, its Cartesian motion is more constrained than that of a typical 6-axis industrial manipulator. This also makes it useful educationally: the limitations of inverse kinematics and robot degrees of freedom become visible in practice.

> **Screenshot placeholder – Cartesian control**  
> Add a screenshot showing Actual X/Y/Z, Cartesian Target and Cartesian jog buttons.

---

# Installation and requirements

The current setup is developed for:

```text
Ubuntu 22.04
ROS 2 Humble
Interbotix X-Series ROS 2 packages
PincherX 100
Python 3
PyQt5
```

Before running the GUI, make sure the PincherX 100 works normally through the Interbotix ROS 2 driver.

---

## 1. Check ROS 2

Open a terminal:

```bash
source /opt/ros/humble/setup.bash

echo $ROS_DISTRO
```

Expected result:

```text
humble
```

---

## 2. Check the Interbotix workspace

The Interbotix packages should be installed in a ROS 2 workspace, for example:

```text
~/interbotix_ws
```

Source it:

```bash
source ~/interbotix_ws/install/setup.bash
```

Check that the required packages are available:

```bash
ros2 pkg list | grep interbotix_xsarm_control
```

You should see:

```text
interbotix_xsarm_control
```

You can also check the main packages:

```bash
ros2 pkg list | grep -E \
"interbotix_xs_sdk|interbotix_xs_modules|interbotix_xsarm_control"
```

---

## 3. Check the USB connection

Connect the PincherX 100 through the Interbotix U2D2 interface and turn on the robot power.

Check the device:

```bash
ls /dev | grep ttyDXL
```

Expected result:

```text
ttyDXL
```

---

# Starting the robot

Two terminals are currently used.

## Terminal 1 – Start the Interbotix driver

```bash
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

ros2 launch interbotix_xsarm_control xsarm_control.launch.py \
  robot_model:=px100 \
  use_rviz:=false \
  load_configs:=false
```

Keep this terminal running.

A successful startup should include messages similar to:

```text
Found DYNAMIXEL ID: 1 ... waist
Found DYNAMIXEL ID: 2 ... shoulder
Found DYNAMIXEL ID: 3 ... elbow
Found DYNAMIXEL ID: 4 ... wrist_angle
Found DYNAMIXEL ID: 5 ... gripper

Interbotix X-Series Driver is up!
InterbotixRobotXS is up!
```

---

## Terminal 2 – Start the Teach Pendant

Source the same environments:

```bash
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

Navigate to the Teach Pendant directory:

```bash
cd ~/pincher_teach_pendant
```

Start the GUI:

```bash
python3 pincher_teach_pendant.py --real
```

The application should now connect to the running ROS 2 driver and begin displaying the actual PincherX 100 joint states.

---

# Basic usage

### Moving the robot

Use the joint controls to make small movements.

For the first tests, use small jog commands such as:

```text
+1°
-1°
```

The **Actual** value shows the measured joint position.

The **Target** value shows the requested position.

### HOME

Press:

```text
HOME
```

to command the arm to its standard home position.

### Saving a position

Move the robot to the desired position and press:

```text
SAVE ACTUAL POSITION
```

The position appears in the program table.

Repeat this process to create additional poses.

### Running the program

After creating multiple poses, press:

```text
RUN PROGRAM
```

The robot executes the poses sequentially.

The active program row is highlighted so that the user can see exactly which instruction is currently being executed.

### Saving the program

Programs can be saved as JSON files and loaded again later.

This allows students to create, modify and exchange simple robot programs without editing Python or ROS 2 code.

---

# Software architecture

The application is deliberately separated into a graphical layer and a robot backend.

```text
┌───────────────────────────────┐
│      Teach Pendant GUI        │
│       Python + PyQt           │
└───────────────┬───────────────┘
                │
         Robot Backend
                │
      ┌─────────┴─────────┐
      │                   │
 Demo Backend       ROS 2 Backend
                          │
                    Interbotix xs_sdk
                          │
                         U2D2
                          │
                    PincherX 100
```

This separation makes it possible to run the interface without a physical robot using **Demo Mode**, while the same GUI can communicate with the real robot using the ROS 2 backend.

Demo Mode:

```bash
python3 pincher_teach_pendant.py --demo
```

---

# Educational use

The Teach Pendant is intended especially for robotics teaching.

A student can use the GUI to investigate:

- robot joints
- joint states
- joint angles
- joint limits
- degrees of freedom
- robot poses
- waypoint programming
- gripper operation
- forward kinematics
- inverse kinematics
- Cartesian versus joint-space motion
- basic pick-and-place programming

After understanding these concepts visually, the same operations can be reproduced using the Interbotix Python API or directly through ROS 2.

The GUI therefore acts as a bridge between **physical experimentation** and **robot programming**.

---

# Safety

This software is an educational interface and **does not replace appropriate robot safety procedures**.

In particular:

- keep the workspace clear before commanding movement
- start with small joint movements
- do not exceed the mechanical payload of the robot
- do not manually force joints while motor torque is enabled
- GUI `STOP` functions should not be considered safety-rated emergency stops
- disconnect or disable robot power according to the appropriate procedure if immediate physical stopping is required

---

# Project status

The project is currently under active development.

Current functionality:

- [x] ROS 2 Humble connection
- [x] PincherX 100 joint-state feedback
- [x] Joint jogging
- [x] HOME / SLEEP
- [x] Pose teaching
- [x] Program sequencing
- [x] Active-program-step highlighting
- [x] Program Save / Load
- [x] Gripper control
- [x] Gripper commands inside programs
- [x] Cartesian position display
- [x] Experimental Cartesian jogging
- [x] Demo Mode

Possible future improvements:

- [ ] Gripper opening commanded directly in millimetres
- [ ] Improved Cartesian trajectory control
- [ ] Automatic ROS 2 / Interbotix driver startup from the GUI
- [ ] Integrated robot 3D visualization
- [ ] Velocity and acceleration controls
- [ ] Teach-by-hand functionality
- [ ] Better program editing tools
- [ ] Loop, condition and delay commands
- [ ] Packaged application / desktop launcher
- [ ] Support for additional Interbotix X-Series arms

---

## Goal

The long-term goal is not to hide ROS 2.

The goal is to make the first contact with ROS-based robotics easier.

A new user should be able to start by **moving the robot and understanding what it does**, and only then move deeper into the ROS 2 architecture behind it.

The Teach Pendant provides that first layer.
