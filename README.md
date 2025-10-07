# 🐢 ROS 2 DRL Training System for TurtleBot Navigation

## 📌 Project Overview

This project provides a complete, **containerized foundation** for training a **Deep Reinforcement Learning (DRL)** agent to achieve **autonomous navigation** (collision avoidance and goal seeking) in a simulated environment.

It features a dedicated **Trainer Node** within a ROS 2 Foxy package (`turtlebot_nav`) that acts as the essential bridge — translating raw **LiDAR** and **Odometry** data into a standardized **state space** for an RL agent, and translating the agent's actions back into executable **velocity commands**.

The entire environment is **containerized with Docker** and uses **X11 forwarding** to seamlessly display the Gazebo simulation GUI on the host machine.

---

## 🧠 System Architecture Overview

This project implements the core **Agent–Environment loop** for Reinforcement Learning within the ROS 2 framework:

<p align="center">
  <img src="licensed-image.jpeg" alt="Reinforcement Learning Feedback Loop" width="600"/>
</p>

| Component             | Technology      | Role                                                                                           |
| --------------------- | --------------- | ---------------------------------------------------------------------------------------------- |
| **Gazebo Simulation** | Gazebo, ROS 2   | The virtual "world" (environment). Handles physics, LiDAR sensor, and robot dynamics.          |
| **Robot Model**       | XACRO / URDF    | Defines the differential-drive TurtleBot-like robot and its sensor placement.                  |
| **Trainer Node**      | ROS 2 (`rclpy`) | Core RL interface. Subscribes to sensors, calculates rewards, and publishes velocity commands. |
| **RL Agent**          | Python (stub)   | Placeholder for the deep learning model. Selects optimal action based on current state.        |

---

## ⚙️ Detailed Workflow – Stage by Stage

### 1. 🤖 Robot & World Definition

**Files:** `urdf/robot.urdf.xacro`, `worlds/small.world`

* The XACRO file defines a **TurtleBot-like robot**, including its geometry and inertia.
* It embeds Gazebo plugins for:

  * **Differential Drive:** listens to `/cmd_vel`
  * **360° LiDAR:** publishes to `/gazebo_ros_laser_controller/out`
* The `small.world` file defines a simple arena with boundary walls and box obstacles.

💡 The environment is intentionally simple to help the RL agent quickly learn basic collision avoidance.

---

### 2. 🚀 Launch Orchestration

**File:** `launch/sim.launch.py`

This ROS 2 launch file manages the entire simulation startup sequence using timed actions:

* Starts the Gazebo **physics server (`gzserver`)** and **GUI client (`gzclient`)**
* Starts `robot_state_publisher` to process the XACRO robot description
* **3.0s delay:** Spawns the robot into the Gazebo world
* **5.0s delay:** Starts the main Trainer Node after all topics are active

---

### 3. 📊 Sensor Processing and State Space

**File:** `src/sensor_helpers.py`

This module handles the crucial task of converting raw ROS messages into RL-friendly state vectors:

* `process_scan`: Converts the 360° `LaserScan` message into a normalized (0–1) 1D NumPy array
* `pose_to_state`: Converts the `Odometry` message into a local pose array

🛠️ The agent only sees the **normalized state vector**, not the full ROS message structure.

---

### 4. 🧠 RL Training Logic (The Loop)

**Files:** `src/trainer_node.py`, `src/rl_agent.py`

#### 🌀 Trainer Node (`trainer_node.py`)

Runs at **10 Hz**, continuously performing the DRL cycle:

1. Read the latest processed sensor data (**state**).
2. *(Stub)* Compute the **reward** based on state and previous action.
3. *(Stub)* Send state to `RLAgent` to get the next **action**.
4. Execute the action by publishing to `/cmd_vel`.
5. Handle **episode termination** (collision or goal reached) by resetting the simulation.

#### 🧠 RL Agent (`rl_agent.py`)

A stub class ready for integration with TensorFlow or PyTorch. It defines essential methods:

* `select_action(state)` – using an ε-greedy policy
* `train(...)` – training logic placeholder

---

## 📂 Project Structure

```
ros2_ws/
└── src/
    └── turtlebot_nav/
        ├── CMakeLists.txt              # Build instructions (ament_cmake)
        ├── package.xml                 # Package dependencies and metadata
        ├── requirements.txt            # Python dependencies (e.g., numpy)
        │
        ├── launch/
        │   └── sim.launch.py           # ROS 2 launch file for starting Gazebo and nodes
        │
        ├── src/
        │   ├── trainer_node.py         # Main RL training loop (ROS 2 Node)
        │   ├── rl_agent.py             # DRL Agent class stub (needs ML framework)
        │   └── sensor_helpers.py       # Utilities for processing LiDAR and Odometry data
        │
        ├── urdf/
        │   └── robot.urdf.xacro        # TurtleBot-like robot description with Gazebo plugins
        │
        └── worlds/
            └── small.world             # Gazebo simulation environment (6x6m arena)
```

---

## ▶️ How to Run the Training System

These steps assume you are running on a **Linux host (e.g., Ubuntu)** with **Docker installed**.

---

### 🖥️ Pre-Requisite: X11 Forwarding Setup

Grant the Docker container permission to display the Gazebo GUI on your desktop:

```bash
xhost +local:docker
```

✅ Output:

```
non-network local connections being added to access control list
```

---

### 🐳 Execution

#### 1. Build the Docker Image

From your project root (containing `ros2_ws` and `Dockerfile`):

```bash
docker build -t turtlebot_nav_foxy .
```

#### 2. Launch the System

Run the container with X11 forwarding and launch the ROS 2 simulation:

```bash
docker run -it --rm \
    --name rl_trainer \
    --net=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    turtlebot_nav_foxy \
    bash -c 'source /opt/ros/foxy/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch turtlebot_nav sim.launch.py'
```

---

### ✅ Expected Output

* The **Gazebo GUI** opens on your host, showing the `small.world` arena and TurtleBot model.
* Terminal logs from the Trainer Node confirm the **anti-collision routine** is active and running.

---

## 📊 What This Demonstrates

This setup shows the **fundamental loop** behind any simulation-to-real-world robotics deployment:

| Stage          | What Happens                                  | Real-World Equivalent                        |
| -------------- | --------------------------------------------- | -------------------------------------------- |
| 🌎 Environment | Gazebo physics + LiDAR data generation        | Physical world interaction & sensor hardware |
| ⚙️ Interface   | ROS 2 node handles subscriptions/publications | Robot’s onboard controller/microcontroller   |
| 🧠 Agent       | DRL model selects actions                     | Embedded AI policy in autonomous robot       |
| 🔁 Feedback    | Collisions trigger episode reset              | Sensor feedback loop for safe operation      |

---

## 🚀 Future Extensions

This system is a **runnable foundation**. Recommended next steps for full DRL functionality:

* 🧠 **Deep Learning Integration:** Implement a DRL network (e.g., DQN, PPO) in `src/rl_agent.py` using TensorFlow or PyTorch.
* 🏆 **Reward Function & Goal Tracking:** Add reward logic (positive for progress, negative for collisions) and include goal coordinates in the state.
* 📚 **Experience Replay:** Add a replay buffer to stabilize off-policy learning algorithms like DQN.

---
