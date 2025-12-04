# Path Smoothing and Trajectory Control (2D)

This repository contains a complete implementation for the assignment "Path Smoothing and Trajectory Control in 2D Space". It includes path smoothing, trajectory generation, trajectory tracking control, and visualization - both as a standalone Python demo and as ROS2 nodes.

---

## Quick Start

### Windows (Standalone - No ROS2 Required)

```powershell
cd "C:\Users\alok kumar\OneDrive\Desktop\Robotics_Ass"
python -m pip install numpy matplotlib
python run_simulation.py
```

This runs a complete animated simulation showing the robot following a smooth trajectory.

### Ubuntu/WSL with ROS2

Run these commands in **3 separate terminals**:

**Terminal 1 - Trajectory Publisher:**
```bash
cd /mnt/c/Users/alok\ kumar/OneDrive/Desktop/Robotics_Ass
source /opt/ros/jazzy/setup.bash
python3 ros2_pkg/ros2_pkg/trajectory_publisher.py
```

**Terminal 2 - Controller Node:**
```bash
cd /mnt/c/Users/alok\ kumar/OneDrive/Desktop/Robotics_Ass
source /opt/ros/jazzy/setup.bash
python3 ros2_pkg/ros2_pkg/controller_node.py
```

**Terminal 3 - Robot Simulator:**
```bash
cd /mnt/c/Users/alok\ kumar/OneDrive/Desktop/Robotics_Ass
source /opt/ros/jazzy/setup.bash
python3 ros2_pkg/ros2_pkg/robot_simulator_node.py
```

**Terminal 4 (Optional) - Visualizer:**
```bash
cd /mnt/c/Users/alok\ kumar/OneDrive/Desktop/Robotics_Ass
source /opt/ros/jazzy/setup.bash
python3 ros2_pkg/ros2_pkg/visualizer_node.py
```

---

## Summary / Goal

- **Input**: A coarse sequence of 2D waypoints (e.g., from a global planner or hand-specified path).
- **Output**: A smooth, time-parameterized trajectory and a controller that drives a simulated differential-drive robot to follow that trajectory.

### Deliverables:
- ✅ **Path smoothing**: Catmull-Rom spline converts discrete waypoints into a smooth path
- ✅ **Trajectory generation**: Time-stamped samples with trapezoidal velocity profiling
- ✅ **Trajectory tracking controller**: Pure-pursuit style controller producing (v, ω) commands
- ✅ **Visualization**: Matplotlib simulation showing path, trajectory, and robot following it
- ✅ **ROS2 Integration**: Full ROS2 nodes for real robot deployment

---

## Project Structure

```
Robotics_Ass/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── run_simulation.py         # Main standalone simulation (recommended)
│
├── src/                      # Core algorithms
│   ├── smoothing.py          # Catmull-Rom spline path smoothing
│   ├── trajectory.py         # Arc-length sampling + trapezoidal velocity profile
│   ├── controller.py         # Proportional tracking controller
│   ├── simulator.py          # Matplotlib closed-loop simulation
│   ├── demo.py               # Basic demo script
│   ├── debug_sim.py          # Debug simulation with console output
│   └── ros2_simulation.py    # Standalone ROS2-like simulation (no ROS2 needed)
│
├── ros2_pkg/                 # ROS2 package
│   ├── ros2_pkg/
│   │   ├── trajectory_publisher.py   # Publishes nav_msgs/Path
│   │   ├── controller_node.py        # Subscribes to trajectory, publishes cmd_vel
│   │   ├── robot_simulator_node.py   # Simulates robot, publishes odometry
│   │   └── visualizer_node.py        # Matplotlib visualization for ROS2
│   ├── launch/
│   │   └── bringup_launch.py         # ROS2 launch file
│   ├── setup.py
│   └── package.xml
│
├── tests/                    # Unit tests
│   └── test_smoothing.py
│
└── out/                      # Output files (generated)
    └── history.csv
```

---

## Available Demos

| Demo | Command | Description |
|------|---------|-------------|
| **Main Simulation** | `python run_simulation.py` | Full animated simulation with velocity plots |
| **Basic Demo** | `python src/demo.py` | Simple trajectory following demo |
| **Debug Demo** | `python src/debug_sim.py` | Console output showing robot position |
| **ROS2-like Demo** | `python src/ros2_simulation.py` | Multi-threaded simulation mimicking ROS2 |

---

## How to Run

### Option 1: Windows Standalone (Recommended for Demo)

**Install dependencies:**
```powershell
cd "C:\Users\alok kumar\OneDrive\Desktop\Robotics_Ass"
python -m pip install numpy matplotlib
```

**Run the main simulation:**
```powershell
python run_simulation.py
```

**Expected output:**
- A matplotlib window opens showing:
  - Left panel: Robot (black dot) following the blue trajectory, leaving a green trail
  - Right panel: Velocity commands (linear and angular) over time
- Console shows progress: `Step: 100 | Pos: (3.50, 0.45) | v=0.48 | dist_to_goal=2.50`
- Simulation ends when robot reaches goal (~15 seconds)

### Option 2: Ubuntu/ROS2 (Full ROS2 Integration)

**Prerequisites:**
- Ubuntu 24.04 (Noble) with ROS2 Jazzy installed
- Or Ubuntu 22.04 with ROS2 Humble

**Install ROS2 Jazzy (if not installed):**
```bash
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-ros-base python3-pip
pip3 install numpy matplotlib
```

**Run the ROS2 nodes (3 terminals required):**

| Terminal | Command |
|----------|---------|
| 1 | `cd /mnt/c/Users/alok\ kumar/OneDrive/Desktop/Robotics_Ass && source /opt/ros/jazzy/setup.bash && python3 ros2_pkg/ros2_pkg/trajectory_publisher.py` |
| 2 | `cd /mnt/c/Users/alok\ kumar/OneDrive/Desktop/Robotics_Ass && source /opt/ros/jazzy/setup.bash && python3 ros2_pkg/ros2_pkg/controller_node.py` |
| 3 | `cd /mnt/c/Users/alok\ kumar/OneDrive/Desktop/Robotics_Ass && source /opt/ros/jazzy/setup.bash && python3 ros2_pkg/ros2_pkg/robot_simulator_node.py` |

**Expected output in Terminal 3 (Robot Simulator):**
```
[RobotSimulatorNode] started - robot at (0, 0, 0)
Robot at (0.50, 0.12), theta=8.5°, v=0.45, omega=0.12
Robot at (1.20, 0.05), theta=3.2°, v=0.48, omega=-0.05
Robot at (2.50, 0.30), theta=5.1°, v=0.50, omega=0.08
...
Robot at (5.95, 0.02), theta=0.5°, v=0.10, omega=-0.02
Goal reached! Final position: (6.00, 0.01)
```

**Monitor ROS2 topics (optional Terminal 4):**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list                    # See available topics
ros2 topic echo /odom              # Robot position
ros2 topic echo /cmd_vel           # Velocity commands
ros2 topic echo /trajectory        # Planned path
```

---

## Algorithms Implemented

### 1. Path Smoothing (Catmull-Rom Spline)
- **File**: `src/smoothing.py`
- **Function**: `smooth_path(waypoints, samples_per_segment=20)`
- **Description**: Converts discrete waypoints into a smooth, continuous curve using Catmull-Rom spline interpolation
- **Properties**: C¹ continuous (smooth first derivative), passes through all waypoints

```python
# Example usage
from smoothing import smooth_path
waypoints = np.array([[0,0], [2,1], [4,0], [6,1]])
smooth = smooth_path(waypoints, samples_per_segment=30)  # Returns Nx2 array
```

### 2. Trajectory Generation (Trapezoidal Velocity Profile)
- **File**: `src/trajectory.py`
- **Function**: `generate_trajectory(polyline, n_samples=200, v_max=1.0, a_max=0.5)`
- **Description**: Creates time-parameterized trajectory with:
  - Arc-length sampling for uniform spatial distribution
  - Trapezoidal velocity profile (accelerate → cruise → decelerate)
- **Output**: Dictionary with `points`, `times`, `distances`, `velocities`

```python
# Example usage
from trajectory import generate_trajectory
traj = generate_trajectory(smooth, n_samples=100, v_max=0.5, a_max=0.3)
# traj['points'] - Nx2 array of (x,y) positions
# traj['times']  - N array of timestamps
# traj['v']      - N array of velocities
```

### 3. Trajectory Tracking Controller (Pure Pursuit)
- **File**: `src/controller.py`
- **Function**: `simple_tracking_control(state, target_point, v_des, k_rho, k_alpha)`
- **Description**: Proportional controller for differential-drive robots
  - Computes distance (ρ) and heading error (α) to target
  - Returns linear velocity (v) and angular velocity (ω)

```python
# Example usage
from controller import simple_tracking_control
state = (x, y, theta)  # Robot pose
target = (tx, ty)      # Target point
v, omega = simple_tracking_control(state, target, v_des=0.5)
```

---

## ROS2 Architecture

The ROS2 implementation uses a publish-subscribe architecture:

```
┌─────────────────────┐     /trajectory      ┌─────────────────────┐
│ trajectory_publisher│ ──────────────────▶  │   controller_node   │
│   (nav_msgs/Path)   │                      │                     │
└─────────────────────┘                      └──────────┬──────────┘
                                                        │
                                                        │ /cmd_vel
                                                        │ (geometry_msgs/Twist)
                                                        ▼
┌─────────────────────┐     /odom            ┌─────────────────────┐
│   controller_node   │ ◀────────────────────│ robot_simulator_node│
│                     │                      │ (or real robot)     │
└─────────────────────┘                      └─────────────────────┘
```

### ROS2 Topics

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|-----------|------------|-------------|
| `/trajectory` | `nav_msgs/Path` | trajectory_publisher | controller_node | Smooth trajectory with 100 poses |
| `/cmd_vel` | `geometry_msgs/Twist` | controller_node | robot_simulator | Velocity commands (v, ω) |
| `/odom` | `nav_msgs/Odometry` | robot_simulator | controller_node | Robot position feedback |

### ROS2 Nodes

| Node | File | Description |
|------|------|-------------|
| `trajectory_publisher` | `trajectory_publisher.py` | Generates smooth trajectory using Catmull-Rom spline and trapezoidal velocity profile |
| `controller_node` | `controller_node.py` | Pure-pursuit controller that tracks the trajectory |
| `robot_simulator` | `robot_simulator_node.py` | Simulates differential-drive robot kinematics |
| `visualizer_node` | `visualizer_node.py` | Matplotlib visualization (optional) |

---

## How to Extend to a Real Robot

To use on a real robot (e.g., TurtleBot3):

1. **Replace robot_simulator_node** with your real robot's odometry source
2. **Ensure /odom topic** is published by your robot's localization system
3. **Connect /cmd_vel** to your robot's motor controller
4. **Tune controller parameters** (lookahead, v_max, k_rot) for your robot

```python
# In controller_node.py, adjust these parameters:
self.declare_parameter('lookahead', 0.3)       # Lookahead distance (m)
self.declare_parameter('v_max', 0.5)           # Max linear velocity (m/s)
self.declare_parameter('k_rot', 1.5)           # Rotation gain
self.declare_parameter('goal_tolerance', 0.15) # Goal threshold (m)
```

---

## Tests

Run unit tests:
```powershell
python -m pytest tests/ -v
```

**Test coverage:**
- `test_smoothing.py`: Verifies smooth path preserves endpoints, has correct length, is continuous

---

## Assignment Rubric Mapping

| Criteria | Points | Implementation |
|----------|--------|----------------|
| Code Quality | 35 | Modular design: `smoothing.py`, `trajectory.py`, `controller.py`, `simulator.py` |
| Trajectory Generation & Smoothing | 15 | Catmull-Rom spline + arc-length sampling + trapezoidal velocity profile |
| Controller | 20 | Pure-pursuit style with tunable gains (lookahead, v_max, k_rot) |
| Simulation | 20 | Matplotlib visualization with real-time animation |
| Testability & QA | 20 | Unit tests in `tests/` folder |
| Documentation & Demo | 15 | This README + demo scripts + ROS2 integration |

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Matplotlib window doesn't show | Try `plt.show(block=True)` or check your Python GUI backend |
| Robot oscillates around path | Reduce `k_rot` gain or increase `lookahead` distance |
| Robot moves too slowly | Increase `v_max` parameter |
| ROS2 not found | Ensure you've sourced: `source /opt/ros/jazzy/setup.bash` |
| "No module named rclpy" | Install ROS2 or run the standalone Windows demo instead |
| Simulation appears stuck | The robot starts slow (v=0 at t=0), wait a few seconds |

---

## Demo Video Script (3-5 minutes)

1. **Introduction** (30s): Show the waypoints and explain the goal
2. **Path Smoothing** (45s): Show before/after - discrete waypoints → smooth Catmull-Rom spline
3. **Trajectory Generation** (45s): Explain trapezoidal velocity profile (accel → cruise → decel)
4. **Controller Demo** (90s): Run `python run_simulation.py` and explain:
   - Blue line = planned trajectory
   - Green line = robot's actual path
   - Right panel = velocity commands over time
5. **ROS2 Integration** (30s): Show the 3-terminal ROS2 setup
6. **Conclusion** (30s): Summarize and mention possible extensions

---

## References

- Catmull-Rom Spline: E. Catmull & R. Rom, "A class of local interpolating splines" (1974)
- Pure Pursuit: R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm" (1992)
- Trapezoidal Velocity Profile: S. M. LaValle, "Planning Algorithms" Chapter 8
- ROS2 Documentation: https://docs.ros.org/en/jazzy/

---

*Last updated: November 2025*
*Tested on: Windows 11 + Python 3.10, Ubuntu 24.04 + ROS2 Jazzy*
