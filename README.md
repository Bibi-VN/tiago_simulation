# 🤖 TIAGo Robot Simulation

This project simulates the TIAGo mobile robot in a kitchen environment using Webots and Python.

## 📽️ Demonstration
Below are two videos demonstrating the robot's capabilities:

### Video 1: Mapping, Navigation, and Path Planning

The robot follows a predefined trajectory to explore and build a map of the environment. Then, it performs path planing and navigation based on the constructed map.

[▶️ Watch Video 1](https://youtu.be/cBmVXozQd8M)

### Video 2: Pick-and-Place Task Execution

The goal is that the robot can autonomously detect, grasp, and transfer three jars from a worktop surface to a designated position on a nearby table.

The solution is designed such that the robot mimics how a human would perform the task. The behavior is structured using a behavior tree and involves the following steps:
1. Move to the worktop
2. Detect the presence of target jars
3. Select and pick one of the detected jars
4. Transfer it to the designated position on the table
5. Repeat until the task is complete

[▶️ Watch Video 2](https://youtu.be/2DOx8iX4RRk)

## 📁 Project Structure
``` 
root/
├── controllers/                    # Robot control scripts
│   ├── Tiago_controller.py         # Main robot control logic
│   ├── navigation.py               # Autonomous navigation based on provided waypoints
│   ├── mapping.py                  # Environment scanning and map building
│   ├── planning.py                 # Path planning for navigation
│   ├── set_waypoint.py             # Waypoint setting utility
│   ├── manipulation.py             # Pick-and-place logic
│   ├── recognition.py              # Object recognition for picking
│   ├── object_selector.py          # Logic for selecting objects to manipulate
│   └── straight_line_navigation.py # Straight-line navigation supporting the picking process
├── worlds/                         # Webots simulation world
│   └── kitchen.wbt
├── environment.yml                 # Conda environment configuration
└── README.md
```
---

## ⚙️ Installation

1. Clone the repository (if applicable):
   ```bash
   git clone https://github.com/Bibi-VN/tiago_simulation.git
   ```
2. Navigate to the project directory 
   ```bash
   cd ~/tiago_simulation
   ```
3. Create and activate the Conda environment:  
   This project uses [Miniconda](https://docs.conda.io/en/latest/miniconda.html).

   ```bash
   conda env create -f environment.yml
   conda activate tiago_simulation
   ```
## 🚀 Usage
1. Launch the Webots simulation
   ```bash
   webots worlds/kitchen.wbt
   ```
2. Webots Integration to use your Conda environment in Webots:
   1. Open Webots
   2. Go to **Tools → Preferences**
   3. Set **Python command** to the full path of the Conda Python:

   ```bash
   /home/your_username/miniconda3/envs/mobile_robots/bin/python
   ```
   
3. Click the "Play" button in Webots after launching

## 🧠 Behavior Architecture

The robot's behavior is structured as a hierarchical **Behavior Tree (BT)**, consisting of modules for mapping, perception, planning, navigation, and manipulation.

### 🔹 Root Node: `Main` (Sequence)

- **Check Map** (Selector):
  - `DoesMapExist`: Checks if a prior map is available
  - `Mapping`: The robot will follow the predefined trajectory to build the map of the environment
- **Pick-and-Place Cycles**:
  - Three repeated `PickAndPlace` sequences handle autonomous object manipulation

---

### 🔹 PickAndPlace (Sequence)

This subtree includes:

1. **Perception**
   - `Recognition`: detects jars
   - `ObjectSelector`: chooses one of detected jars

2. **Approach and Pick**
   - `SetWaypoint`: sets goal to approach the detected jar
   - `StraightLineNavigation`: executes safe approach to jar
   - `Manipulation (reach, closeGripper)`

3. **Placement**
   - `Planning + Navigation`: multi-step path to table
   - `Manipulation (put, openGripper, safe)`

4. **Return**
   - `Planning + Navigation`: multi-step path back to worktop
   
> **Note:** The multi-step navigation to and from the table is necessary due to the robot's kinematic constraints. The TIAGo base cannot execute tight turning angles directly, so the path is divided into smaller segments to ensure feasible execution.
---

### 🧩 Behavior Tree Modules by Function

| Functionality      | Nodes Used                                              |
|--------------------|---------------------------------------------------------|
| **Mapping**        | `DoesMapExist`, `Mapping`, `Navigation`                 |
| **Perception**     | `Recognition`, `ObjectSelector`                         |
| **Waypoint Planning** | `SetWaypoint`                                        |
| **Navigation**     | `Planning`, `Navigation`, `StraightLineNavigation`      |
| **Manipulation**   | `Manipulation (reach, grip, put, safe,...)`    |

> Behavior logic is fully implemented using `py_trees`, and each Pick-and-Place cycle is built using `create_pick_place_sequence(...)`.

## 📄 License

## 👤 Author

**Phu Vo**  
Email: [ngocphuvo.work@gmail.com]

## 🙏 Acknowledgements

This project is inspired by and references the **_Introduction to Robotics with Webots Specialization_** taught by [Prof. Nikolaus Correll](https://www.coursera.org/instructor/correll) on Coursera.  
The specialization is part of the Master's program in Computer Science at the **University of Colorado Boulder**.

