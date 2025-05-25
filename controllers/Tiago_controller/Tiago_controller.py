#Standard libraries
from os.path import exists
import numpy as np

#Py_trees core
import py_trees
from py_trees.composites import Sequence, Selector, Parallel

#Custom Modules
from navigation import Navigation
from mapping import Mapping
from planning import Planning
from recognition import Recognition
from manipulation import Manipulation
from object_selector import ObjectSelector
from set_waypoint import SetWaypoint
from straight_line_navigation import StraightLineNavigation

# Webots Robot Interface
import sys
import os

# Try importing Webots controller module; if fails, append likely path
try:
    from controller import Supervisor
except ModuleNotFoundError:
    # Attempt default Webots install location (Linux)
    default_path = "/usr/local/webots/lib/controller/python"
    if os.path.exists(default_path):
        sys.path.append(default_path)
        try:
            from controller import Supervisor
        except ModuleNotFoundError:
            raise ImportError(" Failed to import Webots 'controller' module after setting path.")
    else:
        raise ImportError(" Webots controller module not found. Please ensure Webots is installed and its Python API is accessible.")

from controller import Supervisor

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Waypoints for environment mapping
WP = [(0.614, -0.19), (0.77, -0.94), (0.60, -2.00), (-0.51, -3.39),
      (-1.75, -2.20), (-1.60, -1.25), (-1.60, -0.00), (-0.74, 0.28), (0.40, 0.65)]
WP = np.concatenate((WP, np.flip(WP, 0)), axis = 0)
# checks if precomputed map exists
class DoesMapExist(py_trees.behaviour.Behaviour):
    def update(self):
        file_exists = exists('cspace.npy')
        if(file_exists):
            print("Map already exists")
            return py_trees.common.Status.SUCCESS
        else:
            print("Map does not exist")
            return py_trees.common.Status.FAILURE
            

# Global memory for sharing data among nodes    
class Blackboard:
    def __init__(self):
        self.data = {}
    def write(self, key, value):
        self.data[key] = value
    def read(self, key):
        return self.data.get(key)
        
# Setup blackboard   
blackboard = Blackboard()
blackboard.write('robot', robot)

# creates round-trip waypoints by appending the reverse of WP
blackboard.write('waypoints', WP)

left_motor = robot.getDevice('wheel_left_joint')
right_motor = robot.getDevice('wheel_right_joint')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

def create_pick_place_sequence(blackboard, pick_goals, place_goals, name_suffix=""):
    return Sequence(
        name=f"PickAndPlace{name_suffix}",
        memory = True,
        children=[
            Recognition(name = f"detect object {name_suffix}", blackboard = blackboard),
            ObjectSelector(name = f"select object {name_suffix}", blackboard = blackboard),
            SetWaypoint(name = f"set waypoint to jar {name_suffix}", blackboard = blackboard, src_key = "approach_point", direction = "forward", dst_key = "target_point"),
            Manipulation(name = f"reach", blackboard = blackboard, config_name = "reach"),
            StraightLineNavigation(name = f"approach to jar {name_suffix}", blackboard = blackboard),
            Manipulation(name = f"close gripper", blackboard = blackboard, config_name = "closeGripper"),
            SetWaypoint(name = f"set waypoint to retreat from jar {name_suffix}", blackboard = blackboard, src_key = "retreat_point", direction = "backward", dst_key = "target_point"),
            StraightLineNavigation(name = f"retreat from jar {name_suffix}", blackboard = blackboard),
            Manipulation(name = f"safe after pick {name_suffix}", blackboard = blackboard, config_name = "safe"),

            Planning(name = f"plan to go to table, step 1", blackboard = blackboard, goal = place_goals[0]),
            Navigation(name = f"go to table, step 1", blackboard = blackboard),
            Planning(name = f"plan to go to table, step 2", blackboard = blackboard, goal = place_goals[1]),
            Navigation(name = f"go to table, step 2", blackboard = blackboard),
            
            Manipulation(name = f"prep put 1", blackboard = blackboard, config_name = "prepare_put_1"),
            Manipulation(name = f"prep put 2", blackboard = blackboard, config_name = "prepare_put_2"),
            Manipulation(name = f"put", blackboard = blackboard, config_name = "put"),
            Manipulation(name = f"open gripper", blackboard = blackboard, config_name = "openGripper"),
            Manipulation(name = f"after put", blackboard = blackboard, config_name = "after_put"),
            Manipulation(name = f"safe", blackboard = blackboard, config_name = "safe"),

            # Go back to worktop for next pick
            Planning(name = f"plan to back to worktop, step 1", blackboard = blackboard, goal = pick_goals[0]),
            Navigation(name=f"back to worktop, step 1", blackboard = blackboard),
            Planning(name=f"plan to back to worktop, step 2", blackboard=blackboard, goal=pick_goals[1]),
            Navigation(name=f"back to worktop, step 2", blackboard=blackboard),
            Planning(name=f"plan to back to worktop, step 3", blackboard=blackboard, goal=pick_goals[2]),
            Navigation(name=f"back to worktop, step 3", blackboard=blackboard),
        ]
    )
# Define root node and behavior tree
root = Sequence(
    name = "Main",
    memory = True,
    children = [
        Selector(
            name = "Check Map",
            memory = True,
            children = [
                DoesMapExist(name = "Map Exists?"),
                Parallel(
                    name = "Mapping",
                    policy = py_trees.common.ParallelPolicy.SuccessOnAll(),
                    children = [
                        Navigation(name = "Move Around", blackboard = blackboard),
                        Mapping(name = "Mapping", blackboard = blackboard)
                    ]
                )
            ]
        ),
        # 3 pick-place cycles
        create_pick_place_sequence(
            blackboard,
            pick_goals = [(-1.75, -2.2), (-1.2, 0.1), (0.4, 0.2)],
            place_goals = [(0.65, -0.15), (0.4, -1.75)],
            name_suffix = "1"
        ),
        create_pick_place_sequence(
            blackboard,
            pick_goals = [(-1.54, -2.96), (-1.2, 0.1), (0.4, 0.2)],
            place_goals = [(0.65, 0.0), (0.4, -1.95)],
            name_suffix = "2"
        ),
        create_pick_place_sequence(
            blackboard,
            pick_goals = [(-1.54, -2.96), (-1.2, 0.1), (0.4, 0.2)],
            place_goals = [(0.8, 0.05), (0.4, -2.15)],
            name_suffix = "3"
        )
    ]
)

# Setup all children recursively                
root.setup_with_descendants()

# wrap into Behaviour tree
tree = py_trees.trees.BehaviourTree(root)

# Main Webots control loop
while robot.step(timestep) != -1:
    tree.tick()
    #print(f"[Tick] Root status: {tree.root.status}")
    
    #for node in tree.root.iterate():
    #    print(f"{node.name}: {node.status}")
        
    # exit when root tree completes successfully
    
    # Navigation active
    if blackboard.read("navigation_active"):
        left_velocity = blackboard.read("velocity_left")
        right_velocity = blackboard.read("velocity_right")
    else:
        left_velocity = right_velocity = 0.0
        

    left_motor.setVelocity(left_velocity)
    right_motor.setVelocity(right_velocity)
    
    if tree.root.status == py_trees.common.Status.SUCCESS:
        print("Sequence complete. Robot stopped.")
        break
