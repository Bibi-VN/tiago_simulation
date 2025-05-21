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

# Define root node and behavior tree
root = Sequence(
    name = "Main",
    memory = True,
    children = [
        Selector(
            name = "Does map exists?",
            memory = True,
            children = [
                DoesMapExist(name = "Test for map"), 
                Parallel(
                    name = "Mapping",
                    policy = py_trees.common.ParallelPolicy.SuccessOnAll(),
                    children = [
                        Navigation(name = "move around the table", blackboard = blackboard),
                        Mapping(name = "map the environment", blackboard = blackboard),
                    ]
                )
            ]
        ),
        #Manipulation(name = "put", blackboard = blackboard, config_name = 'put'),
        Recognition(name = "detect object", blackboard = blackboard),
        ObjectSelector(name = "select nearest object", blackboard = blackboard),
        SetWaypoint(name="set approach point", blackboard = blackboard, src_key = "approach_point", direction = 'forward', dst_key = "target_point"),
        Manipulation(name = "reach", blackboard = blackboard, config_name = 'reach'),
        StraightLineNavigation(name = "approach jar", blackboard = blackboard),
        Manipulation(name = "close Gripper", blackboard = blackboard, config_name = 'closeGripper'),
        SetWaypoint(name="set retreat point", blackboard = blackboard, src_key = "retreat_point", direction = 'backward', dst_key = "target_point"),
        StraightLineNavigation(name = "retreat from jar", blackboard = blackboard),
        Manipulation(name = "safe", blackboard = blackboard, config_name = 'safe'),
        Planning(name = "planning to table", blackboard = blackboard, goal = (0.65, -0.15)),
        Navigation(name = "move to table", blackboard = blackboard),
        Planning(name = "planning to table", blackboard = blackboard, goal = (0.4, -1.75)),
        Navigation(name = "move to table", blackboard = blackboard),
        Manipulation(name = "prepare to put 1", blackboard = blackboard, config_name = 'prepare_put_1'),
        Manipulation(name = "prepare to put 2", blackboard = blackboard, config_name = 'prepare_put_2'),
        Manipulation(name = "put", blackboard = blackboard, config_name = 'put'),
        Manipulation(name = "put", blackboard = blackboard, config_name = 'openGripper'),
        Manipulation(name = "after put", blackboard = blackboard, config_name = 'after_put'),
        Manipulation(name = "safe", blackboard = blackboard, config_name = 'safe'),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (-1.75, -2.2)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (-1.2, 0.1)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (0.4, 0.2)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        
        Recognition(name = "detect object", blackboard = blackboard),
        ObjectSelector(name = "select nearest object", blackboard = blackboard),
        SetWaypoint(name="set approach point", blackboard = blackboard, src_key = "approach_point", direction = 'forward', dst_key = "target_point"),
        Manipulation(name = "reach", blackboard = blackboard, config_name = 'reach'),
        StraightLineNavigation(name = "approach jar", blackboard = blackboard),
        Manipulation(name = "close Gripper", blackboard = blackboard, config_name = 'closeGripper'),
        SetWaypoint(name="set retreat point", blackboard = blackboard, src_key = "retreat_point", direction = 'backward', dst_key = "target_point"),
        StraightLineNavigation(name = "retreat from jar", blackboard = blackboard),
        Manipulation(name = "safe", blackboard = blackboard, config_name = 'safe'),
        Planning(name = "planning to table", blackboard = blackboard, goal = (0.65, 0.0)),
        Navigation(name = "move to table", blackboard = blackboard),
        Planning(name = "planning to place", blackboard = blackboard, goal = (0.4, -1.95)),
        Navigation(name = "move to place position", blackboard = blackboard),
        Manipulation(name = "prepare to put 1", blackboard = blackboard, config_name = 'prepare_put_1'),
        Manipulation(name = "prepare to put 2", blackboard = blackboard, config_name = 'prepare_put_2'),
        Manipulation(name = "put", blackboard = blackboard, config_name = 'put'),
        Manipulation(name = "put", blackboard = blackboard, config_name = 'openGripper'),
        Manipulation(name = "after put", blackboard = blackboard, config_name = 'after_put'),
        Manipulation(name = "safe", blackboard = blackboard, config_name = 'safe'),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (-1.54, -2.96)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (-1.2, 0.1)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (0.4, 0.2)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        
        Recognition(name = "detect object", blackboard = blackboard),
        ObjectSelector(name = "select nearest object", blackboard = blackboard),
        SetWaypoint(name="set approach point", blackboard = blackboard, src_key = "approach_point", direction = 'forward', dst_key = "target_point"),
        Manipulation(name = "reach", blackboard = blackboard, config_name = 'reach'),
        StraightLineNavigation(name = "approach jar", blackboard = blackboard),
        Manipulation(name = "close Gripper", blackboard = blackboard, config_name = 'closeGripper'),
        SetWaypoint(name="set retreat point", blackboard = blackboard, src_key = "retreat_point", direction = 'backward', dst_key = "target_point"),
        StraightLineNavigation(name = "retreat from jar", blackboard = blackboard),
        Manipulation(name = "safe", blackboard = blackboard, config_name = 'safe'),
        Planning(name = "planning to table", blackboard = blackboard, goal = (0.8, 0.05)),
        Navigation(name = "move to table", blackboard = blackboard),
        Planning(name = "planning to place", blackboard = blackboard, goal = (0.4, -2.15)),
        Navigation(name = "move to place position", blackboard = blackboard),
        Manipulation(name = "prepare to put 1", blackboard = blackboard, config_name = 'prepare_put_1'),
        Manipulation(name = "prepare to put 2", blackboard = blackboard, config_name = 'prepare_put_2'),
        Manipulation(name = "put", blackboard = blackboard, config_name = 'put'),
        Manipulation(name = "put", blackboard = blackboard, config_name = 'openGripper'),
        Manipulation(name = "after put", blackboard = blackboard, config_name = 'after_put'),
        Manipulation(name = "safe", blackboard = blackboard, config_name = 'safe'),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (-1.54, -2.96)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (-1.2, 0.1)),
        Navigation(name = "move to worktop", blackboard = blackboard),
        Planning(name = "planning to worktop", blackboard = blackboard, goal = (0.4, 0.2)),
        Navigation(name = "move to worktop", blackboard = blackboard),
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