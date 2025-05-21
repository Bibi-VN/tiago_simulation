import py_trees
import numpy as np

class StraightLineNavigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard): 

        super(StraightLineNavigation, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        
    def setup(self):
           
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        #self.marker = self.robot.getFromDef("marker").getField("translation")
        
        # Store parameters
        self.maxSpeed = 6.24
        self.wheel_radius = 0.0985
        self.distance_2wheels = 0.404
        self.p1, self.p2 = 2.5, 0.2
        
    def initialise(self):
        print("[StraightLineNavigation] Active")
        self.target_point = self.blackboard.read("target_point")
        self.target_direction = self.blackboard.read("target_direction")
        
        if self.target_point is None:
            print("[StraightLineNavigation] No target point")
           
        self.move_direction = 0
        if self.target_direction == 'forward':
            self.move_direction = 1
        elif self.target_direction == 'backward':
            self.move_direction = -1
        else:
            print(f"[StraightLineNavigation] Unknown direction '{self.target_direction}', default to stop.")
            self.move_direction = 0                                                           
        
        self.blackboard.write("navigation_active", True)
        
        
    def update(self):

        if self.target_point is None:
            self.blackboard.write("navigation_active", False)
            return py_trees.common.Status.FAILURE
            
        # Get Robot pose
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        
        # Vector to target point
        dx = self.target_point[0] - xw
        dy = self.target_point[1] - yw
        rho = np.sqrt(dx**2 + dy**2)
        
        # Default values
        delta_theta = 0.0

        # Forward case: use orientation correction
        if self.move_direction == 1:
            target_angle = np.arctan2(dy, dx)
            delta_theta = target_angle - theta
            delta_theta = (delta_theta + np.pi) % (2 * np.pi) - np.pi
            
        # Backward: lock orientation, don't rotate
        elif self.move_direction == -1:
            delta_theta = 0.0  # DO NOT turn when going backward
        
        # calculate Wheel speeds
        v_forward = 2 * self.p2 * self.move_direction * rho
        v_rotation = self.p1 * self.distance_2wheels * delta_theta

        leftSpeed = (-v_rotation + v_forward) / (2 * self.wheel_radius)
        rightSpeed = (v_rotation + v_forward) / (2 * self.wheel_radius)

        leftSpeed = max(min(leftSpeed, self.maxSpeed), -self.maxSpeed)
        rightSpeed = max(min(rightSpeed, self.maxSpeed), -self.maxSpeed)

        self.blackboard.write("velocity_left", leftSpeed)
        self.blackboard.write("velocity_right", rightSpeed)
        self.blackboard.write("navigation_active", True)
         
        if (rho < 0.1):
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
         
    def terminate(self, new_status):
        # Reset intent to stop motion
        self.blackboard.write("velocity_left", 0.0)
        self.blackboard.write("velocity_right", 0.0)
        self.blackboard.write("navigation_active", False)

         
        
        
        
