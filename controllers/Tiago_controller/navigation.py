import py_trees
import numpy as np

class Navigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, theta = None): 

        super(Navigation, self).__init__(name)
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.goal_theta = theta
        
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
        self.p1, self.p2 = 0.75, 0.3
        
    def initialise(self):
        print(f"[Navigation] {self.name}")
        self.WP = self.blackboard.read('waypoints')
        self.index = 0
        
        if self.WP is None or len(self.WP) == 0:
            print("[Navigation] No waypoints found")
            self.blackboard.write("navigation_active", False)
            self.WP = None                                                                         
        
        self.blackboard.write("navigation_active", True)
        
        
    def update(self):
        if self.WP is None:
            self.blackboard.write("navigation_active", False)
            return py_trees.common.Status.FAILURE
            
        # Get Robot pose
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])
        #print(theta)
        
        # set marker     
        #self.marker.setSFVec3f([*self.WP[self.index],0.0])
        
         # calculate Errors
        rho = np.sqrt((xw - self.WP[self.index][0])**2 + (yw - self.WP[self.index][1])**2)
        alpha = np.arctan2(self.WP[self.index][1] - yw, self.WP[self.index][0] - xw) - theta
        if alpha > np.pi:
            alpha = alpha - 2*np.pi
        elif alpha < -np.pi:
            alpha = alpha + 2*np.pi
         
        # calculate Wheel speeds
        leftSpeed = (-self.p1*self.distance_2wheels*alpha + 2*self.p2*rho)/(2*self.wheel_radius)
        rightSpeed = (self.p1*self.distance_2wheels*alpha + 2*self.p2*rho)/(2*self.wheel_radius)
        #print(f"Velocity before fixed: {leftSpeed}, {rightSpeed}")
        leftSpeed = max(min(leftSpeed, self.maxSpeed), -self.maxSpeed)
        rightSpeed = max(min(rightSpeed, self.maxSpeed), -self.maxSpeed)
        #print(f"Velocity after fixed: {leftSpeed}, {rightSpeed}")
        # Write control intent to blackboard
        self.blackboard.write("velocity_left", leftSpeed)
        self.blackboard.write("velocity_right", rightSpeed)
        self.blackboard.write("navigation_active", True)
         
        if (rho < 0.35):
            #print("Reached ", self.index, len(self.WP) - 1)
            
            if self.index == len(self.WP) - 1:
                return py_trees.common.Status.SUCCESS
            else:
                self.index += 1
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING
         
    def terminate(self, new_status):
        # Reset intent to stop motion
        self.blackboard.write("velocity_left", 0.0)
        self.blackboard.write("velocity_right", 0.0)
        self.blackboard.write("navigation_active", False)
        

         
        
        
        
