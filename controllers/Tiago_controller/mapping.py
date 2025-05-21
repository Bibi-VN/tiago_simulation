import py_trees
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal

# World to Map function
def world2map (xw, yw):
    px = int(40*(xw + 2.25))
    py = int(-300/5.6666*(yw - 1.6633))
    px = min(px, 199)
    py = min(py, 299)
    px = max(px, 0)
    py = max(py, 0)
    return [px, py]
    
def map2world(px, py):
    xw = px/40 -2.25
    yw = py*(-5.6666/300) + 1.6633
    return xw, yw
    
class Mapping(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard):
        super(Mapping, self).__init__(name)
        
        #self.robot = blackboard.read('robot')
        #self.blackboard = blackboard
        self.hasrun = False
        self.robot = blackboard.read('robot')
        self.mapping_done = False
        self.blackboard = blackboard
        
    def setup(self):

        self.timestep = int(self.robot.getBasicTimeStep())

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)

        self.lidar = self.robot.getDevice('Hokuyo URG-04LX-UG01')
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

        self.display = self.robot.getDevice('display')

        # log debug information indicating setup has been completed
        #self.logger.debug("    %s [Mapping::setup()]" %self.name)
    def initialise(self):
        self.map = np.zeros((200, 300))
        self.angles = np.linspace(2*np.pi/3, -2*np.pi/3, 667)
        
    def update(self):
        if self.mapping_done:
            self.feedback_message = "Map saved, mapping complete"
            return py_trees.common.Status.SUCCESS

        self.hasrun = True
        xw = self.gps.getValues()[0]
        yw = self.gps.getValues()[1]
        theta = np.arctan2(self.compass.getValues()[0], self.compass.getValues()[1])

        ranges = np.array(self.lidar.getRangeImage())
        mask = np.isfinite(ranges)
        ranges = ranges[mask]
        angles = self.angles[mask]

        w_T_r = np.array([
            [np.cos(theta), -np.sin(theta), xw],
            [np.sin(theta),  np.cos(theta), yw],
            [            0,              0,  1]
        ])
        X_r = np.array([
            ranges * np.cos(angles) + 0.202,
            ranges * np.sin(angles),
            np.ones(len(ranges))
        ])
        
        D = w_T_r @ X_r

        px, py = world2map(xw, yw)
        self.display.setColor(0xFF0000)
        self.display.drawPixel(px, py)

        self.display.setColor(0xFFFFFF)
        for x, y in zip(D[0], D[1]):
            px, py = world2map(x, y)
            if 0 <= px < 200 and 0 <= py < 300:
                self.map[px, py] += 0.005
                self.map[px, py] = min(self.map[px, py], 1.0)
                intensity = int(self.map[px, py] * 255)
                color = (intensity << 16) + (intensity << 8) + intensity
                self.display.setColor(color)
                self.display.drawPixel(px, py)
                
        if self.blackboard.read("navigation_done") is True:
            # Generate configuration space via convolution
            cspace = signal.convolve2d(self.map, np.ones((26, 26)), mode='same')
            np.save('cspace.npy', cspace)
            plt.imsave('cspace.png', (cspace > 0.9).astype(np.uint8), cmap='gray')
            print("Mapping complete due to navigation finishing.")
            self.mapping_done = True
            self.feedback_message = "Navigation finished, map saved"
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status):
        print(f"Mapping node terminated with status: {new_status}")