import py_trees
import numpy as np
    
    # Denote:
    # The coordinate system on the robot's base where the lift joint mounted as {0}
    # The endpoint of the lift joint {1}
    # The endpoint of the joint that rotates the head {2}
    # The endpoint of the joint that tilts the head {3}
    
def camera_to_world_transform(gps, compass, sensor_values):
    xw, yw, zw = gps.getValues()
    theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    # Homogeneous transform
    # World to Robot base
    T_world_robot = np.array([
        [np.cos(theta), -np.sin(theta), 0, xw],
        [np.sin(theta),  np.cos(theta), 0, yw],
        [            0,              0, 1, zw],
        [            0,              0, 0,  1]
    ])
    
    # Robot base to torso
    T_base_torso = np.array([
        [1, 0, 0, -0.054],
        [0, 1, 0,  0.000],
        [0, 0, 1,  0.193],
        [0, 0, 0,  1]
    ])
    
    # Torso lift
    T_lift = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.6 + sensor_values['torso_lift_joint_sensor']],
        [0, 0, 0, 1]
    ])
    
    # head_1_joint
    theta_h1 = sensor_values['head_1_joint_sensor']
    T_h1 = np.array([
        [np.cos(theta_h1), -np.sin(theta_h1), 0, 0.182],
        [np.sin(theta_h1),  np.cos(theta_h1), 0, 0],
        [               0,                 0, 1, 0],
        [               0,                 0, 0, 1]
    ])
    
    # head_2_joint
    theta_h2 = sensor_values['head_2_joint_sensor']
    T_h2 = np.array([
        [np.cos(theta_h2), 0, -np.sin(theta_h2), 0.005],
        [               0, 1,                 0,     0],
        [np.sin(theta_h2), 0,  np.cos(theta_h2), 0.098],
        [               0, 0,                 0,     1]
    ])
    
    # Camera fixed transform
    T_head_rotation = np.array([
        [1, 0,  0, 0],
        [0, 0, -1, 0],
        [0, 1,  0, 0],
        [0, 0,  0, 1]
    ])

    T_camera_offset = np.array([
        [1, 0, 0, 0.107],
        [0, 1, 0, 0.0802],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    T_base_camera = (
        T_world_robot @
        T_base_torso @
        T_lift @
        T_h1 @
        T_h2 @
        T_head_rotation @
        T_camera_offset
    )

    return T_base_camera
    
    

class Recognition(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, distance_threshold = 0.0):
        super(Recognition, self).__init__(name)
        
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.distance_threshold = distance_threshold
        self.robot_sensors = {
            'torso_lift_joint_sensor': None,
            'head_1_joint_sensor': None,
            'head_2_joint_sensor': None
        }
        
        # Store head sensor values
        self.latest_sensor_values  = {}
        
    
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(self.timestep)
        
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        
        for sensor_name in self.robot_sensors.keys():
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.timestep)
            self.robot_sensors[sensor_name] = sensor
            
        
    def initialise(self):
        self.latest_sensor_values.clear()
        
    def update(self):
        #print("[Recognition] Tick started") 
        # Get the sensor values
        for sensor_name, sensor in self.robot_sensors.items():
            self.latest_sensor_values[sensor_name] = sensor.getValue()
            #print("Sensor values ", sensor_name,": ", sensor_value)
            
        T_world_camera = camera_to_world_transform(
            gps = self.gps,
            compass = self.compass, 
            sensor_values = self.latest_sensor_values
        )
        recognized = self.blackboard.read("recognized_objects") or []
        objects = self.camera.getRecognitionObjects()
        #print(f"[Recognition] Detected {len(objects)} objects")
        added = 0
        
        for obj in objects:
                
            pos_cam = np.array(list(obj.getPosition()) + [1]) # (x, y, z, 1)
            pos_world = (T_world_camera @ pos_cam)[:3]
            
            is_duplicate = any(
                np.linalg.norm(np.array(item["position"]) - pos_world) < self.distance_threshold
                for item in recognized
            )
            
            if not is_duplicate:
                print(f"[Recognition] Recognized object at: {pos_world}")
                recognized.append({"position": pos_world})
                added += 1
                
        self.blackboard.write("recognized_objects", recognized)
        
        if added > 0:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING             

        
        
    