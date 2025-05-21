import py_trees
import numpy as np
import copy

class Manipulation(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, config_name = 'safe'):
        super(Manipulation, self).__init__(name)
        
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.config_name = config_name
        self.threshold = 0.05
        self.threshold_force = -10
        
        self.configurations = {
            'safe' : {
                'torso_lift_joint' : 0.05,
                'arm_1_joint' : 1.600,
                'arm_2_joint' : np.pi/4,
                'arm_3_joint' : -2.815,
                'arm_4_joint' : 0.8854,
                'arm_5_joint' : 0,
                'arm_6_joint' : 0,
                'arm_7_joint' : np.pi/2,
                'gripper_left_finger_joint' : 0,
                'gripper_right_finger_joint': 0,
                'head_1_joint':0,
                'head_2_joint':0
            },
            
            'reach' : {
                'torso_lift_joint' : 0.11, #0.35
                'arm_1_joint' : 1.600,
                'arm_2_joint' : np.pi/4,
                'arm_3_joint' : 0.0,
                'arm_4_joint' : 0.8854,
                'arm_5_joint' : 0.0,
                'arm_6_joint' : 0.0,
                'arm_7_joint' : np.pi/2,
                'gripper_left_finger_joint' : 0.045,
                'gripper_right_finger_joint': 0.045,
                'head_1_joint':0,
                'head_2_joint':0
            },
            
            'closeGripper' : {
                'torso_lift_joint' : 0.11, #0.35
                'arm_1_joint' : 1.600,
                'arm_2_joint' : np.pi/4,
                'arm_3_joint' : 0.0,
                'arm_4_joint' : 0.8854,
                'arm_5_joint' : 0.0,
                'arm_6_joint' : 0.0,
                'arm_7_joint' : np.pi/2,
                'gripper_left_finger_joint' : 0.045,
                'gripper_right_finger_joint': 0.045,
                'gripper_left_finger_joint' : 0.0,
                'gripper_right_finger_joint' : 0.0,
            },
            
            'prepare_put_1' : {
                'torso_lift_joint' : 0.2, #0.35
                'arm_1_joint' : 1.600,
                'arm_2_joint' : 0.9,
                'arm_3_joint' : 0.0,
                'arm_4_joint' : 1.2,
                'arm_5_joint' : 0.0,
                'arm_6_joint' : 0.0,
                'arm_7_joint' : np.pi/2,
                'gripper_left_finger_joint' : 0.0,
                'gripper_right_finger_joint': 0.0,
                'head_1_joint':0,
                'head_2_joint':0
            },
            
            'prepare_put_2' : {
                'torso_lift_joint' : 0.2, #0.35
                'arm_1_joint' : 0.1,
                'arm_2_joint' : 0.9,
                'arm_3_joint' : 0.0,
                'arm_4_joint' : 1.2,
                'arm_5_joint' : 0.0,
                'arm_6_joint' : 0.0,
                'arm_7_joint' : np.pi/2,
                'gripper_left_finger_joint' : 0.0,
                'gripper_right_finger_joint': 0.0,
                'head_1_joint':0,
                'head_2_joint':0
            },
            
            'put' : {
                'torso_lift_joint' : 0.11, #0.35
                'arm_1_joint' : 0.1,
                'arm_2_joint' : 0.9,
                'arm_3_joint' : 0.0,
                'arm_4_joint' : 1.2,
                'arm_5_joint' : 0.0,
                'arm_6_joint' : 0.0,
                'arm_7_joint' : 1.576,
                'gripper_left_finger_joint' : 0.0,
                'gripper_right_finger_joint': 0.0,
                'head_1_joint':0,
                'head_2_joint':0
            },
            
            'openGripper' : {
                'torso_lift_joint' : 0.09, #0.35
                'arm_1_joint' : 0.1,
                'arm_2_joint' : 0.9,
                'arm_3_joint' : 0.0,
                'arm_4_joint' : 1.2,
                'arm_5_joint' : 0.0,
                'arm_6_joint' : 0.0,
                'arm_7_joint' : 1.57,
                'gripper_left_finger_joint' : 0.045,
                'gripper_right_finger_joint': 0.045,
                'head_1_joint':0,
                'head_2_joint':0
            },
            
            'after_put' : {
                'torso_lift_joint' : 0.21, #0.35
                'arm_1_joint' : 0.1,
                'arm_2_joint' : 0.9,
                'arm_3_joint' : 0.0,
                'arm_4_joint' : 1.2,
                'arm_5_joint' : 0.0,
                'arm_6_joint' : 0.0,
                'arm_7_joint' : np.pi/2,
                'gripper_left_finger_joint' : 0.045,
                'gripper_right_finger_joint': 0.045,
                'head_1_joint':0,
                'head_2_joint':0
            },
      
        }
       
        self.robot_sensors = {
            'torso_lift_joint_sensor' : None,
            'arm_1_joint_sensor' : None,
            'arm_2_joint_sensor' : None,
            'arm_3_joint_sensor' : None,
            'arm_4_joint_sensor' : None,
            'arm_5_joint_sensor' : None,
            'arm_6_joint_sensor' : None,
            'arm_7_joint_sensor' : None,
            'gripper_left_sensor_finger_joint' : None,
            'gripper_right_sensor_finger_joint': None,
            'head_1_joint_sensor': None,
            'head_2_joint_sensor': None}
            
        self.joint_to_sensor = {
            'torso_lift_joint': 'torso_lift_joint_sensor',
            'arm_1_joint': 'arm_1_joint_sensor',
            'arm_2_joint': 'arm_2_joint_sensor',
            'arm_3_joint': 'arm_3_joint_sensor',
            'arm_4_joint': 'arm_4_joint_sensor',
            'arm_5_joint': 'arm_5_joint_sensor',
            'arm_6_joint': 'arm_6_joint_sensor',
            'arm_7_joint': 'arm_7_joint_sensor',
            'gripper_left_finger_joint': 'gripper_left_sensor_finger_joint',
            'gripper_right_finger_joint': 'gripper_right_sensor_finger_joint',
            'head_1_joint': 'head_1_joint_sensor',
            'head_2_joint': 'head_2_joint_sensor'
        }
        
        if self.config_name not in self.configurations:
            print(f"[Warning] Unknown configuration '{self.config_name}', fallback to 'safe'")
            self.config_name = 'safe'

        self.robot_joints = copy.deepcopy(self.configurations[self.config_name])
        
        self.robot_motors = {}
        self.robot_sensor_values = {}
        
    def setup(self):
        self.timestep = int(self.robot.getBasicTimeStep())
        
        
        for joint_name, position in self.robot_joints.items():
            motor = self.robot.getDevice(joint_name)
            self.robot_motors[joint_name] = motor
            
        for sensor_name in self.robot_sensors.keys():
            sensor = self.robot.getDevice(sensor_name)
            sensor.enable(self.timestep)
            self.robot_sensors[sensor_name] = sensor
        
    def initialise(self):
        print(f"[Manipulation] Using configuration: '{self.config_name}'")
        for joint_name, target_pos in self.robot_joints.items():
            motor = self.robot_motors.get(joint_name)
            if motor is None:
                print(f"[ERROR] Could not find motor: {joint_name}")
                continue
            motor.setPosition(target_pos)
            
        if self.config_name == 'closeGripper':
            left_motor = self.robot_motors.get('gripper_left_finger_joint')
            right_motor = self.robot_motors.get('gripper_right_finger_joint')
    
            if left_motor:
                left_motor.enableForceFeedback(self.timestep)
            else:
                print("[ERROR] Left gripper motor not found.")
    
            if right_motor:
                right_motor.enableForceFeedback(self.timestep)
            else:
                print("[ERROR] Right gripper motor not found.")

            
    def update(self):
    
        all_matched = True
        
        for joint_name, target_pos in self.robot_joints.items():
            sensor_name = self.joint_to_sensor.get(joint_name)
            
            if sensor_name is None:
                print(f"[Warning] No sensor mapping for joint: {joint_name}")
                continue
                
            sensor = self.robot_sensors.get(sensor_name)
            
            if sensor is None:
                print(f"[Warning] Sensor '{sensor_name}' not found in robot_sensors.")
                continue
            
            current_value = sensor.getValue()
            self.robot_sensor_values[sensor_name] = current_value
            
            error = abs(current_value - target_pos)
            #print(f"{sensor_name}: {current_value:.4f} (target: {target_pos}, error: {error:.4f})")
            
            if error > self.threshold:
                all_matched = False
                
        if self.config_name == 'closeGripper':
            # Check both joints and forces
            if all_matched:
                threshold_force = self.threshold_force
                left_motor = self.robot_motors.get('gripper_left_finger_joint')
                right_motor = self.robot_motors.get('gripper_right_finger_joint')
                
                if left_motor and right_motor:
                    left_force = left_motor.getForceFeedback()
                    right_force = right_motor.getForceFeedback()
                    #print(f"[Force Feedback] Left: {left_force:.2f}, Right: {right_force:.2f}")
                
                    if left_force < threshold_force and right_force < threshold_force:
                        print("[INFO] All joints and gripper force matched.")
                        return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.RUNNING
                
        if all_matched:
            print("[Manipulation] All joints reached target positions.")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            
