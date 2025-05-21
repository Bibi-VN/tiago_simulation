import py_trees
import numpy as np

class ObjectSelector(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, distance_threshold=0.1):
        super(ObjectSelector, self).__init__(name)
        self.blackboard = blackboard
        #self.target_model = target_model
        self.distance_threshold = distance_threshold
        
    def update(self):
        # Read recognized objects from blackboard
        recognized_objects = self.blackboard.read("recognized_objects")
        if not recognized_objects or len(recognized_objects) == 0:
            print("[ObjectSelector] no recognized objects")
            return py_trees.common.Status.FAILURE
        
        # Read picked objects list
        picked_positions = self.blackboard.read("picked_positions") or []
        
        # Filter according to model
        filtered_objects = []
        for obj in recognized_objects:
        #    if obj["model"] != self.target_model:
        #        continue

            obj_pos = np.array(obj["position"])
            is_picked = any(
                np.linalg.norm(obj_pos - np.array(p)) < self.distance_threshold
                for p in picked_positions
            )

            if not is_picked:
                filtered_objects.append(obj)
                
        if len(filtered_objects) == 0:
            print("[ObjectSelector] no recognized objects with target model")
            return py_trees.common.Status.FAILURE
            
        # Get robot position
        robot = self.blackboard.read("robot")
        gps = robot.getDevice("gps")
        pos_robot = np.array(gps.getValues())[:2]
        
        # Find the nearest object
        min_distance = float('inf')
        closest_object = None

        for obj in filtered_objects:
            obj_pos = np.array(obj["position"])
            dist = np.linalg.norm(obj_pos[:2] - pos_robot)
            if dist < min_distance:
                min_distance = dist
                closest_object = obj
                
        if closest_object is None:
            print("[ObjectSelector] failed to closest object")
            return py_trees.common.Status.FAILURE
            
        # Only print if selected object is new
        prev_target = self.blackboard.read("current_target")
        if not prev_target or not np.allclose(prev_target["position"], closest_object["position"]):
            print(f"[ObjectSelector] Selected object at: {closest_object['position']}")
            
        # Process for navigation
        pos_obj = np.array(closest_object["position"])
          
        vec = pos_obj[:2] - pos_robot
        norm = np.linalg.norm(vec)
        if norm < 1e-3:
            approach_point = pos_obj[:2]
            retreat_point = approach_point
            theta = 0.0
        else:
            vec_norm = vec / norm
            offset = 0.9
            retreat_distance = 0.9
            approach_point = pos_obj[:2] - offset * vec_norm
            retreat_point = approach_point - retreat_distance * vec_norm


            theta = np.arctan2(vec[1], vec[0])

        # Write to blackboard
        self.blackboard.write("approach_point", approach_point)
        self.blackboard.write("retreat_point", retreat_point)
        self.blackboard.write("goal_orientation", theta)
        picked_positions.append(closest_object["position"])
        self.blackboard.write("picked_positions", picked_positions)  
        return py_trees.common.Status.SUCCESS