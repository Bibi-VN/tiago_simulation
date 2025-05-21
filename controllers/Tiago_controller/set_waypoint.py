import py_trees

class SetWaypoint(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, src_key, direction, dst_key):
        super(SetWaypoint, self).__init__(name)
        self.blackboard = blackboard
        self.src_key = src_key
        self.dst_key = dst_key
        self.direction = direction

    def update(self):
        wp = self.blackboard.read(self.src_key)
        if wp is None:
            print(f"[SetWaypoint] Failed: No waypoint found in key '{self.src_key}'")
            return py_trees.common.Status.FAILURE
        
        self.blackboard.write(self.dst_key, wp)
        self.blackboard.write("target_direction", self.direction)
        print(f"[SetWaypoint] Set {self.dst_key} ← {self.src_key}, direction: {self.direction}")
        return py_trees.common.Status.SUCCESS
