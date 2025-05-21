#!/home/bibi/Webots_Projects/venv/bin/python3
import py_trees
import numpy as np
import heapq
import matplotlib.pyplot as plt
from collections import defaultdict
from skimage.draw import random_shapes, line_nd
from scipy.ndimage import distance_transform_edt

# World to Map function
def world2map (xw, yw):
    px = int(40*(xw + 2.25))
    py = int(-300/5.6666*(yw - 1.6633))
    px = min(px, 199)
    py = min(py, 299)
    px = max(px, 0)
    py = max(py, 0)
    return (px, py)
    
def map2world(px, py):
    xw = px/40 -2.25
    yw = py*(-5.6666/300) + 1.6633
    return xw, yw
    
def getShortestPath(obstacle_map, start, goal):

    def get_neighbors_with_cost(u, obstacle_map):
        neighbors = []
        rows, cols = len(obstacle_map), len(obstacle_map[0])
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),
            (-1, -1), (1, 1), (-1, 1), (1, -1)
        ]
        for dx, dy in directions:
            x, y = u[0] + dx, u[1] + dy # add row with row, column with column
            if (0 <= x < rows and 0 <= y < cols and not obstacle_map[x][y]):
                costvu = np.hypot(dx, dy)
                neighbors.append(((x, y), costvu))
        return neighbors
        
    def heuristic(a, b):
        return np.hypot (a[0] - b[0], a[1] - b[1])
        
    # A* initialization
    visited = set() #set
    distances = defaultdict(lambda:float("inf"))
    distances[start] = 0
    parent = {} #create new empty dictionary
    queue = [] #list
    heapq.heappush(queue, (0, start)) # (cost, node)
    
    while queue:
        current_cost, u = heapq.heappop(queue)
        if u in visited:
            continue
        visited.add(u)
        if u == goal:
            break

        for v, move_cost in get_neighbors_with_cost(u, obstacle_map):
            newcost = distances[u] + move_cost
            if newcost < distances[v]:
                distances[v] = newcost
                parent[v] = u
                priority = newcost + heuristic(v, goal)
                heapq.heappush(queue, (priority, v))

    # Reconstruct path
    def reconstruct_path(parent, start, goal):
        if goal not in parent:
            return None
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        path.reverse()
        return path
        
    return reconstruct_path(parent, start, goal)
    
def sparsify_path (world_path, threshold):
    if not world_path:
        return []
    sparse = [world_path[0]]
    last = world_path[0]
    for pt in world_path[1:]:
        dist = np.sqrt((pt[0] - last[0])**2 + (pt[1] - last[1])**2)
        if dist >= threshold:
            sparse.append(pt)
            last = pt
    return sparse
    
class Planning(py_trees.behaviour.Behaviour):
    def __init__(self, name, blackboard, goal):
        super(Planning, self).__init__(name)
    
        self.robot = blackboard.read('robot')
        self.blackboard = blackboard
        self.goal_px_py = None
        self.map = None
        self.goal_pos = goal
        self.background_color = 0x000000
        self.previous_path_pixels = []
        
    def setup(self):
        self.display = self.robot.getDevice('display')
            
    def initialise(self):
        print("[planning]", self.name)
        # clear previous path
        self.blackboard.write("path", None)
        
        if self.goal_pos is None:
            print("[Planning] No goal_position found on blackboard")
            self.goal_px_py = None
        else:
            self.goal_px_py = world2map(self.goal_pos[0], self.goal_pos[1])

        try:
            self.map = np.load("cspace.npy") > 0.9
        except FileNotFoundError:
            print("Planning Error: cspace.npy not found")
            self.map = None
            
        
    def update(self):
        if self.map is None:
            print("[Planning] cspace.npy not found")
            return py_trees.common.Status.FAILURE
        
        # If goal is occupied by obstacles, find the nearest one    
        gx, gy = self.goal_px_py
        if gx >= 200 or gy >= 300 or self.map[gx][gy]:
            print(f"[Planning] Goal {self.goal_px_py} is blocked, finding nearest free point...")
            dist_map = distance_transform_edt(~self.map)
            free_indices = np.argwhere(dist_map > 1.0)
            if len(free_indices) == 0:
                print("[Planning] No reachable free space found near goal")
                self.goal_px_py = None
                return py_trees.common.Status.FAILURE
             
            dists = np.linalg.norm(free_indices - np.array([gx, gy]), axis=1)
            new_gx, new_gy = free_indices[np.argmin(dists)]
            self.goal_px_py = (int(new_gx), int(new_gy))
            print(f"[Planning] Adjusted goal: {self.goal_px_py}")
            
        # get current robot position
        xw = self.robot.getDevice('gps').getValues()[0]
        yw = self.robot.getDevice('gps').getValues()[1]
        start = tuple(world2map(xw, yw))
        
        # Fix if start is in obstacle
        if self.map[start[0]][start[1]]:
            #print(f"[Planning] Start point {start} is occupied, finding nearest free space...")
            dist_map = distance_transform_edt(~self.map)
            free_indices = np.argwhere(dist_map > 1.0)
            if len(free_indices) == 0:
                print("[Planning] No reachable free space found near robot")
                return py_trees.common.Status.FAILURE
            dists = np.linalg.norm(free_indices - np.array(start), axis=1)
            new_start = tuple(free_indices[np.argmin(dists)])
            #print(f"[Planning] Adjusted start: {new_start}")
            start = new_start
        
        path = getShortestPath(self.map, start, self.goal_px_py)
        
        if path is None:
            print("[Planning] Goal is not appropriate") 
            return py_trees.common.Status.FAILURE
        else:
            self.blackboard.write("path", path)
            
            # convert path to world-space waypoints
            world_path = [map2world(px, py) for (px, py) in path]
            sparse_path = sparsify_path(world_path, threshold = 0.5)
            goal_world = map2world(*self.goal_px_py)
            # ensure that last waypoint is exactly the goal
            dx = sparse_path[-1][0] - goal_world[0]
            dy = sparse_path[-1][1] - goal_world[1]
            if not sparse_path or (dx**2 + dy**2 > 0.01):
                sparse_path.append(goal_world)
            
            self.blackboard.write("waypoints", sparse_path)
            
            # Clear the entire display area
            self.display.setColor(self.background_color)
            self.display.fillRectangle(0, 0, 200, 300)
            self.previous_path_pixels = []
            
            # Draw path
            self.display.setColor(0x00FF00)
            for i in range(len(path) - 1):
                x0, y0 = path[i]         # directly use pixel coordinates
                x1, y1 = path[i + 1]
                rr, cc = line_nd((x0, y0), (x1, y1))
                for px, py in zip(rr, cc):
                    if 0 <= px < 200 and 0 <= py < 300:
                        self.display.drawPixel(px, py)
                        self.previous_path_pixels.append((px, py))
            return py_trees.common.Status.SUCCESS
            
    def terminate(self, newstatus):
        pass
       