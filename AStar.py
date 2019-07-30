from collections import deque
from math import hypot

class PathPlanner():
    """Construct a PathPlanner Object"""
    def __init__(self, M, start=None, goal=None):
        self.map = M
        self.start= start
        self.goal = goal
        self.closedSet = self.create_closedSet() if goal != None and start != None else None
        self.openSet = self.create_openSet() if goal != None and start != None else None
        self.cameFrom = self.create_cameFrom() if goal != None and start != None else None
        self.gScore = self.create_gScore() if goal != None and start != None else None
        self.fScore = self.create_fScore() if goal != None and start != None else None
        self.path = self.run_search() if self.map and self.start != None and self.goal != None else None
    
    def reconstruct_path(self, current):
        """ Reconstructs path after search """
        total_path = [current]
        while current in self.cameFrom.keys():
            current = self.cameFrom[current]
            total_path.append(current)
        return total_path
    
    def _reset(self):
        """Private method used to reset the closedSet, openSet, cameFrom, gScore, fScore, and path attributes"""
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = self.run_search() if self.map and self.start and self.goal else None

    def run_search(self):
        if self.map == None:
            raise(ValueError, "Must create map before running search. Try running PathPlanner.set_map(start_node)")
        if self.goal == None:
            raise(ValueError, "Must create goal node before running search. Try running PathPlanner.set_goal(start_node)")
        if self.start == None:
            raise(ValueError, "Must create start node before running search. Try running PathPlanner.set_start(start_node)")

        self.closedSet = self.closedSet if self.closedSet != None else self.create_closedSet()
        self.openSet = self.openSet if self.openSet != None else  self.create_openSet()
        self.cameFrom = self.cameFrom if self.cameFrom != None else  self.create_cameFrom()
        self.gScore = self.gScore if self.gScore != None else  self.create_gScore()
        self.fScore = self.fScore if self.fScore != None else  self.create_fScore()

        while not self.is_open_empty():
            current = self.get_current_node()

            if current == self.goal:
                self.path = [x for x in reversed(self.reconstruct_path(current))]
                return self.path
            else:
                self.openSet.remove(current)
                self.closedSet.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closedSet:
                    continue    # Ignore the neighbor which is already evaluated.

                if not neighbor in self.openSet:    # Discover a new node
                    self.openSet.add(neighbor)
                
                # The distance from start to a neighbor
                #the "dist_between" function may vary as per the solution requirements.
                if self.get_tentative_gScore(current, neighbor) >= self.get_gScore(neighbor):
                    continue        # This is not a better path.

                # This path is the best until now. Record it!
                self.record_best_path_to(current, neighbor)
        print("No Path Found")
        self.path = None
        return False
        
    def create_closedSet(self):
        return set()
        
    def create_openSet(self):
        if self.start != None:
            return set([self.start])
        else:
            raise(ValueError, "Must create start node before creating an open set. Try running PathPlanner.set_start(start_node)")
            
    def create_cameFrom(self): 
        return dict()
        
    def create_gScore(self):
        gScore = dict()
        for key in self.map.intersections:
            if key == self.start:
                gScore[key] = 0
            else:
                gScore[key] = float('inf')
        return gScore
        
    def create_fScore(self):
        if self.start in self.map.intersections and self.goal in self.map.intersections:
            return deque([[self.start, self.heuristic_cost_estimate(self.start)]])
        else:
            raise(ValueError, '''start and goal nodes must be in map. Try running:\n
                                 PathPlanner.set_start(start_node) or\n
                                 PathPlanner.set_goal(goal_node) or\n
                                 PathPlanner.set_map(start_node)''')
                                 
    def set_map(self, M):
        self._reset(self)
        self.start = None
        self.goal = None 
        self.map = M
        
    def set_start(self, start):
        self._reset(self)
        self.start = start
        self.goal = None
        self.closedSet = None
        self.openSet = None
        self.cameFrom = None
        self.gScore = None
        self.fScore = None
        self.path = None
        
    def set_goal(self, goal):
        self._reset(self)
        self.goal = goal
        
    def is_open_empty(self):
        return not bool(self.openSet)
        
    def get_current_node(self):
        return self.fScore.popleft()[0]
        
    def get_neighbors(self, node):
        return self.map.roads[node]
        
    def get_gScore(self, node):
        if node in self.gScore:
            return self.gScore[node]
        else:
            print("node", node, "not in gScore")
            return
        
    def distance(self, node_1, node_2):
        if node_1 in self.map.intersections and node_2 in self.map.intersections:
            node1 = self.map.intersections[node_1]
            node2 = self.map.intersections[node_2]
            return hypot(node1[0] - node2[0], node1[1] - node2[1])
        else:
            print("either node", node_1, "or", node_2, "not in map")
            return
            
    def get_tentative_gScore(self, current, neighbor):
        return self.get_gScore(current) + self.distance(current,neighbor)
        
    def heuristic_cost_estimate(self, node):
        return self.distance(node,self.goal)
        
    def calculate_fscore(self, node):
        return self.get_gScore(node) + self.heuristic_cost_estimate(node)
        
    def record_best_path_to(self, current, neighbor):
        self.cameFrom[neighbor] = current
        self.gScore[neighbor] = self.get_tentative_gScore(current, neighbor)
        
        fScore = self.calculate_fscore(neighbor)
        inserted_fScore = False
        for idx, val in enumerate(self.fScore):
            if fScore < val[1]:
                self.fScore.insert(idx,[neighbor,fScore])
                inserted_fScore = True
                break
        if not inserted_fScore:
            self.fScore.append([neighbor,fScore])
        return