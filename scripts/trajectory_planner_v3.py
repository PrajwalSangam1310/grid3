from typing import Dict, List, Set, Tuple, Union
from collections import deque
import heapq

  
class Node:

    def __init__(self, location: Tuple = None, parent = None, total_weight: int = None, depth: int = None, orientation: int = None) -> None:
        self.location: Tuple = location
        self.parent: Node = parent
        self.total_weight: int = total_weight
        self.depth: int = depth
        self.robot_orientation: int = orientation
    
    def __lt__(self, other):
        if self.total_weight < other.total_weight:
            return True
        elif self.total_weight == other.total_weight:
            return self.depth < other.depth
        else:
            return False

class TrajectoryPlanner: 
    dl = {
        0 : {},
        1 : {(2,3),(2,4),(3,2),(4,2),(5,3),(5,4),(3,5),(4,5)},
        2 : {(2,7),(2,8),(3,6),(4,6),(5,7),(5,8),(3,9),(4,9)},
        3 : {(2,11),(2,12),(3,10),(4,10),(5,11),(5,12),(3,13),(4,13)},
        4 : {(6,3),(6,4),(7,2),(8,2),(9,3),(9,4),(7,5),(8,5)},
        5 : {(6,7),(6,8),(7,6),(8,6),(9,7),(9,8),(7,9),(8,9)},
        6 : {(6,11),(6,12),(7,10),(8,10),(9,11),(9,12),(7,13),(8,13)},
        7 : {(10,3),(10,4),(11,2),(12,2),(13,3),(13,4),(11,5),(12,5)},
        8 : {(10,7),(10,8),(11,6),(12,6),(13,7),(13,8),(11,9),(12,9)},
        9 : {(10,11),(10,12),(11,10),(12,10),(13,11),(13,12),(11,13),(12,13)},
        10: {(5,15), (10,15)},
        11: {(5,15), (10,15), (1,9),(14,9)}

    }
    GRID = [
        [-1]*16,
        [-1]+[0]*14+[-1],
        [-1]+[0]*14+[-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1]+[0]*14+[0],
        [-1]+[0]*14+[-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1]+[0]*14+[-1],
        [-1]+[0]*14+[0],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1]+[0]*14+[-1],
        [-1]+[0]*14+[-1],
        [-1]*16
    ]

    with_holding_points = [(1,9),(14,9)]
    def __init__(self, grid: List[List[int]], robot_start_locations: List[Tuple[int,int]], robot_directions: List[str]) -> None:
        self.grid: List[List[int]] = grid
        self.number_of_robots: int = len(robot_start_locations)
        self.robots: List[Robot] = [] 
        for index in range(self.number_of_robots):
            self.robots.append(
                Robot(
                    start_location=robot_start_locations[index],
                    start_direction=robot_directions[index]
                )
            )
        
    def find_trajectory(self, robot_index: int, goal_location: Set[Tuple[int,int]]) -> Union[int, deque]:
        '''
        Sets the trajectory of the robot.
        Parameters:
            robot_index: Index of the robot whoose trajectory needs to be done
            goal_location: Set of points. Robot has option to reach any of the points in the set.
        Returns:
            If a path is found, the robot trajectory is set to that path, and a deque is returned
            Otherwise integer -1 is returned
        '''
        final_nodes = {}
        priority_list = [Node(location = self.robots[robot_index].location, total_weight=0, depth=0, orientation=self.robots[robot_index].orientation)]
        path_found = False
        while priority_list:
            current_node = heapq.heappop(priority_list)
            final_nodes[current_node.location] = current_node
            if current_node.location in goal_location:
                path_found = current_node
                break
            neighbors, add_extra_cost = self._get_neighbors(robot_index, current_node, final_nodes)
            for neighbor in neighbors:
                new_node = Node(location=neighbor, parent=current_node, depth = current_node.depth+1)
                # Update Orientation of new_node
                new_node.robot_orientation = self._change_orientation(current_node.location, new_node.location, current_node.robot_orientation)
                # Calculate weight
                new_node.total_weight = self._calculate_weight(new_node)
                if self.with_holding_points[0] in goal_location and neighbor in self.with_holding_points:
                    new_node.total_weight += 50

                if add_extra_cost:
                    new_node.total_weight += 100
                # Add or Reject in priority list based on if present already or not also considering weight
                self._update_priority(priority_list, new_node)
        if path_found:
            trajectory = deque()
            while current_node.location != self.robots[robot_index].location:
                trajectory.appendleft(current_node.location)
                current_node = current_node.parent
            self.robots[robot_index].current_trajectory = trajectory
            return trajectory
        else:
            self.robots[robot_index].current_trajectory = None
            return -1
    
    def _get_neighbors(self, robot_index: int, current_node: Node, final_nodes: Dict):
        neighbors = self._get_four_neighbors(current_node) - final_nodes.keys()
        collisions = self._potential_collision(robot_index, current_node)
        if collisions == -1:
            return ([],False)
        (reopenable_colliding_nodes, closed_colliding_nodes) = collisions
        neighbors = neighbors - closed_colliding_nodes
        possible_neighbors = len(neighbors)
        neighbors = neighbors - reopenable_colliding_nodes
        valid_neighbors = len(neighbors)
        add_extra_cost = False
        if valid_neighbors < possible_neighbors:
            neighbors.add(self._to_weight_nodes(current_node.location))
            add_extra_cost = True
        return (list(neighbors), add_extra_cost)
    
    def _potential_collision(self, robot_index: int, current_node: Node):
        reopenable_colliding_nodes = set()
        closed_colliding_nodes = set()
        current_depth = current_node.depth
        depths_to_check = (current_depth, current_depth+1, current_depth+2, current_depth+3)
        for i in range(self.number_of_robots):
            if i == robot_index:
                continue
            robot_path = self.robots[i].current_trajectory
            if robot_path is None or len(robot_path) == 0:
                closed_colliding_nodes.add((self.robots[i].location[0], self.robots[i].location[1]))
                continue
            else:
                if len(robot_path) >= current_depth and current_depth > 0 and robot_path[current_depth-1][0] == current_node.location[0] and robot_path[current_depth-1][1] == current_node.location[1]:
                    return -1
                for depth in depths_to_check:
                    if len(robot_path) <= depth:
                        closed_colliding_nodes.add((robot_path[-1][0], robot_path[-1][1]))
                    elif depth > 0:
                        reopenable_colliding_nodes.add((robot_path[depth-1][0], robot_path[depth-1][1]))
                    elif depth == 0:
                        reopenable_colliding_nodes.add((self.robots[i].location[0], self.robots[i].location[1]))
        return reopenable_colliding_nodes, closed_colliding_nodes
    

    def _get_four_neighbors(self, current_node: Node) -> Set[Tuple[int,int]]:
        directions = [(1,0),(-1,0),(0,1),(0,-1)]
        inbounds = lambda x,y: x>=0 and y>=0 and x<len(self.grid) and y<len(self.grid[0])
        neighbors = set()
        for direction in directions:
            poss = (current_node.location[0] + direction[0], current_node.location[1] + direction[1])
            if inbounds(poss[0],poss[1]) and self.grid[poss[0]][poss[1]] >= 0:
                neighbors.add(poss)
        return neighbors
    
    @staticmethod
    def _to_weight_nodes(location: Tuple[int,int]) -> Tuple[int,int,int]:
        if len(location) == 3:
            return (location[0],location[1],location[2]+1)
        else:
            return (location[0],location[1],1)
    
    @staticmethod
    def _calculate_weight(new_node: Node) -> int:
        current_node = new_node.parent
        turn_weight = (abs(current_node.robot_orientation - new_node.robot_orientation) % 4) * 15
        if len(new_node.location) == 3:
            # Robot is waiting
            move_weight = 5
        else:
            # Robot is moving
            move_weight = 10
        return current_node.total_weight + turn_weight + move_weight

    def _update_priority(self, heap: List[Node], node: Node) -> None:
        element_present = False
        for i in range(len(heap)):
            if heap[i].location == node.location:
                element_present = True
                break
        if element_present:
            if heap[i].total_weight > node.total_weight:
                heap[i] = node
                if node.total_weight > heap[i].total_weight:
                    heapq._siftup(heap,i)
                elif node.total_weight < heap[i].total_weight:
                    heapq._siftdown(heap, 0, i)
        else:
            heapq.heappush(heap,node)
    
    @staticmethod
    def _change_orientation(initial_location: Tuple, final_location: Tuple, old_orientation: int) -> int:
        y_diff = final_location[0]-initial_location[0]
        x_diff = final_location[1]-initial_location[1]
        if x_diff == 0 and y_diff == 0:
            return old_orientation
        return (y_diff + x_diff - (1-x_diff)*(-1-x_diff))%4

    def update_robot_location(self, robot_index: int) -> None:
        '''
        The location, orientation and trajectory of the robot is updated to the next trajectory point
        Parameter:
            robot_index: Index of the robot who has reached the next point in it's trajectory
        '''
        robot = self.robots[robot_index]
        if robot.current_trajectory is None:
            return
        if len(robot.current_trajectory) == 0:
            print('Robot', robot_index, 'reached destination. ')
            robot.current_trajectory = None
            return
        robot.orientation = self._change_orientation(robot.location, robot.current_trajectory[0], robot.orientation)
        robot.location = robot.current_trajectory[0]
        robot.current_trajectory.popleft()
        if len(robot.current_trajectory) == 0:
            robot.current_trajectory = None

    def force_update_location(self, robot_index: int, new_location: Tuple[int,int] = None, new_direction: str = None):
        if new_location is not None:
            self.robots[robot_index].location = new_location
        if new_direction is not None:
            self.robots[robot_index].orientation = Robot.direction_to_orientation(new_direction)
        self.robots[robot_index].current_trajectory = None

class Robot:
    def __init__(self, start_location: Tuple, start_direction: str) -> None:
        self.location: Union[Tuple[int,int],Tuple[int,int,int]] = start_location
        self.orientation: int = Robot.direction_to_orientation(start_direction)
        self.current_trajectory: deque = None
    
    @staticmethod
    def direction_to_orientation(direction: str) -> int:
        if direction == 'N':
            orientation = 0
        elif direction == 'E':
            orientation = 1
        elif direction == 'S':
            orientation = 2
        elif direction == 'W':
            orientation = 3
        else:
            raise ValueError('Not a valid direction: ' + direction)
        return orientation
    
    def check_direction(self) -> None:
        if self.orientation > 3 or self.orientation < 0:
            raise ValueError('Orientation is out of bounds: ' + self.orientation)

    def get_orientation(self):
        if self.orientation == 0:
            return 'N'
        elif self.orientation == 1:
            return 'E'
        elif self.orientation == 2:
            return 'S'
        elif self.orientation == 3:
            return 'W'

    def get_next_point(self):
        if self.current_trajectory is None:
            return None
        else:
            return self.current_trajectory[0]



if __name__ == '__main__':
    # GRID = [[-1]*6 + [-1,-1,-1,-1] + [-1]*6] + \
    #     ([[-1] + [0]*14 + [-1]]*2 + \
    #     [[-1] + [0,0,-2,-2]*3 + [0,0,-1]]*2)*3 +\
    #     [[-1] + [0]*14 + [-1]]*2 +\
    #     [[-1]*6 + [-1,-1,-1,-1] + [-1]*6]
    # SMALL_GRID = [
    #     [0,-1,0,0],
    #     [-1,-1,-1,0],
    #     [0,0,0,0],
    #     [0,0,0,0]
    # ]

    GRID = [
        [-1]*16,
        [-1]+[0]*14+[-1],
        [-1]+[0]*14+[-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1]+[0]*14+[0],
        [-1]+[0]*14+[-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1]+[0]*14+[-1],
        [-1]+[0]*14+[0],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1,0,0,-2,-2,0,0,-2,-2,0,0,-2,-2,0,0,-1],
        [-1]+[0]*14+[-1],
        [-1]+[0]*14+[-1],
        [-1]*16
    ]


    # TrajectoryPlanner(grid=GRID, robot_start_locations=[(0,6)], robot_directions=['S']).find_trajectory(0,[(4,9)])

    # brain = TrajectoryPlanner(grid=GRID, robot_start_locations=[(0,6),(0,9),(2,12),(1,12)], robot_directions=['S','S','W','W'])
    # print(brain.find_trajectory(0,{(1,2)}))
    # print(brain.find_trajectory(1,{(5,5)}))
    # print(brain.find_trajectory(3,{(8,14)}))
    # print(brain.find_trajectory(2,{(3,9)}))
    # brain.update_robot_location(0)
    # print(brain.find_trajectory(0,{(1,2)}))
    # print(brain.find_trajectory(1,{(5,5)}))
    # print(brain.find_trajectory(3,{(8,14)}))
    # print(brain.find_trajectory(2,{(3,9)}))
    # brain.update_robot_location(1)
    # brain.update_robot_location(1)
    # print(brain.find_trajectory(0,{(1,2)}))
    # print(brain.find_trajectory(1,{(5,5)}))
    # print(brain.find_trajectory(3,{(8,14)}))
    # print(brain.find_trajectory(2,{(3,9)}))

    # brain = TrajectoryPlanner(grid=SMALL_GRID, robot_start_locations=[(3,3),(0,3)], robot_directions=['E','N'])
    # print(brain.find_trajectory(0,{(3,-4)}))
    # print(brain.find_trajectory(0,{(0,3)}))

    # brain = TrajectoryPlanner(grid=GRID, robot_start_locations=[(4,5),(5,10)], robot_directions=['S','W'])
    # print(brain.find_trajectory(0,{(5,7)}))
    # print(brain.find_trajectory(1,{(5,5)}))
    # brain = TrajectoryPlanner(grid=GRID, robot_start_locations=[(5,5),(5,9)], robot_directions=['S','W'])
    # print(brain.find_trajectory(1,{(5,5)}))
    # print(brain.find_trajectory(0,{(5,7)}))
    # print(brain.find_trajectory(1,{(5,5)}))

    # brain = TrajectoryPlanner(grid=GRID, robot_start_locations=[(1,2),(1,4)], robot_directions=['E','W'])
    # print(brain.find_trajectory(0,{(1,5)}))
    # print(brain.find_trajectory(1,{(1,1)}))

    brain = TrajectoryPlanner(grid=GRID, robot_start_locations=[(5,15),(10,15),(0,0),(0,0)], robot_directions=['W','W','N','N'])
    print(brain.find_trajectory(0,{(2, 4), (5, 4), (4, 2), (2, 3), (4, 5), (5, 3), (3, 2), (3, 5)})
)
    # {(2, 4), (5, 4), (4, 2), (2, 3), (4, 5), (5, 3), (3, 2), (3, 5)}

    print(brain.find_trajectory(1,brain.dl[2]))
