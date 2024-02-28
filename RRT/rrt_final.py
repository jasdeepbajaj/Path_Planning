import math
import random
from typing import List
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString, Point
from node import Node


class RRT:
    """
    Implements the Rapidly-exploring Random Tree (RRT) algorithm for path planning.
    
    Attributes:
        start_node (Node): The starting node of the path.
        end_node (Node): The goal node of the path.
        x_min, y_min, x_max, y_max (float): The bounds of the search space.
        obstacleList (List[Polygon]): A list of obstacles represented as polygons.
        extend_length (float): The maximum length to extend towards each random node.
        goal_sampling_rate (int): The percentage chance of sampling the goal node as the next random node.
        max_iter (int): The maximum number of iterations to run the algorithm.
        radius_robot (float): The radius of the robot, used for collision detection.
        node_list (List[Node]): The list of nodes in the RRT tree.
    """
    def __init__(self, start: tuple, end: tuple, bounds: tuple, obstacleList: List[Polygon], extend_length : float =0.2 , goal_sampling_rate : float = 3, \
                 max_iter : int = 1000, radius_robot : float = 0.15):
        self.start_node = Node(start[0], start[1])
        self.end_node = Node(end[0], end[1])
        
        self.x_min, self.y_min, self.x_max, self.y_max =  bounds[0], bounds[0], bounds[1], bounds[1]
        
        self.obstacleList = obstacleList
        self.extend_length = extend_length
        self.goal_sampling_rate = goal_sampling_rate
        self.max_iter = max_iter
        self.radius_robot = radius_robot
        
        self.node_list = [self.start_node]
    
    def sample_random_node(self) -> Node:
        """
        Samples a random node in the search space, with a bias towards the goal node.
        
        Returns:
            Node: The sampled random node.
        """
        if random.randint(0, 100) > self.goal_sampling_rate: #random sampling in the search space
            rnd_node = Node(random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max))
        else:
            rnd_node = self.end_node #sampling towards the goal node with a bias/ goal_sampling_rate 
        return rnd_node
    
    def get_nearest_node(self, random_node: Node) -> Node:
        """
        Finds the nearest node in the tree/ node_list to the given random node.
        
        Args:
            random_node (Node): The node to find the nearest neighbor to.
        
        Returns:
            Node: The nearest node in the tree/ node_list.
        """
        dlist = [(node.x - random_node.x)**2 + (node.y - random_node.y)**2 for node in self.node_list]
        min_index = dlist.index(min(dlist))
        return self.node_list[min_index]
    
    def collision(self, new_edge: LineString, new_node: Node) -> bool:
        """
        Checks if the proposed new node or edge collides with any obstacles.
        
        Args:
            new_edge (LineString): The proposed edge from the nearest node to the new node.
            new_node (Node): The proposed new node.
        
        Returns:
            bool: True if there is a collision, False otherwise.
        """
        robot_circle = Point((new_node.x, new_node.y)).buffer(self.radius_robot)
        for obs in self.obstacleList:
            if new_edge.intersects(obs) or robot_circle.intersects(obs) or new_edge.distance(obs) < self.radius_robot:
                return True  # collision detected
        return False  # no collision detected
    
    def distance_and_angle(self, from_node: Node, to_node: Node) -> tuple:
        """
        Calculates the distance and angle between two nodes.
        
        Args:
            from_node (Node): The starting node.
            to_node (Node): The ending node.
        
        Returns:
            tuple: The distance and angle (in radians) from `from_node` to `to_node`.
        """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.sqrt(dx**2 + dy**2)
        theta = math.atan2(dy, dx)
        return d, theta
    
    def steer(self, from_node: Node, to_node: Node) -> Node:
        """
        Steers from `from_node` towards `to_node`, but only up to a maximum distance of `extend_length`.
        
        Args:
            from_node (Node): The node to steer from.
            to_node (Node): The node to steer towards.
        
        Returns:
            Node: A new node that is in same direction to `to_node` but within the `extend_length` distance from `from_node`.
        """
        d, theta = self.distance_and_angle(from_node, to_node)
        if d > self.extend_length:
            new_node = Node(from_node.x + self.extend_length * math.cos(theta), from_node.y + self.extend_length * math.sin(theta))
        else:
            new_node = to_node
        new_node.parent = from_node
        return new_node

    def plot_obstacles(self):
        """
        Plots the obstacles on the matplotlib figure.
        """
        for obs in self.obstacleList:
            x, y = obs.exterior.xy
            plt.fill(x, y, alpha=0.5, fc='r', ec='none')

    def plot_nodes(self):
        """
        Plots the nodes and their connections in the RRT tree on the matplotlib figure.
        """
        for node in self.node_list:
            plt.plot(node.x, node.y, 'ko', markersize=4)
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'g-')

    def plot_path(self, path: List[Node]):
        """
        Plots the final path from start to goal on the matplotlib figure.
        
        Args:
            path (List[Node]): The list of nodes representing the path from start to goal.
        """
        for node in path:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-', linewidth=2)

    def visualize(self, path: List[Node], save_path: str):
        """
        Visualizes the obstacles, nodes, and path of the RRT algorithm and saves the figure.
        
        Args:
            path (List[Node]): The final path from start to goal.
            save_path (str): The file path to save the visualization.
        """
        plt.figure(figsize=(10, 10))
        self.plot_obstacles()
        self.plot_nodes()
        self.plot_path(path)

        plt.plot(self.start_node.x, self.start_node.y, "bs", markersize=8, label='start')  # Start
        plt.plot(self.end_node.x, self.end_node.y, "gs", markersize=8, label='end')  # End
        plt.xlim(self.x_min, self.x_max)
        plt.ylim(self.y_min, self.y_max)
        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("RRT Path Planning")
        plt.grid(True)

        plt.savefig(save_path) 
        plt.show()  
        plt.close()  
    
    def planning(self) -> List[Node]:
        """
        Executes the RRT path planning algorithm.
        
        Returns:
            List[Node]: The list of nodes representing the path from start to goal, if a path is found; otherwise, an empty list.
        """
        for i in range(self.max_iter):
            random_node = self.sample_random_node()
            nearest_node = self.get_nearest_node(random_node)
            
            new_node = self.steer(nearest_node, random_node)

            new_edge = LineString([(new_node.x, new_node.y), (nearest_node.x, nearest_node.y)])
            if not self.collision(new_edge, new_node):
                self.node_list.append(new_node)
                if self.distance_and_angle(new_node, self.end_node)[0] < self.extend_length:
                    final_edge = LineString([(new_node.x, new_node.y), (self.end_node.x, self.end_node.y)])
                    if not self.collision(final_edge, new_node):
                        print("Reached destination")
                        self.end_node.parent = new_node
                        self.node_list.append(self.end_node)
                        return self.extract_path()  # Extract and return the path
        
        return []  # Return an empty list if no path is found within max_iter iterations

    def extract_path(self) -> List[Node]:
        """
        Extracts the path from the start node to the goal node, if a path has been found.
        
        Returns:
            List[Node]: The list of nodes representing the path from start to goal.
        """
        path = []
        current_node = self.end_node
        while current_node.parent is not None:
            path.append(current_node)
            current_node = current_node.parent
        path.append(self.start_node)  # Add start node
        return path[::-1]  # Reverse path to get it from start to end
