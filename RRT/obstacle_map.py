import random
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString

class ObstacleMap:
    def __init__(self, map_bounds: tuple, obstacles_num: int, shape: int):
        self.xmin, self.xmax, self.ymin, self.ymax = map_bounds[0], map_bounds[1], map_bounds[0], map_bounds[1]
        
        # Ensuring the polygon has at least 3 sides
        if shape < 3:
            raise Exception("Minimum number of sides in a polygon is 3.")
        else:
            self.obstacle_list = self.generate_polygon_obstacles(map_bounds, obstacles_num, shape)
    
    def generate_polygon_obstacles(self, map_bounds: tuple, obstacles_num: int, shape: int) -> list:
        """
        Generate a list of polygon obstacles within the specified map bounds.
        
        Args:
            map_bounds (tuple): Tuple containing map boundary dimensions (xmin, xmax, ymin, ymax).
            obstacles_num (int): Number of obstacles to generate.
            shape (int): Maximum number of sides for the polygons.
            
        Returns:
            list: A list of Shapely Polygon objects representing obstacles.
        """
        obstacles = []
        map_boundaries = [
            LineString([(self.xmin, self.ymin), (self.xmax, self.ymin)]),
            LineString([(self.xmax, self.ymin), (self.xmax, self.ymax)]),
            LineString([(self.xmax, self.ymax), (self.xmin, self.ymax)]),
            LineString([(self.xmin, self.ymin), (self.xmin, self.ymax)])
        ]
        min_distance = 0.5
        count = 0

        while count < obstacles_num:
            radius = random.uniform(0.07, 0.08) * self.xmax
            center = np.array([random.uniform(self.xmin, self.xmax), random.uniform(self.ymin, self.ymax)])
            pshape = random.randint(3, shape) if shape > 3 else 3
            angles_list = self.__get_angles(pshape)
            vertices = self.__get_vertices(center, radius, angles_list)
            polygon = Polygon(vertices)

            if self.__validate_obstacle(polygon, obstacles, map_boundaries, min_distance):
                obstacles.append(polygon)
                count += 1

        return obstacles

    def __get_angles(self, num_of_sides: int) -> list:
        """Generate a list of angles for polygon vertices."""
        random_numbers = [random.uniform(0, 1) for _ in range(num_of_sides)]
        normalized_numbers = [num / sum(random_numbers) for num in random_numbers]
        angles = np.cumsum([2 * np.pi * x for x in normalized_numbers])
        return list(angles)

    def __get_vertices(self, center: np.array, radius: float, angles_list: list) -> np.array:
        """Calculate vertices of a polygon based on center, radius, and angles."""
        return np.array([center + np.array([radius * np.cos(angle), radius * np.sin(angle)]) for angle in angles_list])

    def __validate_obstacle(self, polygon, obstacles, map_boundaries, min_distance) -> bool:
        """Check if a polygon is a valid obstacle based on various constraints."""
        if any(polygon.intersects(boundary) for boundary in map_boundaries):
            return False
        if any(polygon.intersects(obstacle) or polygon.distance(obstacle) < min_distance for obstacle in obstacles):
            return False
        return True

    def generate_plot(self):
        """Generate a plot to visualize the obstacle map."""
        plt.figure(figsize=(10, 6))
        plt.axis('equal')
        for obstacle in self.obstacle_list:
            x, y = obstacle.exterior.xy
            plt.fill(x, y, c="blue")
        plt.plot([self.xmin, self.xmax, self.xmax, self.xmin, self.xmin], 
                 [self.ymin, self.ymin, self.ymax, self.ymax, self.ymin], color='black')
        plt.show()

