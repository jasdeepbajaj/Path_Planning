import random
from shapely.geometry import Point
from obstacle_map import ObstacleMap 
from rrt_final import RRT  
import os

def sample_non_intersecting_start_end(bounds, obstacleList, num_attempts=1000):
    """
    Samples non-intersecting start and end points within the given bounds that do not intersect with any obstacles.
    
    Args:
        bounds (tuple): The bounds of the area, specified as (min, max) for both x and y coordinates.
        obstacleList (list): A list of obstacles represented as shapely geometries.
        num_attempts (int, optional): The number of attempts to sample start and end points. Defaults to 1000.
    
    Returns:
        tuple: A tuple containing the start and end points as (x, y) coordinates, or None if no suitable points were found.
    """
    x_min, y_min, x_max, y_max = bounds[0], bounds[0], bounds[1], bounds[1]
    for _ in range(num_attempts):
        start = Point(random.uniform(x_min, x_max), random.uniform(y_min, y_max))
        end = Point(random.uniform(x_min, x_max), random.uniform(y_min, y_max))
        
        # Check if start or end points intersect with any obstacle
        if any(start.intersects(obstacle) or end.intersects(obstacle) for obstacle in obstacleList):
            continue  # If so, retry
        
        return (start.x, start.y), (end.x, end.y)
    
    # If no suitable points found after all attempts, return None
    print("Failed to sample non-intersecting start and end points within the given bounds and obstacle constraints.")
    return None

def main():
    """
    Main function to execute the RRT path planning.
    """
    # Define the search area bounds and obstacles characteristics
    bounds = (-10.0, 10.0)
    obstacles_num = 100
    shape = 4  # Assuming 'shape' defines some characteristic of the obstacles
    
    # Create an obstacle map with the specified parameters
    obstacle_map = ObstacleMap(bounds, obstacles_num, shape)
    obstacleList = obstacle_map.obstacle_list
    
    # Sample non-intersecting start and end points
    start, end = sample_non_intersecting_start_end(bounds, obstacleList)
    
    # RRT parameters
    goal_sampling_rate = 1.0
    extend_length = 0.30
    max_iter = 5000
    radius_robot = 0.15
    
    # If start and end points were successfully sampled, proceed with RRT
    if start and end:
        rrt = RRT(start, end, bounds, obstacleList, extend_length=extend_length, max_iter=max_iter, radius_robot=radius_robot, goal_sampling_rate=goal_sampling_rate)
        path = rrt.planning()
        
        # If a path was found, visualize and save the result
        if path:
            destination_folder = 'result_rrt'
            if not os.path.exists(destination_folder):
                os.makedirs(destination_folder)
            
            file_name = 'result_' + str(obstacles_num) + "_rrt"
            save_path = os.path.join(destination_folder, file_name)
            rrt.visualize(path, save_path)
        else:
            print("No path found")
    else:
        print("Could not find non-intersecting start and end location in given iterations")
    
if __name__ == "__main__":
    main()
