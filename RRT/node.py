class Node:
    """
    Represents a node in the RRT tree.
    
    Attributes:
        x (float): The x-coordinate of the node.
        y (float): The y-coordinate of the node.
        parent (Node): A reference to the parent node in the RRT tree. `None` for the root node.
    """
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None