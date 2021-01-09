import numpy as np

class Client:
    """
    id: Identifier
    x, y: Coordinates
    """
    def __init__(self, id, x, y):
        self.id = id
        self.x, self.y = x, y
        
    def __repr__(self):
        return f'Client {self.id} ({self.x}, {self.y})'
    
    def __eq__(self, other):
        return self.id == other.id
    
    def get_distance_from(self, point):
        """Euclidian distance from given point"""
        return np.sqrt((point.x-self.x)**2 + (point.y-self.y)**2)