import numpy as np

class Drone:
    """
    id: Identifier
    x, y: Coordinates
    capicity: Capicity
    num_of_packages: Number of packages at exact moment
    temp_client_id: Assigned client's id
    x_client, y_client: Coords of assigned client
    """

    def __init__(self, id, capicity):
        self.id = id
        self.capicity = capicity
        self.num_of_packages = capicity
        self.temp_client_id = None
        self.x, self.y = 0, 0
        self.x_client, self.y_client = None, None
        self.x_prev_client, self.y_prev_client = None, None
        
    def __repr__(self):
        return '(Drone {}, Capicity: {})'.format(self.id, self.capicity)
    
    def change_position(self, x, y):
        """Change drone position"""
        self.x, self.y = x, y
    
    def get_distance_from_client(self):
        """Euclidian distance from client"""
        return np.sqrt((self.x_client-self.x)**2 + (self.y_client-self.y)**2)
    
    def deliver_package(self):
        """Deliver package: update number of packages
        and set appropriate client attributes"""
        self.num_of_packages -= 1
        self.x_prev_client = self.x_client
        self.y_prev_client = self.y_client
        if self.num_of_packages == 0:
            self.temp_client_id = None
            self.x_client = 0
            self.y_client = 0
        else:
            self.temp_client_id = None
            self.x_client = None
            self.y_client = None

    def load_packages(self):
        """Load packages equal to capicity"""
        self.num_of_packages = self.capicity
    
    def specify_client(self, client):
        """Assign passed client to drone"""
        if self.temp_client_id == None:
            self.temp_client_id = client.id
            self.x_client = client.x
            self.y_client = client.y
    
    def travel(self):
        """Drone travel: each call change position by 1
        in direction to assigned client, if distance <= 1
        deliver package"""
        if self.x_client is not None and self.y_client is not None:
            distance = self.get_distance_from_client()
            if (distance <= 1):
                if self.x_client == 0 and self.y_client == 0:
                    self.load_packages()
                self.x = self.x_client
                self.y = self.y_client
                self.deliver_package()
            else:
                w = abs(self.x - self.x_client)
                h = abs(self.y - self.y_client)
                sin_alpha = h/distance
                sin_beta = w/distance
                if (self.x >= self.x_client and self.y >= self.y_client):
                    self.change_position(self.x - sin_beta, self.y - sin_alpha)
                if (self.x <= self.x_client and self.y <= self.y_client):
                    self.change_position(self.x + sin_beta, self.y + sin_alpha)
                if (self.x >= self.x_client and self.y <= self.y_client):
                    self.change_position(self.x - sin_beta, self.y + sin_alpha)
                if (self.x <= self.x_client and self.y >= self.y_client):
                    self.change_position(self.x + sin_beta, self.y - sin_alpha)
