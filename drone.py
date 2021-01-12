import numpy as np

class Drone:
    """
    id: Identifier
    x, y: Coordinates
    capacity: Capacity
    num_of_packages: Number of packages at exact moment
    temp_client_id: Assigned client's id
    x_client, y_client: Coords of assigned client
    """
    def __init__(self, id, capacity):
        self.id = id
        self.capacity = capacity
        self.num_of_packages = 0
        self.temp_client_id = None
        self.x, self.y = 0, 0
        self.x_client, self.y_client = None, None
        self.x_prev_client, self.y_prev_client = None, None
        
    def __repr__(self):
        return '(Drone {}, Capacity: {})'.format(self.id, self.capacity)

    def change_position(self, x, y):
        """Change drone position"""
        self.x, self.y = x, y
    
    def create_log(self, log):
        """Print log and save to file"""
        print(log)
        with open("logs.txt", "a") as file_object:
            file_object.write(log + "\n")

    def get_distance_from_client(self):
        """Euclidian distance from client"""
        return np.sqrt((self.x_client-self.x)**2 + (self.y_client-self.y)**2)
    
    def deliver_package(self, elapsed_time):
        """Deliver package: update number of packages
        and set appropriate client attributes"""
        self.num_of_packages -= 1
        self.x_prev_client = self.x_client
        self.y_prev_client = self.y_client
        self.create_log(f"Time: {elapsed_time} min  (Drone: {self.id} | Packages: {self.num_of_packages}) - Package delivered to client with id {self.temp_client_id}")
        if self.num_of_packages == 0:
            self.x_client = 0
            self.y_client = 0
        else:
            self.x_client = None
            self.y_client = None
        self.temp_client_id = None
        
    def load_packages(self):
        """Load packages equal to capacity"""
        self.num_of_packages = self.capacity
        self.temp_client_id = None

    def specify_client(self, client):
        """Assign passed client to drone"""
        if self.temp_client_id == None:
            self.temp_client_id = client.id
            self.x_client = client.x
            self.y_client = client.y
    
    def travel(self, elapsed_time):
        """Drone travel: each call change position by 1
        in direction to assigned client, if distance <= 1
        deliver package"""
        if self.x_client is not None and self.y_client is not None:
            distance = self.get_distance_from_client()
            if distance <= 1:
                if self.x_client == 0 and self.y_client == 0:
                    self.load_packages()
                    self.create_log(f"Time: {elapsed_time} min  (Drone: {self.id} | Packages: {self.num_of_packages}) - In base")
                    self.change_position(0, 0)
                else:
                    self.deliver_package(elapsed_time)
                    self.x = self.x_prev_client
                    self.y = self.y_prev_client
            else:
                # v = vec[x,y] * vec[x_client,y_client] = [x_client-x, y_client-y]
                # vec[x,y] = vec[x,y] + l * v
                # where l is the length of step
                v_x = self.x_client - self.x
                v_y = self.y_client - self.y
                new_x = self.x + 1/distance * v_x
                new_y = self.y + 1/distance * v_y
                self.change_position(new_x, new_y)
