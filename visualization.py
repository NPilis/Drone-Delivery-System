from matplotlib import pyplot as plt

class WithVisualization:
    """
    drones: All available drones
    clients: Not visited clients
    x_visited, y_visited: Visited clients coords
    total_time: Total time passed since running visualization
    """
    def __init__(self, obj):
        self.drones = obj.drones
        self.clients = obj.clients
        self.solution = obj.processed_solution if obj.processed_solution else obj.generate_first_solution()
        self.total_time = 0
        self.threshold = 2000
        
        self.x_clients, self.y_clients = self._initialize_client_positions(obj.clients)
        self.x_visited, self.y_visited = [], []
        self.x_drones, self.y_drones = [], []
    
    @staticmethod
    def _initialize_client_positions(clients):
        x_clients, y_clients = [], []
        for c in clients:
            x_clients.append(c.x)
            y_clients.append(c.y)
        return x_clients, y_clients
    
    def update_drone_positions(self):
        """Drone path update"""
        for drone in self.drones:
            self.x_drones.append(drone.x)
            self.y_drones.append(drone.y)
    
    def update_visited_clients(self, x, y):
        """Updating visited clients"""
        self.x_visited.append(x)
        self.y_visited.append(y)
    
    def assign_clients(self):
        """Assign all clients to drones"""
        for drone in self.drones:
            if self.solution[drone]:
                drone.specify_client(self.solution[drone].pop(0))
    
    def assign_client(self, drone_id):
        """Assign one client to specified drone with passed ID"""
        for drone in self.drones:
            if drone.id == drone_id:
                if self.solution[drone]:
                    drone.specify_client(self.solution[drone].pop(0))
                    return True
                else:
                    return False
    
    def plot_figure(self, sizes=(12,12)):
        fig = plt.figure(figsize=sizes)

    def visualize_solution(self):
        """Run real time visualization"""
        fig, ax = plt.subplots(figsize=(15, 15))
        k, elapsed_time, D = 1, 0, len(self.drones)
        delivered = False
        while not delivered:
            delivered = True
            k = 1 if k == 4 else k + 1
            for drone in self.drones:
                if drone.temp_client_id == None:
                    is_assigned = self.assign_client(drone.id)
                    if is_assigned or drone.x != 0 or drone.y != 0:
                        delivered = False
                    self.update_visited_clients(drone.x_prev_client, drone.y_prev_client)
                else:
                    drone.travel(elapsed_time)
                    delivered = False
            elapsed_time += 1
            self.update_drone_positions()
            ax.plot(0, 0, 'bo', markersize=14, label="Base")
            ax.plot(self.x_clients, self.y_clients, 'go', markersize=12, label="Receiver")
            ax.plot(self.x_drones[-D:], self.y_drones[-D:], 'm{}'.format(k), markersize=24, markeredgewidth=3, label="Drone")
            ax.plot(self.x_visited, self.y_visited, 'ro', markersize=12, label="Package delivered")
            for s in range(D):
                ax.plot(self.x_drones[s::D], self.y_drones[s::D], linewidth=3, label=f"(Drone: {s+1}, Packages: {self.drones[s].num_of_packages})")
            ax.set_title(f'Actual time since start in minutes: {elapsed_time}')
            ax.set_ylim(-40, 40)
            ax.set_xlim(-40, 40)
            ax.legend()
            ax.grid()
            fig.canvas.draw()
            renderer = fig.canvas.renderer
            ax.draw(renderer) 
            plt.pause(0.01)
            ax.cla()
        ax.plot(0, 0, 'bo', markersize=14, label="Base")
        ax.plot(self.x_clients, self.y_clients, 'go', markersize=12, label="Receiver")
        self.update_drone_positions()
        for s in range(D):
            ax.plot(self.x_drones[s::D], self.y_drones[s::D], linewidth=3, label=f"(Drone: {s+1}, Packages: {self.drones[s].num_of_packages})")
        ax.set_title(f'Actual time since start in minutes: {elapsed_time}')
        ax.set_ylim(-40, 40)
        ax.set_xlim(-40, 40)
        ax.legend()
        ax.grid()
        plt.show()
    
    def plot_solution(self):
        """Plot final solution"""
        for d in self.drones:
            if d in self.solution:
                x_s, y_s = zip(*[(point.x, point.y) for point in self.solution[d]]) 
                plt.plot(x_s, y_s, label=f'{d}')
        plt.plot(self.x_clients, self.y_clients, 'ro', markersize=12, label="Receiver")
        plt.plot(0, 0, 'bo', markersize=12, label="Base")
        plt.grid()
        plt.legend()
        plt.ylim(-40, 40)
        plt.xlim(-40, 40)
        plt.show()