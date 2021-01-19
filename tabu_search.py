from utils import with_timer
from client import Client
from drone import Drone
import numpy as np
import random
MAX_COST = 999999

class TabuSearch:
    
    def __init__(self, num_of_drones=3, drone_capacity=4,
                 num_of_clients=12, clients_file=None):
        self.BASE = Client(0, 0, 0)
        self.M = num_of_drones
        self.Q = drone_capacity
        self.N = num_of_clients
        
        # D - number of routes needed to deliver packages
        self.drones, self.D = self._create_drones(num_of_clients, drone_capacity)
        self.clients = self._initialize_clients(clients_file, num_of_clients)
        self.distance_matrix = self._create_distance_matrix()
        self.TABU = {}
        self.best_solution = None
        self.best_candidate = None
        self.candidate_cost = 0
        self.best_cost = 0
        self.processed_solution = {}
        self.costs = []
        self.best_costs = []
        
    def __repr__(self):
        return f'Tabu search for:\n\nClients: {self.clients}\n\nDrones: {self.drones}\n\nSolution: {self.solution}'
    
    @staticmethod
    def _create_drones(num_of_clients, drone_capacity):
        drones_needed = num_of_clients // drone_capacity
        if num_of_clients % drone_capacity == 0:
            drones = [Drone(i+1, drone_capacity) for i in range(drones_needed)]
        else:
            drones_needed += 1
            drones = [Drone(i+1, drone_capacity) for i in range(drones_needed)]
        return drones, drones_needed
    
    @staticmethod
    def _read_clients_from_file(file_name, num_of_clients):
        test_clients = np.loadtxt(file_name, delimiter=',', dtype=int)
        if len(test_clients) > num_of_clients:
            clients = [Client(*test_clients[i]) for i in range(num_of_clients)]
        else:
            clients = [Client(*c) for c in test_clients]
        return clients
    
    @staticmethod
    def _create_new_client_samples(file_name, num_of_clients):
        clients, busy = [], set()
        with open(file_name, 'w') as file:
            for i in range(num_of_clients):
                x_pos, y_pos = 0, 0
                while y_pos == 0 and x_pos == 0 and (x_pos, y_pos) not in busy:
                    x_pos = random.randint(-35, 35)
                    y_pos = random.randint(-35, 35)
                busy.add((x_pos, y_pos))
                file.write(f'{i+1},{x_pos},{y_pos}\n')
                clients.append(Client(i+1, x_pos, y_pos))
        return clients
    
    def _initialize_clients(self, file_name, num_of_clients):
        if file_name:
            try:
                clients = self._read_clients_from_file(file_name, num_of_clients)
            except OSError:
                return self._create_new_client_samples(file_name, num_of_clients)
            if len(clients) < self.N:
                raise Exception('Not enough samples in the file:\
                                change num_of_clients param passed to constructor.')
            return clients
        return self._create_new_client_samples("test_clients.txt", num_of_clients)
    
    def _create_distance_matrix(self):
        distance_matrix = np.zeros((self.N + 1, self.N + 1))
        for i in range(self.N + 1):
            for j in range(i, self.N + 1):
                if i != j:
                    if i == 0:
                        distance_matrix[i][j] = distance_matrix[j][i]\
                        = self.clients[j-1].get_distance_from(self.BASE)
                    else:
                        distance_matrix[i][j] = distance_matrix[j][i]\
                        = self.clients[i-1].get_distance_from(self.clients[j-1])
        return distance_matrix
    
    def find_next_drone_to_come_back(self, paths):
        lowest, idx = np.inf, 0
        for i, d in enumerate(self.drones):
            if d not in paths:
                break
            s = sum(self.distance_matrix[paths[d][i].id][paths[d][i+1].id] \
                    for i in range(len(paths[d])-1))
            if s <= lowest:
                idx = i
                lowest = s
        return idx
    
    def generate_random_solution(self):
        """Randomly generates solution in the form of arrays
        [[0, 1, 3..., 0],...[0, 9, 12..., 0], [0, 8, 4..., 0]]
        where each number represents ID of unique client
        ID of 0 - represented as Base
        Each path is different and number of paths is the same as
        number of drones necessary to deliver all the packages
        (limited capacity)"""
        paths = [[0] for _ in range(self.D)]
        samps = random.sample(self.clients, k=len(self.clients))
        for p in paths:
            if len(samps) > self.Q:
                p.extend([samps.pop().id for _ in range(self.Q)])
            else:
                p.extend([samps.pop().id for _ in range(len(samps))])
            p.append(0)
        return paths
    
    def _fitness(self, solution):
        """Cost function for given solution: 
        sum of distances for each path"""
        return sum(map(self.route_fitness, solution))
    
    def route_fitness(self, route):
        return sum(self.distance_matrix[route[i]][route[i+1]] for i in range(len(route)-1))
    
    def sort_route(self, route):
        """Sorts route by searching permutations and finding
        the most optimal (with least cost)
        # NOTE: it may be unefficient when route consist
                of more than 6 clients"""
        r = route
        best_fitness = self.route_fitness(r)
        for perm in list(itertools.permutations(r[1:-1])):
            perm = list(perm)
            perm.insert(0, route[0])
            perm.append(route[-1])
            temp_fitness = self.route_fitness(perm)
            if best_fitness > temp_fitness:
                best_fitness = temp_fitness
                r = perm
        return r
    
    def sort_solution(self, solution):
        """Sorts solution by sorting each route"""
        return [self.sort_route(r) for r in solution]
    
    def find_closest_client(self, client):
        """Finds closest neighbor to passed client"""
        min_ = MAX_COST
        for c in self.clients:
            if c != self.BASE and c != client:
                dist_ = self.distance_matrix[client.id][c.id]
                if dist_ < min_:
                    client = c
                    min_ = dist_
        return client

    def find_neighborhood(self):
        """Generate each neighbor by randomly swapping
        only one client between every two paths from best candidate"""
        solution = [path[:] for path in self.best_candidate]
        neighborhood = []
        moves = []
        for id_1 in range(self.D):
            for id_2 in range(id_1, self.D):
                s_copy = [path[:] for path in solution]
                samp_1 = np.random.randint(1,len(solution[id_1])-1)
                samp_2 = np.random.randint(1,len(solution[id_2])-1)
                s_copy[id_1][samp_1], s_copy[id_2][samp_2] = s_copy[id_2][samp_2], s_copy[id_1][samp_1]
                moves.append((id_1, samp_1, id_2, samp_2))
                neighborhood.append(s_copy)
        return neighborhood, moves
    
    def find_neighborhood2(self):
        """Generates number of changes between paths
        First path swaps one client with second path,
        second with third and so on.. last paths swaps
        with first"""
        num_of_swaps = 3
        solution = [path[:] for path in self.best_candidate]
        neighborhood = []
        moves = []
        for _ in range(num_of_swaps):
            for idx in self.D:
                s_copy = [path[:] for path in solution]
                samp_1 = samp_2 = 0
                while samp_1 == samp_2:
                    samp_1 = np.random.randint(1,len(solution[idx-1])-1)
                    samp_2 = np.random.randint(1,len(solution[idx])-1)
                s_copy[idx-1][samp_1], s_copy[idx][samp_2] = s_copy[idx][samp_2], s_copy[idx-1][samp_1]
                moves.append((id_1, samp_1, id_2, samp_2))
                neighborhood.append(s_copy)
        return neighborhood, moves

    def process_solution(self):
        """Process solution which can be later visualized"""
        for idx, path in enumerate(self.best_solution):
            temp_path = []
            for client_id in path:
                if client_id == 0:
                    temp_path.append(self.BASE)
                else:
                    temp_path.append(self.clients[client_id-1])
            self.processed_solution[self.drones[idx]] = temp_path
    
    def initialize_solution(self, solution):
        """Initialize random solution"""
        self.best_candidate = solution
        self.best_solution = solution
        self.best_cost = self._fitness(solution)
        self.candidate_cost = self.best_cost
        self.costs.append(self.best_cost)
        self.best_costs.append(self.best_cost)
        self.processed_solution = {}
    
    @with_timer
    def search(self, tabu_size=50, n_iters=1000):
        """Main search loop
        1. Create and initialize random solution
        2. Iterate over neighbors and search for best candidate,
           save it if move not in tabu list
        3. If move in tabu list check aspiration criteria
        4. If best candidate's cost function is less than best known
           solution then save it
        5. Add candidate's move to tabu list
        6. If tabu list length is > max tabu size then remove
           the first one in the list"""
        sol = self.generate_random_solution()
        self.initialize_solution(sol)  
        temp_move = None
        
        while n_iters > 0:
            nh, moves = self.find_neighborhood()
            for nb, move in zip(nh, moves):
                if move not in self.TABU:
                    self.best_candidate = nb
                    self.candidate_cost = self._fitness(self.best_candidate)
                    temp_move = move
                    break
            
            for nb, move  in zip(nh[1:], moves[1:]):
                nb_cost = self._fitness(nb)
                if nb_cost < self.candidate_cost:
                    if move not in self.TABU:
                        self.best_candidate = nb
                        self.candidate_cost = nb_cost
                        temp_move = move
                    else:
                        # Aspiration criteria
                        if nb_cost < self.best_cost:
                            self.best_candidate = nb
                            self.candidate_cost = nb_cost
                            temp_move = move
            
            if self.candidate_cost < self.best_cost:
                self.best_solution = self.best_candidate
                self.best_cost = self.candidate_cost
            
            # Add candidate to tabu and update cost history
            self.TABU[temp_move] = tabu_size
            self.costs.append(self.candidate_cost)
            self.best_costs.append(self.best_cost)
            
            # Remove moves from tabu list
            moves_to_delete = []
            for move, i in self.TABU.items():
                if i == 0:
                    moves_to_delete.append(move)
                else:
                    self.TABU[move] -= 1
                    
            for s in moves_to_delete:
                del self.TABU[s]
            
            n_iters -= 1
    
        self.process_solution()