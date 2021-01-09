from utils import with_timer
from client import Client
from drone import Drone
import numpy as np
import random
INF = 999999

class TabuSearch:
    
    def __init__(self, num_of_drones=3, drone_capacity=4,
                 num_of_clients=12, clients_file=None):
        self.BASE = Client(0, 0, 0)
        self.M = num_of_drones # Actual number of drones
        self.Q = drone_capacity
        self.N = num_of_clients
        
        # D - number of routes needed to deliver packages
        self.drones, self.D = self._create_drones(num_of_clients, drone_capacity)
        self.clients = self._initialize_clients(clients_file, num_of_clients)
        self.distance_matrix = self._create_distance_matrix()
        
        self.tabu_list = []
        self.best_solution = None
        self.best_candidate = None
        self.candidate_cost = 0
        self.best_cost = 0
        self._best = {}
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
                    x_pos = random.randint(-25,25)
                    y_pos = random.randint(-25,25)
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
        """
        TESTING
        Sorts route by searching permutations and finding
        the most optimal (with least cost)
        # NOTE: it may be unefficient when route consist
                of more than 6 clients
        """
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
        """
        TESTING
        Sorts solution by sorting each route
        """
        return [self.sort_route(r) for r in solution]
    
    def find_closest_client(self, client):
        """
        TESTING
        Finds closest neighbor to passed client
        """
        min_ = INF
        for c in self.clients:
            if c != self.BASE and c != client:
                dist_ = self.distance_matrix[client.id][c.id]
                if dist_ < min_:
                    client_ID = c.id
                    client = c
                    min_ = dist_
        return client

    def find_neighborhood(self):
        sol = [s[:] for s in self.best_candidate]
        nb = []
        for i in range(self.D):
            for j in range(i, self.D):
                s_copy = [s[:] for s in sol]
                f_samp = np.random.randint(1,len(sol[i])-1)
                s_samp = np.random.randint(1,len(sol[j])-1)
                s_copy[i][f_samp], s_copy[j][s_samp] = s_copy[j][s_samp], s_copy[i][f_samp]
                nb.append(s_copy)
        return nb
    
    def find_neighborhood2(self):
        """
        Generuje podaną ilość zmian pomiędzy ścieżkami  (jak dobrać ilość zmian ?)
        pierwsza scieżka zamienia randomowo sie z drugą jednym klientem druga z trzecia itd.
        ostatnia zamienia się z pierwszą na początku
        Każde nowe rozwiązanie jest zapisywane osobno i zwracane
        """
        num_of_swaps = 4
        sol = [s for s in self.best_candidate]
        nb = []
        for j in range(num_of_swaps):
            for i in range(len(sol)):
                s_copy = [s[:] for s in sol]
                f_samp = s_samp = 0
                while f_samp == s_samp:
                    f_samp = np.random.randint(1,len(sol[i-1])-1)
                    s_samp = np.random.randint(1,len(sol[i])-1)
                s_copy[i-1][f_samp], s_copy[i][s_samp] = s_copy[i][s_samp], s_copy[i-1][f_samp]
                nb.append(s_copy)
        return nb
    
    def process_solution(self):
        for i, p in enumerate(self.best_solution):
            path = []
            for x in p:
                if x == 0:
                    path.append(self.BASE)
                else:
                    path.append(self.clients[x-1])
            self._best[self.drones[i]] = path
    
    
    ### TESTING
    def initialize_solution(self, solution):
        self.best_candidate = solution
        self.best_solution = solution
        self.tabu_list.append(solution)
        self.best_cost = self._fitness(solution)
        self.candidate_cost = self.best_cost
        self.costs.append(self.best_cost)
        self.best_costs.append(self.best_cost)
    
    ### TESTING
    def __reset(self):
        self.tabu_list = []
        self.best_solution = None
        self.best_candidate = None
        self.candidate_cost = 0
        self.best_cost = 0
        self._best = {}
        self.costs = []
        self.best_costs = []
    
    
    @with_timer
    def search(self, tabu_size=50, n_iters=1000):
        """
        Główna pętla
        1. Tworzymy randomowe rozwiązanie i zapisujemy w zmiennych
        2. Iterujemy po zwróconych sąsiadach i szukamy najlepszego kandydata
        3. Jeżeli funkcja kosztu tego kandydata jest mniejsza od najlepszego znalezionego
           to zapisujemy go
        4. Dodajemy kandydata do listy tabu
        5. Jeśli lista jest wieksza niż maksymalny rozmiar to odrzuć to dodane najwcześniej (jak dobrać?)2
        """
        
        ### TESTING
        sol = self.generate_random_solution()
        self.initialize_solution(sol)  
        
        while n_iters > 0:
            nh = self.find_neighborhood()
            for nb in nh:
                if nb not in self.tabu_list:
                    self.best_candidate = nb
                    self.candidate_cost = self._fitness(self.best_candidate)
                    break
            
            for nb in nh[1:]:
                nb_cost = self._fitness(nb)
                if nb_cost < self.candidate_cost:
                    if nb not in self.tabu_list:
                        self.best_candidate = nb
                        self.candidate_cost = nb_cost
            
            if self.candidate_cost < self.best_cost:
                self.best_solution = self.best_candidate
                self.best_cost = self.candidate_cost
            
            # Add candidate to tabu and update cost history
            self.tabu_list.append(self.best_candidate)
            self.costs.append(self.candidate_cost)
            self.best_costs.append(self.best_cost)
            
            if len(self.tabu_list) > tabu_size:
                self.tabu_list.pop(0)
            
            n_iters -= 1
    
        self.process_solution()