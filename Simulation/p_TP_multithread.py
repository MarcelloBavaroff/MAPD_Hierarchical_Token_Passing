from math import fabs
import random
import numpy as np
import threading
from Simulation.CBS.p_cbs import CBS, Environment
from dijkstar import Graph, find_path  #tizi simpatici che hanno implementato dijkstra


class frontier:
    def __init__(self, front):
        self.start_partition = front[2]
        self.destination_partition = front[5]
        self.start_pos = tuple((front[0], front[1]))
        self.destination_pos = tuple((front[3], front[4]))


# noinspection PyTypeChecker
class TokenPassing(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, simulation,
                 goal_endpoints, frontiers, matrix_cells_partitions, a_star_max_iter=500):
        random.seed(1234)
        self.agents = agents
        self.dimensions = dimensions
        self.obstacles = set(obstacles)
        self.non_task_endpoints = non_task_endpoints
        self.number_of_areas = number_of_areas
        self.frontiers = self.convert_frontiers(frontiers)
        self.partitions = partitions
        self.matrix_cells_partitions = matrix_cells_partitions
        # if len(agents) > len(non_task_endpoints):
        #     print('There are more agents than non task endpoints, instance is not well-formed.')
        #     exit(1)

        self.tokens = []
        self.simulation = simulation
        self.a_star_max_iter = a_star_max_iter
        self.chiamateAstar = 0
        self.sommaEspansioniAtot = 0
        self.sommaEspansioniAmaxTimestep = 0
        self.sumOfCosts = 0
        self.maxAstar = 0
        self.update_A_lock = threading.Lock()
        self.goal_endpoints = goal_endpoints
        self.global_view = {}
        self.init_global_view()
        self.init_tokens(partitions)
        self.graph = Graph()
        self.create_graph()

        self.vec_areas = {}
        self.parallel_rounds = 0
        for i in range(self.number_of_areas):
            self.vec_areas[i] = []

        self.espansioniAstarXpart = [0] * self.number_of_areas
        self.heatmap = np.zeros((dimensions[0], dimensions[1]))
        #serve per vedere quanti thread hanno finito nel mentre che vengono eseguite altre operazioni
        # self.just_finished_threads = 0
        # self.jft_lock = threading.Lock()
        self.finish_event = threading.Event()
        self.finish_event_lock = threading.Lock()
        self.print_lock = threading.Lock()
        #vedi sotto


    def get_heatmap(self):
        return self.heatmap

    def get_parallel_rounds(self):
        return self.parallel_rounds

    def get_vec_areas(self):
        return self.vec_areas

    def get_max_Astar(self):
        return self.maxAstar

    def get_sum_of_costs(self):
        return self.sumOfCosts

    def get_number_of_areas(self):
        return self.number_of_areas

    def init_global_view(self):
        self.global_view['tasks'] = {}
        self.global_view['start_tasks_times'] = {}
        self.global_view['completed_tasks_times'] = {}

        self.global_view['pre_assignment_agents_tasks'] = {}
        self.global_view['completed_tasks'] = []
        self.global_view['agents_to_areas'] = {}
        self.global_view['occupied_non_task_endpoints'] = set()
        #dizionario con corrispondenza agent_name -> lista di zone da visitare
        self.global_view['abstract_to_loc1'] = {}
        self.global_view['abstract_to_loc2'] = {}
        self.global_view['discarded_tasks_for_agents'] = {}
        self.global_view['current_goals'] = set()

        for t in self.simulation.get_new_tasks():
            self.global_view['tasks'][t['task_name']] = [t['pickup'], t['delivery']]
            self.global_view['start_tasks_times'][t['task_name']] = self.simulation.get_time()

        for a in self.agents:
            pos = [a['start']]
            self.global_view['agents_to_areas'][a['name']] = []
            self.global_view['agents_to_areas'][a['name']].append(self.find_partition(pos[0]))
            if tuple(pos[0]) in self.non_task_endpoints:
                self.global_view['occupied_non_task_endpoints'].add(tuple(a['start']))

            self.global_view['abstract_to_loc1'][a['name']] = []
            self.global_view['abstract_to_loc2'][a['name']] = []
            self.global_view['discarded_tasks_for_agents'][a['name']] = set()

    #initialize a single token
    def init_token(self, index=0, partition=None):
        self.tokens[index]['agents'] = {}
        self.tokens[index]['path_ends'] = set()
        self.tokens[index]['partition'] = partition  #x_min, y_min, x_max, y_max
        self.tokens[index]['own_frontiers'] = {}
        self.tokens[index]['occupied_frontiers'] = {}

        # qui salvo solo le frontiere che partono dalla partizione corrente
        # per ogni destinazione ho una lista frontiere che mi ci portano
        for f in self.frontiers:
            if f.start_partition == index:
                try:
                    self.tokens[index]['own_frontiers'][f.destination_partition].append(f)
                except:
                    self.tokens[index]['own_frontiers'][f.destination_partition] = [f]

        for a in self.agents:
            if self.find_partition(a['start']) == index:
                self.tokens[index]['agents'][a['name']] = [a['start']]
                if not tuple(a['start']) in self.non_task_endpoints:
                    self.tokens[index]['path_ends'].add(tuple(a['start']))

    #initialize all tokens
    def init_tokens(self, partitions):
        for t in range(self.number_of_areas):
            self.tokens.append({})
            self.init_token(t, partitions[t])

    def convert_frontiers(self, front):
        frontiers = []
        for f in front:
            frontiers.append(frontier(f))
        return frontiers

    def create_graph(self):
        for f in self.frontiers:
            self.graph.add_edge(f.start_partition, f.destination_partition, 1)

    #restituisce l'indice della partizione in cui si trova la posizione pos (thanks co-pilot)
    def find_partition_old(self, pos):

        for i in range(self.number_of_areas):
            if self.partitions[i][0] <= pos[0] <= self.partitions[i][2] and self.partitions[i][1] <= pos[1] <= \
                    self.partitions[i][3]:
                return i
        return -1

    def find_partition(self, pos):
        return self.matrix_cells_partitions[pos[0]][pos[1]]

    #in teoria agenti in idle hanno il path verso la loro posizione attuale
    # def get_idle_agents(self):
    #     agents = {}
    #     for name, path in self.tokens[0]['agents'].items():
    #         if len(path) == 1:
    #             agents[name] = path
    #     return agents

    # devo restituire in dizionario di agenti in idle con nome come chiave e posizione come valore (lista)

    def get_start_tasks_times(self):
        return self.global_view['start_tasks_times']

    def get_idle_agents_without_preass(self):

        #itero per ogni partizione/token così da avere la posizione attuale dell'agente
        agents = {}
        for t in range(self.number_of_areas):
            for name, path in self.tokens[t]['agents'].items():
                if name not in self.global_view['pre_assignment_agents_tasks']:
                    agents[name] = path

        return agents

    def get_idle_agents_global(self):
        all_idle_agents = self.tokens[0]['agents'].copy()

        for t in range(1, self.number_of_areas):
            all_idle_agents.update(self.tokens[t]['agents'].copy())

        return all_idle_agents

    #agenti che hanno un task assegnato e per cui devo pianificare (quelli in idle non ci sono perchè non hanno un pre_ass)
    def get_agents_to_plan(self):
        agents = {}
        for t in range(self.number_of_areas):
            for name, path in self.tokens[t]['agents'].items():
                if name in self.global_view['pre_assignment_agents_tasks'] and len(path) == 1:
                    agents[name] = path

        agents_copy = agents.copy()

        #agenti in migrazione
        for name, path in agents_copy.items():
            if len(self.global_view['agents_to_areas'][name]) > 1:
                agents.pop(name)

        return agents

    #distanza in celle verticali ed orizzontali
    def admissible_heuristic(self, task_pos, agent_pos):
        return fabs(task_pos[0] - agent_pos[0]) + fabs(task_pos[1] - agent_pos[1])

    def get_closest_task_name(self, available_tasks, agent_pos):
        closest = random.choice(list(available_tasks.keys()))
        dist = self.admissible_heuristic(available_tasks[closest][0], agent_pos)
        for task_name, task in available_tasks.items():
            if self.admissible_heuristic(task[0], agent_pos) < dist:
                closest = task_name
        return closest

    #considerando già che l'agente è su una frontiera
    def is_migrating(self, agent_name):
        migrante = False
        if len(self.global_view['abstract_to_loc1'][agent_name]) > 1:
            migrante = True
        elif len(self.global_view['abstract_to_loc2'][agent_name]) > 1:
            migrante = True
        return migrante

    def get_moving_obstacles_agents(self, agents, time_start):
        obstacles = {}
        negative_obstacles = {}
        for name, path in agents.items():  #agents.item ritorna un dizionario con nome agente e coordinate dello stesso
            if len(path) > time_start and len(path) > 1:
                for i in range(time_start, len(path)):
                    k = i - time_start
                    obstacles[(path[i][0], path[i][1], k)] = name
                    #se l'ultima posizione è una frontiera metto l'ostacolo negativo a patto
                    #di non aver pianificato la migrazione (len areas == 2), se ho pianificato non metto negativo
                    if i == len(path) - 1 and (not self.is_frontier_start_pos(path[i]) or len(
                            self.global_view['agents_to_areas'][name]) == 1):
                        try:
                            negative_obstacles[-k].append((path[i][0], path[i][1]))
                        except:
                            negative_obstacles[-k] = [(path[i][0], path[i][1])]
        return obstacles, negative_obstacles

    def is_frontier_start_pos(self, pos):

        own_partition = self.find_partition(pos)
        for f in self.tokens[own_partition]['own_frontiers']:
            for front in self.tokens[own_partition]['own_frontiers'][f]:
                if front.start_pos == tuple(pos):
                    return True

        # for f in self.frontiers:
        #     if f.start_pos == tuple(pos):
        #         return True
        return False

    def abort_planning(self, agent_name, pos_to_go, part_index):

        #frontiere occupate per un tempo indefinito
        occupied_frontiers = set()
        for a in self.tokens[part_index]['occupied_frontiers']:
            # se invece l'agente ha due aree associate vuol dire che sta migrando e quindi
            # la frontiera non è occupata per un tempo indefinito
            # caso particolare: ha 2 aree e la seconda è part index perché ci sta migrando
            if a != agent_name and (len(self.global_view['agents_to_areas'][a]) == 1 or
                                    (len(self.global_view['agents_to_areas'][a]) == 2 and
                                     self.global_view['agents_to_areas'][a][1] == part_index)):
                occupied_frontiers.add(self.tokens[part_index]['occupied_frontiers'][a])

        # if agent_name in self.tokens[part_index]['occupied_frontiers']:
        # primo ciclo trova tutte le chiavi di own_frontiers, cioè le partizioni verso cui si può andare
        for f in self.tokens[part_index]['own_frontiers']:
            # frontiere verso la singola partizione
            for front in self.tokens[part_index]['own_frontiers'][f]:
                # corrisponde ad una frontiera e len areas == 1 abort planning, finirò in un A* limit
                if front.start_pos == tuple(pos_to_go):
                    if front in occupied_frontiers:
                        return True
                    else:
                        return False
        return False

    def get_idle_obstacles_agents(self, agents_paths, time_start, agent_name):

        obstacles = set()
        #for g in self.goal_endpoints:
        #   obstacles.add(tuple(g))
        for agent in agents_paths:
            if agent != agent_name:
                # quelli nelle stazioni non li segno come ostacoli and tuple(path[0]) not in charging_stations_pos
                if len(agents_paths[agent]) == 1 and len(self.global_view['agents_to_areas'][agent]) == 1:
                    obstacles.add((agents_paths[agent][0][0], agents_paths[agent][0][1]))
                # presumo agenti che finiranno il loro percorso e si fermeranno? Quindi metto ultima
                # loro posizione
                if 1 < len(agents_paths[agent]) <= time_start:
                    #idle obstacles se sto andando alla frontiera, se invece sono lì ed ho pianificato di spostarmi
                    #non lo metto
                    if (not self.is_frontier_start_pos(agents_paths[agent][-1]) or len(
                            self.global_view['agents_to_areas'][agent]) == 1):
                        obstacles.add((agents_paths[agent][-1][0], agents_paths[agent][-1][1]))

        return obstacles

    def check_safe1(self, agent_pos):
        return tuple(agent_pos) in self.non_task_endpoints

    def check_safe2(self, agent_pos):
        for task_name, task in self.global_view['tasks'].items():
            if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                return False

        return True

    #questo controllo penso sia deprecato, serviva quando avvenivano i ricalcoli post path cancellati
    def check_safe3(self, agent_pos):
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False
        return True

    def check_safe4(self, agent_pos):
        # ragioniamo:
        # probabilmente superfluo
        for f in self.frontiers:
            if f.start_pos == tuple(agent_pos) or f.destination_pos == tuple(agent_pos):
                return False
        return True

    def check_safe_idle(self, agent_pos):

        if tuple(agent_pos) in self.non_task_endpoints:
            return True

        #becca solo i delivery
        if tuple(agent_pos) in self.goal_endpoints:
            for task_name, task in self.global_view['tasks'].items():
                if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                    return False
            return True

        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False

        own_partition = self.find_partition(agent_pos)
        for f in self.tokens[own_partition]['own_frontiers']:
            for front in self.tokens[own_partition]['own_frontiers'][f]:
                if front.start_pos == tuple(agent_pos):
                    return False

        # for f in self.frontiers:
        #     if f.start_pos == tuple(agent_pos) or f.destination_pos == tuple(agent_pos):
        #         return False
        return True

    #questo l'ho ripreso dal lavoro sulle batterie, sono accettate come pos safe solo i NONte
    #ed i goal dove non mira nessun altro
    def check_safe_idle_strict(self, agent_pos):

        if tuple(agent_pos) in self.non_task_endpoints:
            return True

        if agent_pos in self.goal_endpoints:
            return self.check_safe2(agent_pos)

        return False

    def get_closest_non_task_endpoint(self, agent_pos):
        dist = -1
        res = -1
        for endpoint in self.non_task_endpoints:
            if endpoint not in self.global_view['occupied_non_task_endpoints']:
                if dist == -1:
                    dist = self.admissible_heuristic(endpoint, agent_pos)
                    res = endpoint
                else:
                    tmp = self.admissible_heuristic(endpoint, agent_pos)
                    if tmp < dist:
                        dist = tmp
                        res = endpoint
        if res == -1:
            with self.print_lock:
                print('Error in finding non-task endpoint, is instance well-formed?')
            #exit(1)
        return res


    def no_agent_passing_in_frontier(self, pos, part_index):
        for a in self.tokens[part_index]['agents']:
            if [pos[0], pos[1]] in self.tokens[part_index]['agents'][a]:
                return False
            # for p in self.tokens[part_index]['agents'][a]:
            #     if tuple(p) == tuple(pos):
            #         return False
        return True
    #restituisce una frontiera non una coordinata
    def get_closest_frontier(self, agent_pos, frontiers_to_next_part, actual_part):
        occupied_frontiers = set()
        for a in self.tokens[actual_part]['occupied_frontiers']:
            # se invece l'agente ha due aree associate vuol dire che sta migrando e quindi
            # la frontiera non è occupata per un tempo indefinito
            if len(self.global_view['agents_to_areas'][a]) == 1:
                occupied_frontiers.add(self.tokens[actual_part]['occupied_frontiers'][a])

        # nessun agente deve aver intenzione di passare su quella frontiera
        # pickup inclusi
        # all_occupied_cells = set()
        # for agent in self.tokens[actual_part]['agents']:
        #     for pos in self.tokens[actual_part]['agents'][agent]:
        #         all_occupied_cells.add(tuple(pos))

        dist = -1
        res = -1
        for f in frontiers_to_next_part:
            if f not in occupied_frontiers and self.no_agent_passing_in_frontier(f.start_pos, actual_part):
                if dist == -1:
                    dist = self.admissible_heuristic(f.start_pos, agent_pos)
                    res = f
                else:
                    tmp = self.admissible_heuristic(f.start_pos, agent_pos)
                    if tmp < dist:
                        dist = tmp
                        res = f

        if res == -1:
            with self.print_lock:
                print('*************** NO AVAILABLE FRONTIER in actual part:', actual_part, agent_pos, '****************')
            #exit(1)
        return res

    def update_ends(self, agent_pos, part_index):
        if tuple(agent_pos) in self.tokens[part_index]['path_ends']:
            self.tokens[part_index]['path_ends'].remove(tuple(agent_pos))
        #elif tuple(agent_pos) in self.tokens[part_index]['occupied_non_task_endpoints']:
        #    self.tokens[part_index]['occupied_non_task_endpoints'].remove(tuple(agent_pos))

    # def get_agents_to_tasks_goals(self):
    #     goals = set()
    #     for el in self.global_view['pre_assignment_agents_tasks'].values():
    #         goals.add(tuple(el['goal']))
    #     return goals

    #potrei aver frainteso, non dovevo mettere tutti i taks, ma sono gli agents_to_tasks
    def not_in_assigned_goals(self, pos0, pos1):
        for el in self.global_view['pre_assignment_agents_tasks'].values():
            if tuple(el['goal']) == tuple(pos0) or tuple(el['goal']) == tuple(pos1):
                return False
        return True

    def not_in_assigned_goals2(self, pos0, pos1, assigned_goals):
        return tuple(pos0) not in assigned_goals and tuple(pos1) not in assigned_goals

    def not_in_assigned_goals3(self, pos0, pos1):
        return tuple(pos1) not in self.global_view['current_goals'] and tuple(pos0) not in self.global_view['current_goals']




    def get_agents_to_tasks_starts_goals(self):
        starts_goals = set()
        for el in self.global_view['pre_assignment_agents_tasks'].values():
            starts_goals.add(tuple(el['goal']))
            starts_goals.add(tuple(el['start']))
        return starts_goals

    def get_completed_tasks(self):
        return self.global_view['completed_tasks']

    def get_completed_tasks_times(self):
        return self.global_view['completed_tasks_times']

    def get_token(self, index):
        return self.tokens[index]

    def get_global_view(self):
        return self.global_view

    #cbs single agent quindi Astar
    def search(self, cbs, part_index):

        path, espansioniA = cbs.search()
        with self.update_A_lock:
            self.chiamateAstar += 1
            self.sommaEspansioniAtot += espansioniA
            self.espansioniAstarXpart[part_index] += espansioniA
            if espansioniA == self.a_star_max_iter:
                self.maxAstar += 1

        return path

    def get_Astar_calls(self):
        return self.chiamateAstar

    def get_total_expansions(self):
        return self.sommaEspansioniAtot

    def get_exp_sum_max_per_timestep(self):
        return self.sommaEspansioniAmaxTimestep

    def collect_new_tasks(self):
        for t in self.simulation.get_new_tasks():
            self.global_view['tasks'][t['task_name']] = [t['pickup'], t['delivery']]
            self.global_view['start_tasks_times'][t['task_name']] = self.simulation.get_time()

    def update_completed_tasks(self):
        # Update completed tasks
        for agent in self.agents:
            # pos = posizione attuale agente
            agent_name = agent['name']
            pos = self.simulation.actual_paths[agent_name][-1]
            partition = self.find_partition([pos['x'], pos['y']])

            # ---------------------CASO AGENTE ARRIVATO------------------
            # se agente assegnato ad un task E le sue coordinate attuali sono = al suo goal
            # E il suo path attuale lungo 1 ed il suo taks non è safe idle
            if agent_name in self.global_view['pre_assignment_agents_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']) \
                    and len(self.tokens[partition]['agents'][agent_name]) == 1 and \
                    self.global_view['pre_assignment_agents_tasks'][agent_name][
                        'task_name'] != 'safe_idle' and self.global_view['abstract_to_loc2'][agent_name] == []:
                # il controllo con abs2 serve per evitare che un agente che ha come goal il suo stesso punto di partenza
                # e che non è riuscito a pianificare venga segnato come agente che ha completato il task

                self.global_view['completed_tasks'].append(
                    self.global_view['pre_assignment_agents_tasks'][agent_name]['task_name'])
                self.global_view['completed_tasks_times'][
                    self.global_view['pre_assignment_agents_tasks'][agent_name][
                        'task_name']] = self.simulation.get_time()
                task_to_remove = self.global_view['pre_assignment_agents_tasks'].pop(agent_name)
                self.global_view['current_goals'].remove(tuple(task_to_remove['goal']))
                self.global_view['discarded_tasks_for_agents'][agent_name] = set()

            if agent_name in self.global_view['pre_assignment_agents_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']) \
                    and len(self.tokens[partition]['agents'][agent_name]) == 1 and \
                    self.global_view['pre_assignment_agents_tasks'][agent_name][
                        'task_name'] == 'safe_idle':
                task_to_remove = self.global_view['pre_assignment_agents_tasks'].pop(agent_name)
                self.global_view['current_goals'].remove(tuple(task_to_remove['goal']))
                self.global_view['discarded_tasks_for_agents'][agent_name] = set()

    def check_path_ends(self, agent_pos, task, all_path_ends):
        return (tuple(task[0]) == tuple(agent_pos) or tuple(task[0]) not in all_path_ends) \
                and (tuple(task[1]) == tuple(agent_pos) or tuple(task[1]) not in all_path_ends)

    def check_path_ends2(self, agent_pos, task):

        #se agent pos == task[0] passo al controllo dopo
        if tuple(task[0]) != tuple(agent_pos):
            #altrimenti devo verificare che non sia in path ends di altri agenti
            for a in range(self.number_of_areas):
                if tuple(task[0]) in self.tokens[a]['path_ends']:
                    return False

        if tuple(task[1]) != tuple(agent_pos):
            for a in range(self.number_of_areas):
                if tuple(task[1]) in self.tokens[a]['path_ends']:
                    return False

        return True

    def check_path_ends3(self, agent_pos, task):

        #se agent pos == task[0] passo al controllo dopo
        if tuple(task[0]) != tuple(agent_pos):
            part = self.find_partition(task[0])
            #altrimenti devo verificare che non sia in path ends di altri agenti
            if tuple(task[0]) in self.tokens[part]['path_ends']:
                return False

        if tuple(task[1]) != tuple(agent_pos):
            part = self.find_partition(task[1])
            # altrimenti devo verificare che non sia in path ends di altri agenti
            if tuple(task[1]) in self.tokens[part]['path_ends']:
                return False

        return True


    def create_all_path_ends(self):
        all_path_ends = set()
        for a in range(self.number_of_areas):
            for tup in self.tokens[a]['path_ends']:
                all_path_ends.add(tuple(tup))
        return all_path_ends

    def create_assigned_goals(self):
        assigned_goals = set()
        for el in self.global_view['pre_assignment_agents_tasks'].values():
            assigned_goals.add(tuple(el['goal']))
        return assigned_goals
    #qui di base controlla che nessun agente abbia come path ends pickup o delivery ed
    # inoltre
    def find_available_tasks(self, agent_pos, agent_name):
        #part_index = self.find_partition(agent_pos)
        #all_path_ends = self.create_all_path_ends()
        #assigned_goals = self.create_assigned_goals()

        available_tasks = {}
        for task_name, task in self.global_view['tasks'].items():
            # se inizio e fine task non in path ends degli agenti (meno me) AND nemmeno in goals
            #meglio cercare prima in assigned goals
            if self.not_in_assigned_goals3(task[0], task[1]) and self.check_path_ends3(agent_pos, task):
                #     and tuple(task[0]) not in self.get_agents_to_tasks_goals() and tuple(
                # task[1]) not in self.get_agents_to_tasks_goals():

                #se da errore di chiave vuol dire che l'agente non ha task discarded

                if task_name not in self.global_view['discarded_tasks_for_agents'][agent_name]:
                    available_tasks[task_name] = task

        return available_tasks

    # qui metto nel token global l'assegnamento agente task
    def choose_task(self, agent_name, agent_pos, available_tasks):  # , all_idle_agents):
        closest_task_name = self.get_closest_task_name(available_tasks, agent_pos)
        closest_task = available_tasks.pop(closest_task_name)
        self.global_view['tasks'].pop(closest_task_name)
        pickup = closest_task[0]
        delivery = closest_task[1]
        self.global_view['pre_assignment_agents_tasks'][agent_name] = {'task_name': closest_task_name, 'start': pickup,
                                                                       'goal': delivery}
        self.global_view['current_goals'].add(tuple(delivery))

        # return self.compute_real_path(agent_name, agent_pos, closest_task, closest_task_name, all_idle_agents,
        #                               available_tasks)

    def choose_non_task_endpoint(self, agent_name, agent_pos):  #, all_idle_agents):
        closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos)
        if closest_non_task_endpoint != -1:
            self.global_view['pre_assignment_agents_tasks'][agent_name] = {'task_name': "safe_idle", 'start': agent_pos,
                                                                           'goal': closest_non_task_endpoint}
            self.global_view['current_goals'].add(tuple(closest_non_task_endpoint))
            self.global_view['occupied_non_task_endpoints'].add(tuple(closest_non_task_endpoint))


    def compute_real_path_double(self, agent_name, agent_pos, loc1, loc2, all_idle_agents, part_index, time_start=0):
        if self.abort_planning(agent_name, loc1, part_index) or self.abort_planning(agent_name, loc2, part_index):
            return False

        moving_obstacles_agents, negative_moving_obstacles = self.get_moving_obstacles_agents(self.tokens[part_index]['agents'], time_start)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)
        idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(loc1)})
        idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(loc1)})
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': loc1}
        env = Environment(self.tokens[part_index]['partition'], [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, negative_moving_obstacles, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path1 = self.search(cbs, part_index)
        if not path1:
            with self.print_lock:
                print("Solution not found to loc1 for agent", agent_name, " idling at current position...")
            return False
        else:
            #print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            cost1 = env.compute_solution_cost(path1)

            moving_obstacles_agents, negative_moving_obstacles = self.get_moving_obstacles_agents(self.tokens[part_index]['agents'],
                                                                       time_start + cost1 - 1)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start + cost1 - 1, agent_name)
            idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(loc1), tuple(loc2)})
            idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(loc1), tuple(loc2)})
            idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

            agent = {'name': agent_name, 'start': loc1, 'goal': loc2}
            env = Environment(self.tokens[part_index]['partition'], [agent], self.obstacles | idle_obstacles_agents,
                              moving_obstacles_agents, negative_moving_obstacles, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path2 = self.search(cbs, part_index)
            if not path2:
                with self.print_lock:
                    print("Solution not found to task goal for agent", agent_name, " idling at current position...")
                return False
            else:
                with self.print_lock:
                    print("Solution found to task start for agent", agent_name, " doing task...")
                # serve per mettere nel nuovo token l'agente che migra e solo dal timestep dopo iniziare il percorso
                for i in range(time_start):
                    path1[agent_name].insert(0, path1[agent_name][0])
                self.apply_path(agent_name, agent_pos, path1[agent_name], path2[agent_name], part_index)
                return True

    def compute_real_path_single(self, agent_name, agent_pos, goal_position, all_idle_agents, part_index, time_start=0, inside_other_functions=True):

        if self.abort_planning(agent_name, goal_position, part_index):
            return False

        moving_obstacles_agents, negative_moving_obstacles = self.get_moving_obstacles_agents(self.tokens[part_index]['agents'], time_start)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)
        idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(goal_position)})
        idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(goal_position)})
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': goal_position}
        env = Environment(self.tokens[part_index]['partition'], [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, negative_moving_obstacles, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path = self.search(cbs, part_index)

        if not path:
            with self.print_lock:
                print("Solution not found to task goal for agent", agent_name, " idling at current position...")
            outcome = False
        else:
            with self.print_lock:
                print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            #serve per mettere nel nuovo token l'agente che migra e solo dal timestep dopo iniziare il percorso
            #for i in range(time_start):
            #    path[agent_name].insert(0, path[agent_name][0])

            self.apply_path(agent_name, agent_pos, None, path[agent_name], part_index)
            outcome = True

        if not inside_other_functions:
            self.finish_event.set()
        return outcome

    # se ho solo un path passo solo il secondo
    def apply_path(self, agent_name, agent_pos, path1, path2, part_index):
        last_step = path2[-1]
        agent_part = self.find_partition(agent_pos)
        self.update_ends(agent_pos, agent_part)

        self.tokens[part_index]['agents'][agent_name] = []

        self.tokens[part_index]['path_ends'].add(tuple([last_step['x'], last_step['y']]))
        if path1 is not None:
            #self.tokens[part_index]['path_ends'].add(tuple([last_step['x'], last_step['y']]))
            for el in path1:
                self.tokens[part_index]['agents'][agent_name].append([el['x'], el['y']])
            # Don't repeat twice same step, elimino ultimo elemento
            self.tokens[part_index]['agents'][agent_name] = self.tokens[part_index]['agents'][agent_name][:-1]

        for el in path2:
            self.tokens[part_index]['agents'][agent_name].append([el['x'], el['y']])

        self.sumOfCosts += len(self.tokens[part_index]['agents'][agent_name])

        #update matrix
        for t in self.tokens[part_index]['agents'][agent_name]:
            self.heatmap[t[0], t[1]] += 1



    # assegnamento dei task agli agenti, senza tener conto del percorso
    def assign_tasks(self):
        idle_agents = self.get_idle_agents_without_preass()

        while len(idle_agents) > 0:
            agent_name = random.choice(list(idle_agents.keys()))
            all_idle_agents = self.get_idle_agents_global()
            all_idle_agents.pop(agent_name)
            agent_pos = idle_agents.pop(agent_name)[0]
            available_tasks = self.find_available_tasks(agent_pos, agent_name)
            #se ho abs1, ma non ho un percorso nel token vuol dire che non sono riuscito a trovarlo
            #per qualche ragione, quindi reinserisco il vecchio tasks e lo rimuovo da pre_assignment
            #PENSO CHE AVENDO MODIFICATO GET IDLE AGENTS QUI NON CI ARRIVI MAI
            # if len(self.global_view['abstract_to_loc1'][agent_name]) > 0:
            #     self.global_view['tasks'][self.global_view['pre_assignment_agents_tasks'][agent_name]['task_name']] = \
            #         [self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
            #          self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']]
            #     self.global_view['pre_assignment_agents_tasks'].pop(agent_name)
            #     self.global_view['abstract_to_loc1'][agent_name] = []
            #     self.global_view['abstract_to_loc2'][agent_name] = []

            if len(available_tasks) > 0:
                self.choose_task(agent_name, agent_pos, available_tasks)

            elif self.check_safe_idle(agent_pos):
                a = 0
                #print('No available tasks for agent', agent_name, ' idling at current position...')

            else:
                self.choose_non_task_endpoint(agent_name, agent_pos)
                #self.go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents)

    def compute_abstract_path(self, start, goal, agent_name, loc):

        start_partition = self.find_partition(start)
        goal_partition = self.find_partition(goal)

        path = find_path(self.graph, start_partition, goal_partition)
        if loc == 1:
            self.global_view['abstract_to_loc1'][agent_name] = path.nodes
            # caso per quando vai verso il non task endpoint e quindi hai solo abs2
            if len(path.nodes) == 1 and start == goal:
                self.global_view['abstract_to_loc1'][agent_name] = []

        else:
            self.global_view['abstract_to_loc2'][agent_name] = path.nodes

    def go_to_frontier(self, agent_name, agent_pos, all_idle_agents, actual_part, next_part):
        #scelgo la frontiera più vicina in base all'area in cui sono e a dove voglio andare
        frontiers_to_next_part = self.tokens[actual_part]['own_frontiers'][next_part]
        #discarded_frontiers = []
        closest_frontier = self.get_closest_frontier(agent_pos, frontiers_to_next_part, actual_part)

        if closest_frontier != -1:
            #do per scontato che trovo sempre un path
            self.compute_real_path_single(agent_name, agent_pos, closest_frontier.start_pos, all_idle_agents,
                                          actual_part)
            self.tokens[actual_part]['occupied_frontiers'][agent_name] = closest_frontier

        # se non ho trovato una frontiera dove andare (len path = 1) cambio task
        # se abs1 ha almeno len 1 allora devo fare il pickup
        # se len(abs1) == 0 e sono sul pickup posso ancora cancellare il task
        elif len(self.tokens[actual_part]['agents'][agent_name]) == 1 and \
                (len(self.global_view['abstract_to_loc1'][agent_name]) > 0 or agent_pos ==
                 self.global_view['pre_assignment_agents_tasks'][agent_name]['start']):
            self.remove_task_from_agents(agent_name, [actual_part])

        self.finish_event.set()

    def find_next_goal(self, agent_name, agent_pos, next_part, num_abs):
        abstract = "abstract_to_loc" + str(num_abs)

        if len(self.global_view[abstract][agent_name]) > 2:
            #devo andare alla prossima frontiera
            frontiers_to_next_part = self.tokens[next_part]['own_frontiers'][self.global_view[abstract][agent_name][2]]
            closest_frontier = self.get_closest_frontier(agent_pos, frontiers_to_next_part,
                                                         self.global_view[abstract][agent_name][1])
            #matti nel dizionario il pair agente frontiera
            if closest_frontier != -1:
                self.tokens[next_part]['occupied_frontiers'][agent_name] = closest_frontier
                return closest_frontier.start_pos
            else:
                #print('*************** NO AVAILABLE FRONTIER in actual part:', next_part, agent_pos, '****************')
                return -1

        elif len(self.global_view[abstract][agent_name]) == 2:
            return self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']

    def migrazione(self, agent_name, agent_pos, actual_part, next_part, num_abs, closest_frontier):
        #closest_frontier = self.tokens[actual_part]['occupied_frontiers'][agent_name]
        #next goal sarebbe da spostare
        all_idle_agents = self.tokens[next_part]['agents'].copy()

        pic_in_part = (num_abs == 1 and len(self.global_view['abstract_to_loc1'][agent_name]) == 2)

        # se sto migrando, devo ancora fare il pickup e questo è nella partizione successiva
        if pic_in_part:
            #valid_path = self.pickup_in_partition(agent_name, closest_frontier.destination_pos, self.global_view['pre_assignment_agents_tasks'][agent_name]['start'], all_idle_agents, next_part, time_start=1)
            valid_path = self.pickup_in_partition(agent_name, agent_pos,
                                                  self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                                  all_idle_agents, next_part, time_start=0, inside_migration=True)
        # altrimenti o devo andare da una frontiera all'altra o al delivery
        else:
            next_goal = self.find_next_goal(agent_name, closest_frontier.destination_pos, next_part, num_abs)
            #valid_path = self.compute_real_path_single(agent_name, closest_frontier.destination_pos, next_goal, all_idle_agents, next_part, time_start=0) #perché time_start = 0?
            if next_goal == -1:
                valid_path = False
            else:
                valid_path = self.compute_real_path_single(agent_name, agent_pos, next_goal, all_idle_agents, next_part,
                                                           time_start=0)

        if valid_path:
            with self.print_lock:
                print('Agent', agent_name, 'migrating to partition', next_part, '...')
            #se serve rimanere il wait nella posizione di frontiera
            num_wait = self.tokens[next_part]['agents'][agent_name].count(agent_pos)
            if num_wait > 1:
                self.tokens[actual_part]['agents'][agent_name] = []
                for i in range(num_wait):
                    self.tokens[actual_part]['agents'][agent_name].append([agent_pos[0], agent_pos[1]])
                self.global_view['agents_to_areas'][agent_name].append(next_part)

                #self.delete_conflicting_paths_more_strict(agent_name, actual_part)
                #agents_to_plan = self.get_agents_to_plan()

            else:
                #self.global_view['agents_to_areas'][agent_name] = []
                self.global_view['agents_to_areas'][agent_name].append(next_part)
                #self.tokens[actual_part]['agents'].pop(agent_name)
                self.update_ends(agent_pos, actual_part)  # apply path aggiorna solo dell'area dopo

        else:
            with self.print_lock:
                print('NO PATH DOPO MIGRAZIONE', agent_name, ' idling at current position...')
            #segnali che l'agente rimarrà fermo in attesa di riprovare
            self.tokens[actual_part]['agents'][agent_name].append([agent_pos[0], agent_pos[1]])
            #self.delete_conflicting_paths_more_strict(agent_name, actual_part)
            #se sono sulla casella di pickup posso considerare come se non lo avessi fatto
            #if not pic_in_part or (agent_name in self.global_view['pre_assignment_agents_tasks'] and agent_pos == self.global_view['pre_assignment_agents_tasks'][agent_name]['start']):
            if agent_name in self.global_view['pre_assignment_agents_tasks'] and (
                    num_abs == 1 or agent_pos == self.global_view['pre_assignment_agents_tasks'][agent_name]['start']):
                self.remove_task_from_agents(agent_name, [actual_part, next_part])
            #agents_to_plan = self.get_agents_to_plan()
        self.finish_event.set()
        #return agents_to_plan

    def on_a_frontier_old(self, agent_pos, actual_part):
        frontiers = self.tokens[actual_part]['own_frontiers']
        for part, front in frontiers.items():
            for f in front:
                if tuple(agent_pos) == f.start_pos:
                    return part

        return -1

    #restituisce la partizione di destinazione
    def on_a_frontier(self, agent_name, agent_pos, actual_part):

        if agent_name in self.tokens[actual_part]['occupied_frontiers']:
            f = self.tokens[actual_part]['occupied_frontiers'][agent_name]
            if tuple(agent_pos) == f.start_pos:
                return f.destination_partition

        return -1



    def remove_task_from_agents(self, agent_name, part_to_remove):
        print('Rimozione del task dall\'', agent_name)
        task_name = self.global_view['pre_assignment_agents_tasks'][agent_name]['task_name']
        if task_name != 'safe_idle':
            self.global_view['discarded_tasks_for_agents'][agent_name].add(task_name)
            self.global_view['tasks'][task_name] = \
                [self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                 self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']]
        elif tuple(self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']) in self.global_view[
            'occupied_non_task_endpoints']:
            self.global_view['occupied_non_task_endpoints'].remove(
                tuple(self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']))

        task_to_remove = self.global_view['pre_assignment_agents_tasks'].pop(agent_name)
        self.global_view['current_goals'].remove(tuple(task_to_remove['goal']))

        self.global_view['abstract_to_loc1'][agent_name] = []
        self.global_view['abstract_to_loc2'][agent_name] = []

        for part in part_to_remove:
            if agent_name in self.tokens[part]['occupied_frontiers']:
                self.tokens[part]['occupied_frontiers'].pop(agent_name)

    def pickup_in_partition(self, agent_name, agent_pos, pickup_position, all_idle_agents, part_index, time_start=0, inside_migration=False):
        #quì io sto facendo il pickup quindi il mio abs1 ha solo una partizione che è quella attuale
        #len abs2 = 1 vuol dire che il delivery è quì
        #len abs2 > 1 vuol dire che la prossima destinazione sarà una frontiera
        #in abs2[0] c'è sempre la partizione corrente, quindi devo vedere abs2[1] per la prossima partizione
        planned = False
        if len(self.global_view['abstract_to_loc2'][agent_name]) == 1:
            loc2 = self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']
            planned = self.compute_real_path_double(agent_name, agent_pos, pickup_position, loc2, all_idle_agents,
                                                    part_index, time_start)
        else:
            next_part = self.global_view['abstract_to_loc2'][agent_name][1]
            frontiers_to_next_part = self.tokens[part_index]['own_frontiers'][next_part]
            closest_frontier = self.get_closest_frontier(pickup_position, frontiers_to_next_part, part_index)
            if closest_frontier != -1:
                # do per scontato che trovo sempre un path
                planned = self.compute_real_path_double(agent_name, agent_pos, pickup_position,
                                                        closest_frontier.start_pos,
                                                        all_idle_agents, part_index, time_start)
                self.tokens[part_index]['occupied_frontiers'][agent_name] = closest_frontier

        #se pickup in partition viene chiamato in maniera diretta cancello da quì
        #se viene chiamato da migrazione non lo cancello
        if not planned and self.find_partition(agent_pos) == part_index:
            self.remove_task_from_agents(agent_name, [part_index])

        if not inside_migration:
            self.finish_event.set()

        return planned

    def update_A_star_stats(self):
        self.sommaEspansioniAmaxTimestep += max(self.espansioniAstarXpart)

        if sum(self.espansioniAstarXpart) != max(self.espansioniAstarXpart):
            self.parallel_rounds += 1

        for i in range(self.number_of_areas):
            self.vec_areas[i].append(self.espansioniAstarXpart[i])

        self.espansioniAstarXpart = [0] * self.number_of_areas

    def verifica_nonte(self):
        count = 0
        for part in range(self.number_of_areas):
            for agent in self.tokens[part]['agents']:
                if tuple(self.tokens[part]['agents'][agent][0]) in self.non_task_endpoints:
                    count += 1

        if count != len(self.global_view['occupied_non_task_endpoints']):
            print('ERRORE NON TASK ENDPOINTS')
            exit(1)

    def clean_threads(self, executive_threads):
        # rimozione thread inattivi
        finished_threads = [key for key, thread in executive_threads.items() if not thread.is_alive()]
        for key in finished_threads:
            del executive_threads[key]

    def check_available_token(self, agent_name, part_index, waiting_agents, executive_threads):

        #rimozione thread inattivi
        self.clean_threads(executive_threads)
        if part_index not in executive_threads.keys():
            return True
        else:
            try:
                waiting_agents[part_index].append(agent_name)
            except KeyError:
                waiting_agents[part_index] = [agent_name]
            return False

    def path_selection(self, agent_name, agent_pos, waiting_agents, executive_threads):
        agent_partition = self.global_view['agents_to_areas'][agent_name][0]

        local_idle_agents = self.tokens[agent_partition]['agents'].copy()
        local_idle_agents.pop(agent_name)

        # se non ha abstract path(s) calcolo
        if len(self.global_view['abstract_to_loc1'][agent_name]) == 0 and \
                len(self.global_view['abstract_to_loc2'][agent_name]) == 0:
            self.compute_abstract_path(agent_pos,
                                       self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                       agent_name, 1)
            self.compute_abstract_path(self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                       self.global_view['pre_assignment_agents_tasks'][agent_name]['goal'],
                                       agent_name, 2)

        # -----------------------------PATH REALI--------------------------------
        if len(self.global_view['abstract_to_loc1'][agent_name]) > 1:
            on_frontier = self.on_a_frontier(agent_name, agent_pos, agent_partition)
            # contiene la partizione di destinazione
            if on_frontier != -1:
                closest_frontier = self.tokens[agent_partition]['occupied_frontiers'][agent_name]
                if self.check_available_token(agent_name, on_frontier, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.migrazione, args=(agent_name, agent_pos, agent_partition, on_frontier, 1, closest_frontier))
                    executive_threads[on_frontier] = th
                    th.start()
            else:
                if self.check_available_token(agent_name, agent_partition, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.go_to_frontier, args=(agent_name, agent_pos, local_idle_agents, agent_partition,
                                                                          self.global_view['abstract_to_loc1'][agent_name][1]))
                    executive_threads[agent_partition] = th
                    th.start()


        # il pickup è nell'area in cui mi trovo
        # o all'inizio o appena mi viene assegnato un nuovo task
        elif len(self.global_view['abstract_to_loc1'][agent_name]) == 1:
            if self.check_available_token(agent_name, agent_partition, waiting_agents, executive_threads):
                th = threading.Thread(target=self.pickup_in_partition, args=(agent_name, agent_pos,
                                                                             self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                                                             local_idle_agents, agent_partition))
                executive_threads[agent_partition] = th
                th.start()

        # da qui in giù abstract path 1 è vuoto quindi devo andare al delivery o al non task endpoint
        elif len(self.global_view['abstract_to_loc2'][agent_name]) > 1:
            on_frontier = self.on_a_frontier(agent_name, agent_pos, agent_partition)
            #contiene la partizione di destinazione
            if on_frontier != -1:
                closest_frontier = self.tokens[agent_partition]['occupied_frontiers'][agent_name]
                if self.check_available_token(agent_name, on_frontier, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.migrazione, args=(agent_name, agent_pos, agent_partition, on_frontier, 2, closest_frontier))
                    executive_threads[on_frontier] = th
                    th.start()
            else:
                if self.check_available_token(agent_name, agent_partition, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.go_to_frontier, args=(agent_name, agent_pos, local_idle_agents, agent_partition,
                                                                          self.global_view['abstract_to_loc2'][agent_name][1]))
                    executive_threads[agent_partition] = th
                    th.start()


        elif len(self.global_view['abstract_to_loc2'][agent_name]) == 1:
            if self.check_available_token(agent_name, agent_partition, waiting_agents, executive_threads):
                th = threading.Thread(target=self.compute_real_path_single, args=(agent_name, agent_pos,
                                                                                 self.global_view['pre_assignment_agents_tasks'][agent_name]['goal'],
                                                                                 local_idle_agents, agent_partition, 0, False))
                executive_threads[agent_partition] = th
                th.start()

        else:
            print("Entrambi gli abstact path sono vuoti, errore? " + agent_name)

    def handle_waiting_agents(self, waiting_agents, executive_threads):

        copy_waiting_agents = waiting_agents.copy()
        while len(waiting_agents) > 0:
            self.clean_threads(executive_threads)
            for part in copy_waiting_agents:
                if part not in executive_threads.keys() and part in waiting_agents.keys():
                    agent_name = waiting_agents[part].pop(0)
                    if len(waiting_agents[part]) == 0:
                        del waiting_agents[part]

                    self.path_selection(agent_name, self.tokens[self.global_view['agents_to_areas'][agent_name][0]]['agents'][agent_name][0], waiting_agents, executive_threads)

            with self.finish_event_lock:
                self.finish_event.wait()
                self.finish_event.clear()

    def time_forward(self):
        self.update_completed_tasks()
        self.collect_new_tasks()
        #self.verifica_nonte()
        self.assign_tasks()

        agents_to_plan = self.get_agents_to_plan()
        priority_agents = {}
        #dict of partitions, for each one with a list of waiting agents
        waiting_agents = {}
        #dict of partitions, for each one with the executing thread
        executive_threads = {}
        copy_atp = agents_to_plan.copy()
        for agent in copy_atp:
            if agent in self.tokens[self.global_view['agents_to_areas'][agent][0]]['occupied_frontiers']:
                priority_agents[agent] = agents_to_plan[agent]
                agents_to_plan.pop(agent)

        #agents_to_REplan = {}
        while len(agents_to_plan) > 0 or len(priority_agents) > 0:

            if len(priority_agents) > 0:
                agent_name = random.choice(list(priority_agents.keys()))
                agent_pos = priority_agents.pop(agent_name)[0]
            else:
                agent_name = random.choice(list(agents_to_plan.keys()))
                agent_pos = agents_to_plan.pop(agent_name)[0]

            self.path_selection(agent_name, agent_pos, waiting_agents, executive_threads)
            #ricordati che serve un buffer per le wait sulle frontiere

        # qui metterei una nuova funzione che si occupa di gestire gli agenti in attesa
        self.handle_waiting_agents(waiting_agents, executive_threads)

        for t in executive_threads:
            executive_threads[t].join()

        #self.update_non_task_endpoints()
        if 'safe_idle' in self.global_view['tasks']:
            self.global_view['tasks'].pop('safe_idle')

        self.update_A_star_stats()
