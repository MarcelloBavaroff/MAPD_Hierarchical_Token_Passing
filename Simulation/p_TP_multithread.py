from math import fabs
import random
import numpy as np
import threading
from Simulation.CBS.p_cbs import CBS, Environment
from dijkstar import Graph, find_path  #tizi simpatici che hanno implementato dijkstra


class bridge:
    def __init__(self, br):
        self.start_segment = br[2]
        self.destination_segment = br[5]
        self.entry_cell = tuple((br[0], br[1]))
        self.exit_cell = tuple((br[3], br[4]))


class TokenPassing(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, number_of_segments, segments, simulation,
                 goal_endpoints, bridges, matrix_cells_partitions, a_star_max_iter=500):
        random.seed(1234)
        self.agents = agents
        self.dimensions = dimensions
        self.obstacles = set(obstacles)
        self.non_task_endpoints = non_task_endpoints
        self.number_of_segments = number_of_segments
        self.bridges = self.convert_bridges(bridges)
        self.segments = segments
        self.matrix_cells_segments = matrix_cells_partitions
        # if len(agents) > len(non_task_endpoints):
        #     print('There are more agents than non task endpoints, instance is not well-formed.')
        #     exit(1)

        self.local_tokens = []
        self.simulation = simulation
        self.a_star_max_iter = a_star_max_iter
        self.chiamateAstar = 0
        self.sommaEspansioniAtot = 0
        self.sommaEspansioniAmaxTimestep = 0
        self.sumOfCosts = 0
        self.maxAstar = 0
        self.update_A_lock = threading.Lock()
        self.goal_endpoints = goal_endpoints
        self.global_token = {}
        self.init_global_token()
        self.init_local_tokens(segments)
        self.graph = Graph()
        self.create_graph()

        self.vec_areas = {}
        self.parallel_rounds = 0
        for i in range(self.number_of_segments):
            self.vec_areas[i] = []

        self.espansioniAstarXpart = [0] * self.number_of_segments
        self.count_num_of_parallel_theory = 0
        self.max_threads_x_timestep = 0
        self.count_num_of_parallel_real = 0
        self.heatmap = np.zeros((dimensions[0], dimensions[1]))
        #serve per vedere quanti thread hanno finito nel mentre che vengono eseguite altre operazioni
        # self.just_finished_threads = 0
        # self.jft_lock = threading.Lock()
        self.finish_event = threading.Event()
        self.finish_event_lock = threading.Lock()
        self.print_lock = threading.Lock()
        #vedi sotto


    def update_max_threads(self, executive_threads):
        self.max_threads_x_timestep = max(self.max_threads_x_timestep, len(executive_threads))

    def get_count_num_of_parallel_real(self):
        return self.count_num_of_parallel_real
    def get_count_num_of_parallel_theory(self):
        return self.count_num_of_parallel_theory

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

    def get_number_of_segments(self):
        return self.number_of_segments

    def init_global_token(self):
        self.global_token['tasks'] = {}
        self.global_token['start_tasks_times'] = {}
        self.global_token['completed_tasks_times'] = {}

        self.global_token['pre_assignment_agents_tasks'] = {}
        self.global_token['completed_tasks'] = []
        self.global_token['agents_to_segments'] = {}
        self.global_token['occupied_non_task_endpoints'] = set()
        #dizionario con corrispondenza agent_name -> lista di zone da visitare
        self.global_token['high_level_path_to_loc1'] = {}
        self.global_token['high_level_path_to_loc2'] = {}
        self.global_token['discarded_tasks_for_agents'] = {}
        self.global_token['current_goals'] = set()

        for t in self.simulation.get_new_tasks():
            self.global_token['tasks'][t['task_name']] = [t['pickup'], t['delivery']]
            self.global_token['start_tasks_times'][t['task_name']] = self.simulation.get_time()

        for a in self.agents:
            pos = [a['start']]
            self.global_token['agents_to_segments'][a['name']] = []
            self.global_token['agents_to_segments'][a['name']].append(self.find_segment(pos[0]))
            if tuple(pos[0]) in self.non_task_endpoints:
                self.global_token['occupied_non_task_endpoints'].add(tuple(a['start']))

            self.global_token['high_level_path_to_loc1'][a['name']] = []
            self.global_token['high_level_path_to_loc2'][a['name']] = []
            self.global_token['discarded_tasks_for_agents'][a['name']] = set()

    #initialize a single token
    def init_token(self, index=0, segment=None):
        self.local_tokens[index]['agents'] = {}
        self.local_tokens[index]['path_ends'] = set()
        self.local_tokens[index]['segment'] = segment  #x_min, y_min, x_max, y_max
        self.local_tokens[index]['own_bridges'] = {}
        self.local_tokens[index]['occupied_bridges'] = {}

        # qui salvo solo le frontiere che partono dalla partizione corrente
        # per ogni destinazione ho una lista frontiere che mi ci portano
        for f in self.bridges:
            if f.start_segment == index:
                try:
                    self.local_tokens[index]['own_bridges'][f.destination_segment].append(f)
                except:
                    self.local_tokens[index]['own_bridges'][f.destination_segment] = [f]

        for a in self.agents:
            if self.find_segment(a['start']) == index:
                self.local_tokens[index]['agents'][a['name']] = [a['start']]
                if not tuple(a['start']) in self.non_task_endpoints:
                    self.local_tokens[index]['path_ends'].add(tuple(a['start']))

    #initialize all local tokens
    def init_local_tokens(self, partitions):
        for t in range(self.number_of_segments):
            self.local_tokens.append({})
            self.init_token(t, partitions[t])

    def convert_bridges(self, br):
        bridges = []
        for f in br:
            bridges.append(bridge(f))
        return bridges

    def create_graph(self):
        for f in self.bridges:
            self.graph.add_edge(f.start_segment, f.destination_segment, 1)

    def find_segment(self, pos):
        return self.matrix_cells_segments[pos[0]][pos[1]]

    def get_start_tasks_times(self):
        return self.global_token['start_tasks_times']

    def get_idle_agents_without_preass(self):

        #itero per ogni partizione/token così da avere la posizione attuale dell'agente
        agents = {}
        for t in range(self.number_of_segments):
            for name, path in self.local_tokens[t]['agents'].items():
                if name not in self.global_token['pre_assignment_agents_tasks']:
                    agents[name] = path

        return agents

    def get_idle_agents_global(self):
        all_idle_agents = self.local_tokens[0]['agents'].copy()

        for t in range(1, self.number_of_segments):
            all_idle_agents.update(self.local_tokens[t]['agents'].copy())

        return all_idle_agents

    #agenti che hanno un task assegnato e per cui devo pianificare (quelli in idle non ci sono perchè non hanno un pre_ass)
    def get_agents_to_plan(self):
        agents = {}
        for t in range(self.number_of_segments):
            for name, path in self.local_tokens[t]['agents'].items():
                if name in self.global_token['pre_assignment_agents_tasks'] and len(path) == 1:
                    agents[name] = path

        agents_copy = agents.copy()

        #agenti in migrazione
        for name, path in agents_copy.items():
            if len(self.global_token['agents_to_segments'][name]) > 1:
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

    def is_migrating(self, agent_name):
        migrante = False
        if len(self.global_token['high_level_path_to_loc1'][agent_name]) > 1:
            migrante = True
        elif len(self.global_token['high_level_path_to_loc2'][agent_name]) > 1:
            migrante = True
        return migrante

    def get_moving_obstacles_agents(self, agents, time_start):
        obstacles = {}
        negative_obstacles = {}
        for name, path in agents.items():  
            if len(path) > time_start and len(path) > 1:
                for i in range(time_start, len(path)):
                    k = i - time_start
                    obstacles[(path[i][0], path[i][1], k)] = name
                    # if the last position is an entry cell, I put the negative obstacle provided
                    # but only if I'm not migrating (len areas == 2), if I'm migrating I don't put negative
                    if i == len(path) - 1 and (not self.is_entry_cell(path[i]) or len(
                            self.global_token['agents_to_segments'][name]) == 1):
                        try:
                            negative_obstacles[-k].append((path[i][0], path[i][1]))
                        except:
                            negative_obstacles[-k] = [(path[i][0], path[i][1])]
        return obstacles, negative_obstacles

    def is_entry_cell(self, pos):

        own_partition = self.find_segment(pos)
        for f in self.local_tokens[own_partition]['own_bridges']:
            for front in self.local_tokens[own_partition]['own_bridges'][f]:
                if front.entry_cell == tuple(pos):
                    return True
        return False

    def abort_planning(self, agent_name, pos_to_go, seg_index):

        #frontiere occupate per un tempo indefinito
        occupied_bridges = set()
        for a in self.local_tokens[seg_index]['occupied_bridges']:
            # se invece l'agente ha due aree associate vuol dire che sta migrando e quindi
            # la frontiera non è occupata per un tempo indefinito
            # caso particolare: ha 2 aree e la seconda è part index perché ci sta migrando
            if a != agent_name and (len(self.global_token['agents_to_segments'][a]) == 1 or
                                    (len(self.global_token['agents_to_segments'][a]) == 2 and
                                     self.global_token['agents_to_segments'][a][1] == seg_index)):
                occupied_bridges.add(self.local_tokens[seg_index]['occupied_bridges'][a])

        # if agent_name in self.tokens[seg_index]['occupied_bridges']:
        # primo ciclo trova tutte le chiavi di own_frontiers, cioè le partizioni verso cui si può andare
        for f in self.local_tokens[seg_index]['own_bridges']:
            # frontiere verso la singola partizione
            for front in self.local_tokens[seg_index]['own_bridges'][f]:
                # corrisponde ad una frontiera e len areas == 1 abort planning, finirò in un A* limit
                if front.entry_cell == tuple(pos_to_go):
                    if front in occupied_bridges:
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
                if len(agents_paths[agent]) == 1 and len(self.global_token['agents_to_segments'][agent]) == 1:
                    obstacles.add((agents_paths[agent][0][0], agents_paths[agent][0][1]))
                # presumo agenti che finiranno il loro percorso e si fermeranno? Quindi metto ultima
                # loro posizione
                if 1 < len(agents_paths[agent]) <= time_start:
                    #idle obstacles se sto andando alla frontiera, se invece sono lì ed ho pianificato di spostarmi
                    #non lo metto
                    if (not self.is_entry_cell(agents_paths[agent][-1]) or len(
                            self.global_token['agents_to_segments'][agent]) == 1):
                        obstacles.add((agents_paths[agent][-1][0], agents_paths[agent][-1][1]))

        return obstacles

    def check_safe1(self, agent_pos):
        return tuple(agent_pos) in self.non_task_endpoints

    def check_safe2(self, agent_pos):
        for task_name, task in self.global_token['tasks'].items():
            if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                return False

        return True

    # deprecated
    def check_safe3(self, agent_pos):
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False
        return True

    def check_safe4(self, agent_pos):
        for f in self.bridges:
            if f.entry_cell == tuple(agent_pos) or f.exit_cell == tuple(agent_pos):
                return False
        return True

    def check_safe_idle(self, agent_pos):
        if tuple(agent_pos) in self.non_task_endpoints:
            return True

        #becca solo i delivery
        if tuple(agent_pos) in self.goal_endpoints:
            for task_name, task in self.global_token['tasks'].items():
                if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                    return False
            return True

        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False

        own_partition = self.find_segment(agent_pos)
        for f in self.local_tokens[own_partition]['own_bridges']:
            for front in self.local_tokens[own_partition]['own_bridges'][f]:
                if front.entry_cell == tuple(agent_pos):
                    return False

        return True

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
            if endpoint not in self.global_token['occupied_non_task_endpoints']:
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


    def no_agent_passing_on_bridge(self, pos, seg_index):
        for a in self.local_tokens[seg_index]['agents']:
            if [pos[0], pos[1]] in self.local_tokens[seg_index]['agents'][a]:
                return False
        return True
    
    #it returns the closest bridge to the agent
    def get_closest_bridge(self, agent_pos, frontiers_to_next_part, actual_part):
        occupied_bridges = set()
        for a in self.local_tokens[actual_part]['occupied_bridges']:
            # se invece l'agente ha due aree associate vuol dire che sta migrando e quindi
            # la frontiera non è occupata per un tempo indefinito
            if len(self.global_token['agents_to_segments'][a]) == 1:
                occupied_bridges.add(self.local_tokens[actual_part]['occupied_bridges'][a])

        dist = -1
        res = -1
        for f in frontiers_to_next_part:
            if f not in occupied_bridges and self.no_agent_passing_on_bridge(f.entry_cell, actual_part):
                if dist == -1:
                    dist = self.admissible_heuristic(f.entry_cell, agent_pos)
                    res = f
                else:
                    tmp = self.admissible_heuristic(f.entry_cell, agent_pos)
                    if tmp < dist:
                        dist = tmp
                        res = f

        if res == -1:
            with self.print_lock:
                print('*************** NO AVAILABLE FRONTIER in actual part:', actual_part, agent_pos, '****************')
            #exit(1)
        return res

    def update_ends(self, agent_pos, seg_index):
        if tuple(agent_pos) in self.local_tokens[seg_index]['path_ends']:
            self.local_tokens[seg_index]['path_ends'].remove(tuple(agent_pos))

    def not_in_assigned_goals(self, pos0, pos1):
        for el in self.global_token['pre_assignment_agents_tasks'].values():
            if tuple(el['goal']) == tuple(pos0) or tuple(el['goal']) == tuple(pos1):
                return False
        return True

    def not_in_assigned_goals2(self, pos0, pos1, assigned_goals):
        return tuple(pos0) not in assigned_goals and tuple(pos1) not in assigned_goals

    def not_in_assigned_goals3(self, pos0, pos1):
        return tuple(pos1) not in self.global_token['current_goals'] and tuple(pos0) not in self.global_token['current_goals']

    def get_agents_to_tasks_starts_goals(self):
        starts_goals = set()
        for el in self.global_token['pre_assignment_agents_tasks'].values():
            starts_goals.add(tuple(el['goal']))
            starts_goals.add(tuple(el['start']))
        return starts_goals

    def get_completed_tasks(self):
        return self.global_token['completed_tasks']

    def get_completed_tasks_times(self):
        return self.global_token['completed_tasks_times']

    def get_token(self, index):
        return self.local_tokens[index]

    def get_global_token(self):
        return self.global_token

    #cbs single agent quindi Astar
    def search(self, cbs, seg_index):

        path, espansioniA = cbs.search()
        with self.update_A_lock:
            self.chiamateAstar += 1
            self.sommaEspansioniAtot += espansioniA
            self.espansioniAstarXpart[seg_index] += espansioniA
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
            self.global_token['tasks'][t['task_name']] = [t['pickup'], t['delivery']]
            self.global_token['start_tasks_times'][t['task_name']] = self.simulation.get_time()

    def update_completed_tasks(self):
        for agent in self.agents:
            agent_name = agent['name']
            pos = self.simulation.actual_paths[agent_name][-1]
            partition = self.find_segment([pos['x'], pos['y']])

            # ---------------------Agent is arrived now------------------

            # if the agent has a task assigned and its current position is equal to its goal
            # and its path is of length 1 and its task is not safe idle
            if agent_name in self.global_token['pre_assignment_agents_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.global_token['pre_assignment_agents_tasks'][agent_name]['goal']) \
                    and len(self.local_tokens[partition]['agents'][agent_name]) == 1 and \
                    self.global_token['pre_assignment_agents_tasks'][agent_name][
                        'task_name'] != 'safe_idle' and self.global_token['high_level_path_to_loc2'][agent_name] == []:

                #the check on high_level_paht2 is to avoid that an agent that has as goal its own starting point
                #and that has not been able to plan is marked as an agent that has completed the task

                self.global_token['completed_tasks'].append(
                    self.global_token['pre_assignment_agents_tasks'][agent_name]['task_name'])
                self.global_token['completed_tasks_times'][
                    self.global_token['pre_assignment_agents_tasks'][agent_name][
                        'task_name']] = self.simulation.get_time()
                task_to_remove = self.global_token['pre_assignment_agents_tasks'].pop(agent_name)
                self.global_token['current_goals'].remove(tuple(task_to_remove['goal']))
                self.global_token['discarded_tasks_for_agents'][agent_name] = set()

            if agent_name in self.global_token['pre_assignment_agents_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.global_token['pre_assignment_agents_tasks'][agent_name]['goal']) \
                    and len(self.local_tokens[partition]['agents'][agent_name]) == 1 and \
                    self.global_token['pre_assignment_agents_tasks'][agent_name][
                        'task_name'] == 'safe_idle':
                task_to_remove = self.global_token['pre_assignment_agents_tasks'].pop(agent_name)
                self.global_token['current_goals'].remove(tuple(task_to_remove['goal']))
                self.global_token['discarded_tasks_for_agents'][agent_name] = set()

    def check_path_ends(self, agent_pos, task, all_path_ends):
        return (tuple(task[0]) == tuple(agent_pos) or tuple(task[0]) not in all_path_ends) \
                and (tuple(task[1]) == tuple(agent_pos) or tuple(task[1]) not in all_path_ends)

    def check_path_ends2(self, agent_pos, task):

        if tuple(task[0]) != tuple(agent_pos):
            # I have to check that the agent is not in the path ends of other agents
            for a in range(self.number_of_segments):
                if tuple(task[0]) in self.local_tokens[a]['path_ends']:
                    return False

        if tuple(task[1]) != tuple(agent_pos):
            for a in range(self.number_of_segments):
                if tuple(task[1]) in self.local_tokens[a]['path_ends']:
                    return False

        return True

    def check_path_ends3(self, agent_pos, task):

        if tuple(task[0]) != tuple(agent_pos):
            part = self.find_segment(task[0])
            # I have to check that the agent is not in the path ends of other agents
            if tuple(task[0]) in self.local_tokens[part]['path_ends']:
                return False

        if tuple(task[1]) != tuple(agent_pos):
            part = self.find_segment(task[1])
            # I have to check that the agent is not in the path ends of other agents
            if tuple(task[1]) in self.local_tokens[part]['path_ends']:
                return False

        return True

    def create_all_path_ends(self):
        all_path_ends = set()
        for a in range(self.number_of_segments):
            for tup in self.local_tokens[a]['path_ends']:
                all_path_ends.add(tuple(tup))
        return all_path_ends

    def create_assigned_goals(self):
        assigned_goals = set()
        for el in self.global_token['pre_assignment_agents_tasks'].values():
            assigned_goals.add(tuple(el['goal']))
        return assigned_goals

    def find_available_tasks(self, agent_pos, agent_name):
        available_tasks = {}
        for task_name, task in self.global_token['tasks'].items():
            if self.not_in_assigned_goals3(task[0], task[1]) and self.check_path_ends3(agent_pos, task):
                if task_name not in self.global_token['discarded_tasks_for_agents'][agent_name]:
                    available_tasks[task_name] = task

        return available_tasks

    def choose_task(self, agent_name, agent_pos, available_tasks): 
        closest_task_name = self.get_closest_task_name(available_tasks, agent_pos)
        closest_task = available_tasks.pop(closest_task_name)
        self.global_token['tasks'].pop(closest_task_name)
        pickup = closest_task[0]
        delivery = closest_task[1]
        self.global_token['pre_assignment_agents_tasks'][agent_name] = {'task_name': closest_task_name, 'start': pickup,
                                                                       'goal': delivery}
        self.global_token['current_goals'].add(tuple(delivery))


    def choose_non_task_endpoint(self, agent_name, agent_pos):  #, all_idle_agents):
        closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos)
        if closest_non_task_endpoint != -1:
            self.global_token['pre_assignment_agents_tasks'][agent_name] = {'task_name': "safe_idle", 'start': agent_pos,
                                                                           'goal': closest_non_task_endpoint}
            self.global_token['current_goals'].add(tuple(closest_non_task_endpoint))
            self.global_token['occupied_non_task_endpoints'].add(tuple(closest_non_task_endpoint))


    def compute_real_path_double(self, agent_name, agent_pos, loc1, loc2, all_idle_agents, seg_index, time_start=0):
        if self.abort_planning(agent_name, loc1, seg_index) or self.abort_planning(agent_name, loc2, seg_index):
            return False

        moving_obstacles_agents, negative_moving_obstacles = self.get_moving_obstacles_agents(self.local_tokens[seg_index]['agents'], time_start)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)
        idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(loc1)})
        idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(loc1)})
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': loc1}
        env = Environment(self.local_tokens[seg_index]['segment'], [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, negative_moving_obstacles, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path1 = self.search(cbs, seg_index)
        if not path1:
            with self.print_lock:
                print("Solution not found to loc1 for agent", agent_name, " idling at current position...")
            return False
        else:
            #print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            cost1 = env.compute_solution_cost(path1)

            moving_obstacles_agents, negative_moving_obstacles = self.get_moving_obstacles_agents(self.local_tokens[seg_index]['agents'],
                                                                                                  time_start + cost1 - 1)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start + cost1 - 1, agent_name)
            idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(loc1), tuple(loc2)})
            idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(loc1), tuple(loc2)})
            idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

            agent = {'name': agent_name, 'start': loc1, 'goal': loc2}
            env = Environment(self.local_tokens[seg_index]['segment'], [agent], self.obstacles | idle_obstacles_agents,
                              moving_obstacles_agents, negative_moving_obstacles, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path2 = self.search(cbs, seg_index)
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
                self.apply_path(agent_name, agent_pos, path1[agent_name], path2[agent_name], seg_index)
                return True

    def compute_real_path_single(self, agent_name, agent_pos, goal_position, all_idle_agents, seg_index, time_start=0, inside_other_functions=True):

        if self.abort_planning(agent_name, goal_position, seg_index):
            return False

        moving_obstacles_agents, negative_moving_obstacles = self.get_moving_obstacles_agents(self.local_tokens[seg_index]['agents'], time_start)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)
        idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(goal_position)})
        idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(goal_position)})
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': goal_position}
        env = Environment(self.local_tokens[seg_index]['segment'], [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, negative_moving_obstacles, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path = self.search(cbs, seg_index)

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

            self.apply_path(agent_name, agent_pos, None, path[agent_name], seg_index)
            outcome = True

        if not inside_other_functions:
            self.finish_event.set()
        return outcome

    # se ho solo un path passo solo il secondo
    def apply_path(self, agent_name, agent_pos, path1, path2, seg_index):
        last_step = path2[-1]
        agent_part = self.find_segment(agent_pos)
        self.update_ends(agent_pos, agent_part)

        self.local_tokens[seg_index]['agents'][agent_name] = []

        self.local_tokens[seg_index]['path_ends'].add(tuple([last_step['x'], last_step['y']]))
        if path1 is not None:
            #self.tokens[seg_index]['path_ends'].add(tuple([last_step['x'], last_step['y']]))
            for el in path1:
                self.local_tokens[seg_index]['agents'][agent_name].append([el['x'], el['y']])
            self.local_tokens[seg_index]['agents'][agent_name] = self.local_tokens[seg_index]['agents'][agent_name][:-1]

        for el in path2:
            self.local_tokens[seg_index]['agents'][agent_name].append([el['x'], el['y']])

        self.sumOfCosts += len(self.local_tokens[seg_index]['agents'][agent_name])

        #update matrix
        for t in self.local_tokens[seg_index]['agents'][agent_name]:
            self.heatmap[t[0], t[1]] += 1

    def assign_tasks(self):
        idle_agents = self.get_idle_agents_without_preass()

        while len(idle_agents) > 0:
            agent_name = random.choice(list(idle_agents.keys()))
            all_idle_agents = self.get_idle_agents_global()
            all_idle_agents.pop(agent_name)
            agent_pos = idle_agents.pop(agent_name)[0]
            available_tasks = self.find_available_tasks(agent_pos, agent_name)

            if len(available_tasks) > 0:
                self.choose_task(agent_name, agent_pos, available_tasks)

            elif self.check_safe_idle(agent_pos):
                a = 0
                print('No available tasks for agent', agent_name, ' idling at current position...')

            else:
                self.choose_non_task_endpoint(agent_name, agent_pos)

    def compute_high_level_path(self, start, goal, agent_name, loc):

        start_partition = self.find_segment(start)
        goal_partition = self.find_segment(goal)

        path = find_path(self.graph, start_partition, goal_partition)
        if loc == 1:
            self.global_token['high_level_path_to_loc1'][agent_name] = path.nodes
            if len(path.nodes) == 1 and start == goal:
                self.global_token['high_level_path_to_loc1'][agent_name] = []

        else:
            self.global_token['high_level_path_to_loc2'][agent_name] = path.nodes

    def go_to_bridge(self, agent_name, agent_pos, all_idle_agents, actual_part, next_part):

        bridges_to_next_seg = self.local_tokens[actual_part]['own_bridges'][next_part]
        closest_bridge = self.get_closest_bridge(agent_pos, bridges_to_next_seg, actual_part)

        if closest_bridge != -1:
            self.compute_real_path_single(agent_name, agent_pos, closest_bridge.start_pos, all_idle_agents,
                                          actual_part)
            self.local_tokens[actual_part]['occupied_bridges'][agent_name] = closest_bridge

        elif len(self.local_tokens[actual_part]['agents'][agent_name]) == 1 and \
                (len(self.global_token['high_level_path_to_loc1'][agent_name]) > 0 or agent_pos ==
                 self.global_token['pre_assignment_agents_tasks'][agent_name]['start']):
            self.remove_task_from_agents(agent_name, [actual_part])

        self.finish_event.set()

    def find_next_goal(self, agent_name, agent_pos, next_part, num_abs):
        abstract = "high_level_path_to_loc" + str(num_abs)

        if len(self.global_token[abstract][agent_name]) > 2:
            bridges_to_next_part = self.local_tokens[next_part]['own_bridges'][self.global_token[abstract][agent_name][2]]
            closest_bridge = self.get_closest_bridge(agent_pos, bridges_to_next_part,
                                                         self.global_token[abstract][agent_name][1])
            if closest_bridge != -1:
                self.local_tokens[next_part]['occupied_bridges'][agent_name] = closest_bridge
                return closest_bridge.start_pos
            else:
                #print('*************** NO AVAILABLE FRONTIER in actual part:', next_part, agent_pos, '****************')
                return -1

        elif len(self.global_token[abstract][agent_name]) == 2:
            return self.global_token['pre_assignment_agents_tasks'][agent_name]['goal']

    def migrazione(self, agent_name, agent_pos, actual_part, next_part, num_abs, closest_bridge):

        all_idle_agents = self.local_tokens[next_part]['agents'].copy()

        pic_in_part = (num_abs == 1 and len(self.global_token['high_level_path_to_loc1'][agent_name]) == 2)


        if pic_in_part:
            valid_path = self.pickup_in_segment(agent_name, agent_pos, self.global_token['pre_assignment_agents_tasks'][agent_name]['start'],
                                                all_idle_agents, next_part, time_start=0, inside_migration=True)
        else:
            next_goal = self.find_next_goal(agent_name, closest_bridge.exit_cell, next_part, num_abs)
            if next_goal == -1:
                valid_path = False
            else:
                valid_path = self.compute_real_path_single(agent_name, agent_pos, next_goal, all_idle_agents, next_part,time_start=0)

        if valid_path:
            with self.print_lock:
                print('Agent', agent_name, 'migrating to partition', next_part, '...')
            num_wait = self.local_tokens[next_part]['agents'][agent_name].count(agent_pos)
            if num_wait > 1:
                self.local_tokens[actual_part]['agents'][agent_name] = []
                for i in range(num_wait):
                    self.local_tokens[actual_part]['agents'][agent_name].append([agent_pos[0], agent_pos[1]])
                self.global_token['agents_to_segments'][agent_name].append(next_part)

            else:
                self.global_token['agents_to_segments'][agent_name].append(next_part)
                self.update_ends(agent_pos, actual_part)

        else:
            with self.print_lock:
                print('NO PATH AFTER MIGRATION', agent_name, ' idling at current position...')

            self.local_tokens[actual_part]['agents'][agent_name].append([agent_pos[0], agent_pos[1]])

            if agent_name in self.global_token['pre_assignment_agents_tasks'] and (
                    num_abs == 1 or agent_pos == self.global_token['pre_assignment_agents_tasks'][agent_name]['start']):
                self.remove_task_from_agents(agent_name, [actual_part, next_part])
        self.finish_event.set()

    # returns the destination partition
    def on_an_entry_cell(self, agent_name, agent_pos, actual_segment):

        if agent_name in self.local_tokens[actual_segment]['occupied_bridges']:
            f = self.local_tokens[actual_segment]['occupied_bridges'][agent_name]
            if tuple(agent_pos) == f.entry_cell:
                return f.destination_segment

        return -1

    def remove_task_from_agents(self, agent_name, part_to_remove):
        print('Remove task from', agent_name)
        task_name = self.global_token['pre_assignment_agents_tasks'][agent_name]['task_name']
        if task_name != 'safe_idle':
            self.global_token['discarded_tasks_for_agents'][agent_name].add(task_name)
            self.global_token['tasks'][task_name] = \
                [self.global_token['pre_assignment_agents_tasks'][agent_name]['start'],
                 self.global_token['pre_assignment_agents_tasks'][agent_name]['goal']]
        elif tuple(self.global_token['pre_assignment_agents_tasks'][agent_name]['goal']) in self.global_token[
            'occupied_non_task_endpoints']:
            self.global_token['occupied_non_task_endpoints'].remove(
                tuple(self.global_token['pre_assignment_agents_tasks'][agent_name]['goal']))

        task_to_remove = self.global_token['pre_assignment_agents_tasks'].pop(agent_name)
        self.global_token['current_goals'].remove(tuple(task_to_remove['goal']))

        self.global_token['high_level_path_to_loc1'][agent_name] = []
        self.global_token['high_level_path_to_loc2'][agent_name] = []

        for part in part_to_remove:
            if agent_name in self.local_tokens[part]['occupied_bridges']:
                self.local_tokens[part]['occupied_bridges'].pop(agent_name)

    def pickup_in_segment(self, agent_name, agent_pos, pickup_position, all_idle_agents, seg_index, time_start=0, inside_migration=False):

        if len(self.global_token['high_level_path_to_loc2'][agent_name]) == 1:
            loc2 = self.global_token['pre_assignment_agents_tasks'][agent_name]['goal']
            planned = self.compute_real_path_double(agent_name, agent_pos, pickup_position, loc2, all_idle_agents,
                                                    seg_index, time_start)
        else:
            next_seg = self.global_token['high_level_path_to_loc2'][agent_name][1]
            bridge_to_next_seg = self.local_tokens[seg_index]['own_bridges'][next_seg]
            closest_bridge = self.get_closest_bridge(pickup_position, bridge_to_next_seg, seg_index)
            if closest_bridge != -1:
                # do per scontato che trovo sempre un path
                planned = self.compute_real_path_double(agent_name, agent_pos, pickup_position,
                                                        closest_bridge.start_pos,
                                                        all_idle_agents, seg_index, time_start)
                self.local_tokens[seg_index]['occupied_bridges'][agent_name] = closest_bridge

        if not planned and self.find_segment(agent_pos) == seg_index:
            self.remove_task_from_agents(agent_name, [seg_index])

        if not inside_migration:
            self.finish_event.set()

        return planned

    def update_A_star_stats(self):
        self.sommaEspansioniAmaxTimestep += max(self.espansioniAstarXpart)

        if sum(self.espansioniAstarXpart) != max(self.espansioniAstarXpart):
            self.parallel_rounds += 1
            for e in self.espansioniAstarXpart:
                if e > 0:
                    self.count_num_of_parallel_theory += 1
            self.count_num_of_parallel_real += self.max_threads_x_timestep
            self.max_threads_x_timestep = 0


        for i in range(self.number_of_segments):
            self.vec_areas[i].append(self.espansioniAstarXpart[i])

        self.espansioniAstarXpart = [0] * self.number_of_segments

    def check_non_te(self):
        count = 0
        for part in range(self.number_of_segments):
            for agent in self.local_tokens[part]['agents']:
                if tuple(self.local_tokens[part]['agents'][agent][0]) in self.non_task_endpoints:
                    count += 1

        if count != len(self.global_token['occupied_non_task_endpoints']):
            print('ERROR NON TASK ENDPOINTS')
            exit(1)

    def clean_threads(self, executive_threads):
        # removes not active threads
        finished_threads = [key for key, thread in executive_threads.items() if not thread.is_alive()]
        for key in finished_threads:
            del executive_threads[key]

    def check_available_token(self, agent_name, seg_index, waiting_agents, executive_threads):

        self.clean_threads(executive_threads)
        if seg_index not in executive_threads.keys():
            return True
        else:
            try:
                waiting_agents[seg_index].append(agent_name)
            except KeyError:
                waiting_agents[seg_index] = [agent_name]
            return False

    def path_planning(self, agent_name, agent_pos, waiting_agents, executive_threads):
        agent_segment = self.global_token['agents_to_segments'][agent_name][0]

        local_idle_agents = self.local_tokens[agent_segment]['agents'].copy()
        local_idle_agents.pop(agent_name)

        # se non ha abstract path(s) calcolo
        if len(self.global_token['high_level_path_to_loc1'][agent_name]) == 0 and \
                len(self.global_token['high_level_path_to_loc2'][agent_name]) == 0:
            self.compute_high_level_path(agent_pos,
                                         self.global_token['pre_assignment_agents_tasks'][agent_name]['start'],
                                         agent_name, 1)
            self.compute_high_level_path(self.global_token['pre_assignment_agents_tasks'][agent_name]['start'],
                                         self.global_token['pre_assignment_agents_tasks'][agent_name]['goal'],
                                         agent_name, 2)

        # -------------------------------------------------------------
        if len(self.global_token['high_level_path_to_loc1'][agent_name]) > 1:
            on_an_entry_cell = self.on_an_entry_cell(agent_name, agent_pos, agent_segment)
            # it contains the destination segment
            if on_an_entry_cell != -1:
                closest_bridge = self.local_tokens[agent_segment]['occupied_bridges'][agent_name]
                # check if the local token desired is available
                if self.check_available_token(agent_name, on_an_entry_cell, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.migrazione, args=(agent_name, agent_pos, agent_segment, on_an_entry_cell, 1, closest_bridge))
                    executive_threads[on_an_entry_cell] = th
                    th.start()
            else:
                # check if the local token desired is available
                if self.check_available_token(agent_name, agent_segment, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.go_to_bridge, args=(agent_name, agent_pos, local_idle_agents, agent_segment,
                                                                          self.global_token['high_level_path_to_loc1'][agent_name][1]))
                    executive_threads[agent_segment] = th
                    th.start()

        # pickup is in this segment
        # or first planning after task assignment
        elif len(self.global_token['high_level_path_to_loc1'][agent_name]) == 1:
            # check if the local token desired is available
            if self.check_available_token(agent_name, agent_segment, waiting_agents, executive_threads):
                th = threading.Thread(target=self.pickup_in_segment, args=(agent_name, agent_pos,
                                                                           self.global_token['pre_assignment_agents_tasks'][agent_name]['start'],
                                                                           local_idle_agents, agent_segment))
                executive_threads[agent_segment] = th
                th.start()

        # pickup completed, delivery/non-task endpoint in another segment
        elif len(self.global_token['high_level_path_to_loc2'][agent_name]) > 1:
            on_an_entry_cell = self.on_an_entry_cell(agent_name, agent_pos, agent_segment)
            #it contains the destination segmente
            if on_an_entry_cell != -1:
                closest_bridge = self.local_tokens[agent_segment]['occupied_bridges'][agent_name]
                # check if the local token desired is available
                if self.check_available_token(agent_name, on_an_entry_cell, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.migrazione, args=(agent_name, agent_pos, agent_segment, on_an_entry_cell, 2, closest_bridge))
                    executive_threads[on_an_entry_cell] = th
                    th.start()
            else:
                # check if the local token desired is available
                if self.check_available_token(agent_name, agent_segment, waiting_agents, executive_threads):
                    th = threading.Thread(target=self.go_to_bridge, args=(agent_name, agent_pos, local_idle_agents, agent_segment,
                                                                          self.global_token['high_level_path_to_loc2'][agent_name][1]))
                    executive_threads[agent_segment] = th
                    th.start()
        # The delivery location is in this segment
        elif len(self.global_token['high_level_path_to_loc2'][agent_name]) == 1:
            #check if the local token desired is available
            if self.check_available_token(agent_name, agent_segment, waiting_agents, executive_threads):
                th = threading.Thread(target=self.compute_real_path_single, args=(agent_name, agent_pos,
                                                                                 self.global_token['pre_assignment_agents_tasks'][agent_name]['goal'],
                                                                                 local_idle_agents, agent_segment, 0, False))
                executive_threads[agent_segment] = th
                th.start()

        else:
            print("High level path is empthy" + agent_name)

    def handle_waiting_agents(self, waiting_agents, executive_threads):

        copy_waiting_agents = waiting_agents.copy()
        while len(waiting_agents) > 0:
            self.clean_threads(executive_threads)
            for part in copy_waiting_agents:
                if part not in executive_threads.keys() and part in waiting_agents.keys():
                    agent_name = waiting_agents[part].pop(0)
                    if len(waiting_agents[part]) == 0:
                        del waiting_agents[part]

                    self.path_planning(agent_name, self.local_tokens[self.global_token['agents_to_segments'][agent_name][0]]['agents'][agent_name][0], waiting_agents, executive_threads)
            self.update_max_threads(executive_threads)
            with self.finish_event_lock:
                self.finish_event.wait()
                self.finish_event.clear()

    def time_forward(self):
        self.update_completed_tasks()
        self.collect_new_tasks()

        self.assign_tasks()

        agents_to_plan = self.get_agents_to_plan()
        priority_agents = {}
        #dict of partitions, for each one with a list of waiting agents
        waiting_agents = {}
        #dict of partitions, for each one with the executing thread
        executive_threads = {}
        copy_atp = agents_to_plan.copy()
        for agent in copy_atp:
            if agent in self.local_tokens[self.global_token['agents_to_segments'][agent][0]]['occupied_bridges']:
                priority_agents[agent] = agents_to_plan[agent]
                agents_to_plan.pop(agent)

        # path selection for each agent with priority or not
        while len(agents_to_plan) > 0 or len(priority_agents) > 0:

            if len(priority_agents) > 0:
                agent_name = random.choice(list(priority_agents.keys()))
                agent_pos = priority_agents.pop(agent_name)[0]
            else:
                agent_name = random.choice(list(agents_to_plan.keys()))
                agent_pos = agents_to_plan.pop(agent_name)[0]

            self.path_planning(agent_name, agent_pos, waiting_agents, executive_threads)
            self.update_max_threads(executive_threads)

        self.handle_waiting_agents(waiting_agents, executive_threads)

        # wait ultil all agents have finished to plan
        for t in executive_threads:
            executive_threads[t].join()

        #self.update_non_task_endpoints()
        if 'safe_idle' in self.global_token['tasks']:
            self.global_token['tasks'].pop('safe_idle')

        #only for statistics
        self.update_A_star_stats()
