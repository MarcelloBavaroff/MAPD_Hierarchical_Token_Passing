from math import fabs
import random
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
                 goal_endpoints, frontiers, a_star_max_iter=100):
        random.seed(1234)
        self.agents = agents
        self.dimensions = dimensions
        self.obstacles = set(obstacles)
        self.non_task_endpoints = non_task_endpoints
        self.number_of_areas = number_of_areas
        self.frontiers = self.convert_frontiers(frontiers)
        self.partitions = partitions
        if len(agents) > len(non_task_endpoints):
            print('There are more agents than non task endpoints, instance is not well-formed.')
            exit(1)

        self.tokens = []
        self.simulation = simulation
        self.a_star_max_iter = a_star_max_iter
        self.chiamateAstar = 0
        self.sommaEspansioniAtot = 0
        self.sommaEspansioniAmaxTimestep = 0
        self.goal_endpoints = goal_endpoints
        self.global_view = {}
        self.init_global_view()
        self.init_tokens(partitions)
        self.graph = Graph()
        self.create_graph()

        self.espansioniAstarXpart = [0] * self.number_of_areas

        #vedi sotto

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

        for t in self.simulation.get_new_tasks():
            self.global_view['tasks'][t['task_name']] = [t['pickup'], t['delivery']]
            self.global_view['start_tasks_times'][t['task_name']] = self.simulation.get_time()

        for a in self.agents:
            pos = [a['start']]
            self.global_view['agents_to_areas'][a['name']] = []
            self.global_view['agents_to_areas'][a['name']].append(self.find_partition(pos[0]))
            if pos in self.non_task_endpoints:
                self.global_view['occupied_non_task_endpoints'].add(tuple(a['start']))

            self.global_view['abstract_to_loc1'][a['name']] = []
            self.global_view['abstract_to_loc2'][a['name']] = []
            self.global_view['discarded_tasks_for_agents'][a['name']] = []

    #initialize a single token
    def init_token(self, index=0, partition=None):
        self.tokens[index]['agents'] = {}
        self.tokens[index]['path_ends'] = []
        self.tokens[index]['partition'] = partition  #x_min, y_min, x_max, y_max
        self.tokens[index]['own_frontiers'] = {}
        #self.tokens[index]['occupied_non_task_endpoints'] = set()

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
                    self.tokens[index]['path_ends'].append(tuple(a['start']))
                # else:
                #     self.tokens[index]['occupied_non_task_endpoints'].add(tuple(a['start']))

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
        #print(self.graph)

    #restituisce l'indice della partizione in cui si trova la posizione pos (thanks co-pilot)
    def find_partition(self, pos):

        for i in range(self.number_of_areas):
            if self.partitions[i][0] <= pos[0] <= self.partitions[i][2] and self.partitions[i][1] <= pos[1] <= \
                    self.partitions[i][3]:
                return i
        return -1

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
        for name, path in agents.items():  #agents.item ritorna un dizionario con nome agente e coordinate dello stesso
            if len(path) > time_start and len(path) > 1:
                for i in range(time_start, len(path)):
                    k = i - time_start
                    obstacles[(path[i][0], path[i][1], k)] = name
                    #se ultima posizione del path non è una frontiera oppure è una frontiera
                    #e l'agente non sta migrando (abs1 = 1 o abs2 = 1) allora metto come ostacolo
                    #se gli ho cancellato il task LO metto come ostacolo
                    if i == len(path) - 1 and (not self.is_frontier_start_pos(path[i]) or not self.is_migrating(name)
                                               or (len(self.global_view['abstract_to_loc1'][name]) == 0 and len(
                                self.global_view['abstract_to_loc2'][name]) == 0)):
                        obstacles[(path[i][0], path[i][1], -k)] = name
        return obstacles

    def is_frontier_start_pos(self, pos):
        for f in self.frontiers:
            if f.start_pos == tuple(pos):
                return True
        return False

    def get_idle_obstacles_agents(self, agents_paths, time_start, agent_name):

        obstacles = set()
        #for g in self.goal_endpoints:
        #    obstacles.add(tuple(g))

        for agent in agents_paths:
            if agent != agent_name:
                # quelli nelle stazioni non li segno come ostacoli and tuple(path[0]) not in charging_stations_pos
                if len(agents_paths[agent]) == 1:
                    obstacles.add((agents_paths[agent][0][0], agents_paths[agent][0][1]))
                # presumo agenti che finiranno il loro percorso e si fermeranno? Quindi metto ultima
                # loro posizione
                if 1 < len(agents_paths[agent]) <= time_start:
                    #se l'agente sta per migrare non lo metto come idle obstacle
                    if (not self.is_frontier_start_pos(agents_paths[agent][-1]) or not self.is_migrating(agent)
                            or (len(self.global_view['abstract_to_loc1'][agent]) == 0 and
                                len(self.global_view['abstract_to_loc2'][agent]) == 0)):
                        obstacles.add((agents_paths[agent][-1][0], agents_paths[agent][-1][1]))

        return obstacles

    def check_safe_idle(self, agent_pos):

        if tuple(agent_pos) in self.non_task_endpoints:
            return True

        for task_name, task in self.global_view['tasks'].items():
            if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                return False
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False
        #probabilmente superfluo
        for f in self.frontiers:
            if f.start_pos == tuple(agent_pos) or f.destination_pos == tuple(agent_pos):
                return False
        return True

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
            print('Error in finding non-task endpoint, is instance well-formed?')
            exit(1)
        return res

    #restituisce una frontiera non una coordinata
    def get_closest_frontier(self, agent_pos, frontiers_to_next_part, discarded_frontiers=[]):
        dist = -1
        res = -1
        for f in frontiers_to_next_part:
            if f not in discarded_frontiers:
                if dist == -1:
                    dist = self.admissible_heuristic(f.entry_cell, agent_pos)
                    res = f
                else:
                    tmp = self.admissible_heuristic(f.entry_cell, agent_pos)
                    if tmp < dist:
                        dist = tmp
                        res = f

        if res == -1:
            print('Error in finding non-task endpoint, is instance well-formed?')
            exit(1)
        return res

    def update_ends(self, agent_pos, part_index):
        if tuple(agent_pos) in self.tokens[part_index]['path_ends']:
            self.tokens[part_index]['path_ends'].remove(tuple(agent_pos))
        #elif tuple(agent_pos) in self.tokens[part_index]['occupied_non_task_endpoints']:
        #    self.tokens[part_index]['occupied_non_task_endpoints'].remove(tuple(agent_pos))

    def get_agents_to_tasks_goals(self):
        goals = set()
        for el in self.global_view['pre_assignment_agents_tasks'].values():
            goals.add(tuple(el['goal']))
        return goals

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
        self.chiamateAstar += 1
        self.sommaEspansioniAtot += espansioniA
        self.espansioniAstarXpart[part_index] += espansioniA
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
                self.global_view['pre_assignment_agents_tasks'].pop(agent_name)
                self.global_view['discarded_tasks_for_agents'][agent_name] = []

            if agent_name in self.global_view['pre_assignment_agents_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']) \
                    and len(self.tokens[partition]['agents'][agent_name]) == 1 and \
                    self.global_view['pre_assignment_agents_tasks'][agent_name][
                        'task_name'] == 'safe_idle':
                self.global_view['pre_assignment_agents_tasks'].pop(agent_name)
                self.global_view['discarded_tasks_for_agents'][agent_name] = []

    #qui di base controlla che nessun agente abbia come path ends pickup o delivery ed
    # inoltre
    def find_available_tasks(self, agent_pos, agent_name):
        #part_index = self.find_partition(agent_pos)
        all_path_ends = set()
        for a in range(self.number_of_areas):
            for tup in self.tokens[a]['path_ends']:
                all_path_ends.add(tuple(tup))

        available_tasks = {}
        for task_name, task in self.global_view['tasks'].items():
            # se inizio e fine task non in path ends degli agenti (meno me) AND nemmeno in goals
            if tuple(task[0]) not in all_path_ends.difference({tuple(agent_pos)}) and tuple(
                    task[1]) not in all_path_ends.difference({tuple(agent_pos)}) \
                    and tuple(task[0]) not in self.get_agents_to_tasks_goals() and tuple(
                task[1]) not in self.get_agents_to_tasks_goals():

                #se da errore di chiave vuol dire che l'agente non ha task discarded

                if task_name not in self.global_view['discarded_tasks_for_agents'][agent_name]:
                    available_tasks[task_name] = task

        return available_tasks

    # qui metto nel token global l'assegnamento agente task
    def choose_task(self, agent_name, agent_pos, available_tasks):  #, all_idle_agents):
        closest_task_name = self.get_closest_task_name(available_tasks, agent_pos)
        closest_task = available_tasks.pop(closest_task_name)
        self.global_view['tasks'].pop(closest_task_name)
        pickup = closest_task[0]
        delivery = closest_task[1]
        self.global_view['pre_assignment_agents_tasks'][agent_name] = {'task_name': closest_task_name, 'start': pickup,
                                                                       'goal': delivery}

        # return self.compute_real_path(agent_name, agent_pos, closest_task, closest_task_name, all_idle_agents,
        #                               available_tasks)

    def choose_non_task_endpoint(self, agent_name, agent_pos):  #, all_idle_agents):
        closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos)
        self.global_view['pre_assignment_agents_tasks'][agent_name] = {'task_name': "safe_idle", 'start': agent_pos,
                                                                       'goal': closest_non_task_endpoint}
        self.global_view['occupied_non_task_endpoints'].add(tuple(closest_non_task_endpoint))

    def compute_real_path_double(self, agent_name, agent_pos, loc1, loc2, all_idle_agents, part_index, time_start=0):

        moving_obstacles_agents = self.get_moving_obstacles_agents(self.tokens[part_index]['agents'], time_start)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)
        idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(loc1)})
        idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(loc1)})
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': loc1}
        env = Environment(self.tokens[part_index]['partition'], [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path1 = self.search(cbs, part_index)
        if not path1:
            print("Solution not found to loc1 for agent", agent_name, " idling at current position...")
            return False
        else:
            #print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            cost1 = env.compute_solution_cost(path1)

            moving_obstacles_agents = self.get_moving_obstacles_agents(self.tokens[part_index]['agents'],
                                                                       time_start + cost1 - 1)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start + cost1 - 1, agent_name)
            idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(loc1), tuple(loc2)})
            idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(loc1), tuple(loc2)})
            idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

            agent = {'name': agent_name, 'start': loc1, 'goal': loc2}
            env = Environment(self.tokens[part_index]['partition'], [agent], self.obstacles | idle_obstacles_agents,
                              moving_obstacles_agents, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path2 = self.search(cbs, part_index)
            if not path2:
                print("Solution not found to task goal for agent", agent_name, " idling at current position...")
                return False
            else:
                print("Solution found to task start for agent", agent_name, " doing task...")
                # serve per mettere nel nuovo token l'agente che migra e solo dal timestep dopo iniziare il percorso
                for i in range(time_start):
                    path1[agent_name].insert(0, path1[agent_name][0])
                self.apply_path(agent_name, agent_pos, path1[agent_name], path2[agent_name], part_index)
                return True

    def compute_real_path_single(self, agent_name, agent_pos, goal_position, all_idle_agents, part_index, time_start=0):

        moving_obstacles_agents = self.get_moving_obstacles_agents(self.tokens[part_index]['agents'], time_start)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, time_start, agent_name)
        idle_obstacles_agents |= (set(self.non_task_endpoints) - {tuple(goal_position)})
        idle_obstacles_agents |= (set(self.goal_endpoints) - {tuple(goal_position)})
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos)}

        agent = {'name': agent_name, 'start': agent_pos, 'goal': goal_position}
        env = Environment(self.tokens[part_index]['partition'], [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, self.non_task_endpoints, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path = self.search(cbs, part_index)
        if not path:
            print("Solution not found to task goal for agent", agent_name, " idling at current position...")
            return False
        else:
            print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            #serve per mettere nel nuovo token l'agente che migra e solo dal timestep dopo iniziare il percorso
            #for i in range(time_start):
            #    path[agent_name].insert(0, path[agent_name][0])

            self.apply_path(agent_name, agent_pos, None, path[agent_name], part_index)
            return True

    # se ho solo un path passo solo il secondo
    def apply_path(self, agent_name, agent_pos, path1, path2, part_index):
        last_step = path2[-1]
        agent_part = self.find_partition(agent_pos)
        self.update_ends(agent_pos, agent_part)

        self.tokens[part_index]['agents'][agent_name] = []

        self.tokens[part_index]['path_ends'].append(tuple([last_step['x'], last_step['y']]))
        if path1 is not None:
            #self.tokens[part_index]['path_ends'].add(tuple([last_step['x'], last_step['y']]))
            for el in path1:
                self.tokens[part_index]['agents'][agent_name].append([el['x'], el['y']])
            # Don't repeat twice same step, elimino ultimo elemento
            self.tokens[part_index]['agents'][agent_name] = self.tokens[part_index]['agents'][agent_name][:-1]

        for el in path2:
            self.tokens[part_index]['agents'][agent_name].append([el['x'], el['y']])

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
                print('No available tasks for agent', agent_name, ' idling at current position...')

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
        discarded_frontiers = []

        while len(self.tokens[actual_part]['agents'][agent_name]) == 1 \
                and len(frontiers_to_next_part) > len(discarded_frontiers):
            closest_frontier = self.get_closest_frontier(agent_pos, frontiers_to_next_part, discarded_frontiers)
            self.compute_real_path_single(agent_name, agent_pos, closest_frontier.start_pos, all_idle_agents,
                                          actual_part)
            if len(self.tokens[actual_part]['agents'][agent_name]) == 1:
                discarded_frontiers.append(closest_frontier)

        # se non ho trovato una frontiera dove andare (len path = 1) cambio task
        # se abs1 ha almeno len 1 allora devo fare il pickup
        # se len(abs1) == 0 e sono sul pickup posso ancora cancellare il task
        if len(self.tokens[actual_part]['agents'][agent_name]) == 1 and \
            (len(self.global_view['abstract_to_loc1'][agent_name]) > 0 or agent_pos == self.global_view['pre_assignment_agents_tasks'][agent_name]['start']):
            self.remove_task_from_agents(agent_name)


    def find_next_goal(self, agent_name, agent_pos, next_part, num_abs):
        abstract = "abstract_to_loc" + str(num_abs)

        if len(self.global_view[abstract][agent_name]) > 2:
            #devo andare alla prossima frontiera
            frontiers_to_next_part = self.tokens[next_part]['own_frontiers'][self.global_view[abstract][agent_name][2]]
            closest_frontier = self.get_closest_frontier(agent_pos, frontiers_to_next_part)
            return closest_frontier.start_pos

        elif len(self.global_view[abstract][agent_name]) == 2:
            return self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']

    def migrazione(self, agent_name, agent_pos, actual_part, next_part, num_abs, agents_to_plan):
        #TODO: non è la più vicina, ma quella dove sono al momento che dovrei usare (coincidono quindi per ora ok)
        frontiers_to_next_part = self.tokens[actual_part]['own_frontiers'][next_part]
        closest_frontier = self.get_closest_frontier(agent_pos, frontiers_to_next_part)
        #next goal sarebbe da spostare
        next_goal = self.find_next_goal(agent_name, closest_frontier.destination_pos, next_part, num_abs)
        all_idle_agents = self.tokens[next_part]['agents'].copy()

        pic_in_part = (num_abs == 1 and len(self.global_view['abstract_to_loc1'][agent_name]) == 2)

        # se sto migrando, devo ancora fare il pickup e questo è nella partizione successiva
        if pic_in_part:
            #valid_path = self.pickup_in_partition(agent_name, closest_frontier.destination_pos, self.global_view['pre_assignment_agents_tasks'][agent_name]['start'], all_idle_agents, next_part, time_start=1)
            valid_path = self.pickup_in_partition(agent_name, agent_pos,
                                                  self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                                  all_idle_agents, next_part, time_start=0)
        # altrimenti o devo andare da una frontiera all'altra o al delivery
        else:
            #valid_path = self.compute_real_path_single(agent_name, closest_frontier.destination_pos, next_goal, all_idle_agents, next_part, time_start=0) #perché time_start = 0?
            valid_path = self.compute_real_path_single(agent_name, agent_pos, next_goal, all_idle_agents, next_part,
                                                       time_start=0)

        if valid_path:
            print('Agent', agent_name, 'migrating to partition', next_part, '...')
            #se serve rimanere il wait nella posizione di frontiera
            num_wait = self.tokens[next_part]['agents'][agent_name].count(agent_pos)
            if num_wait > 1:
                self.tokens[actual_part]['agents'][agent_name] = []
                for i in range(num_wait):
                    self.tokens[actual_part]['agents'][agent_name].append([agent_pos[0], agent_pos[1]])
                self.global_view['agents_to_areas'][agent_name].append(next_part)

                self.delete_conflicting_paths_more_strict(agent_name, actual_part)
                agents_to_plan = self.get_agents_to_plan()

            else:
                #self.global_view['agents_to_areas'][agent_name] = []
                self.global_view['agents_to_areas'][agent_name].append(next_part)
                #self.tokens[actual_part]['agents'].pop(agent_name)
                self.update_ends(agent_pos, actual_part)  # apply path aggiorna solo dell'area dopo

        else:
            print('NO PATH DOPO MIGRAZIONE', agent_name, ' idling at current position...')
            #segnali che l'agente rimarrà fermo in attesa di riprovare
            self.tokens[actual_part]['agents'][agent_name].append([agent_pos[0], agent_pos[1]])
            self.delete_conflicting_paths_more_strict(agent_name, actual_part)
            #se sono sulla casella di pickup posso considerare come se non lo avessi fatto
            if not pic_in_part or (agent_name in self.global_view['pre_assignment_agents_tasks'] and agent_pos == self.global_view['pre_assignment_agents_tasks'][agent_name]['start']):
                self.remove_task_from_agents(agent_name)
            agents_to_plan = self.get_agents_to_plan()

        return agents_to_plan

    def delete_conflicting_paths(self, agent_name, agent_pos, part_index, num_wait):
        for name, path in self.tokens[part_index]['agents'].items():
            if name != agent_name:
                for i in range(min(num_wait, len(path))):
                    if path[i] == agent_pos:
                        if self.global_view['agents_to_areas'][name][0] == part_index:
                            #dovrebbe tenere solo la pozione attuale
                            self.tokens[part_index]['agents'][name] = path[:1]
                            break
                        # se invece l'agente dovrà arrivare in questa partizione, ma attualmente è in frontiera altrove
                        else:
                            self.tokens[part_index]['agents'].pop(name)
                            self.tokens[self.global_view['agents_to_areas'][name][0]]['agents'][name] = path[:1]
                            break

    # cancella il path di tutti quelli che in un qualche istante di tempo andranno in agent_pos
    def delete_conflicting_paths_strict(self, agent_name, agent_pos, part_index, num_wait):
        copy = self.tokens[part_index]['agents'].copy()
        for name, path in copy.items():
            if name != agent_name and agent_pos in path:
                if self.global_view['agents_to_areas'][name][0] == part_index:
                    # dovrebbe tenere solo la pozione attuale
                    # non è più un path ends
                    self.update_ends(self.tokens[part_index]['agents'][name][-1], part_index)
                    self.tokens[part_index]['agents'][name] = path[:1]
                    self.tokens[part_index]['path_ends'].append(tuple(path[0]))
                # se invece l'agente dovrà arrivare in questa partizione, ma attualmente è in frontiera altrove
                else:
                    self.update_ends(self.tokens[part_index]['agents'][name][-1], part_index)
                    self.tokens[part_index]['agents'].pop(name)
                    #self.global_view['agents_to_areas'][name] = self.global_view['agents_to_areas'][name][:1]
                    self.global_view['agents_to_areas'][name] = []
                    self.global_view['agents_to_areas'][name].append(self.find_partition(path[0]))
                    self.tokens[self.global_view['agents_to_areas'][name][0]]['agents'][name] = path[:1]
                    self.tokens[self.global_view['agents_to_areas'][name][0]]['path_ends'].append(tuple(path[0]))

    #rimuovo tutti tranne quelli che sono in attesa su una partizione e che hanno già pianificato
    #la migrazione
    def delete_conflicting_paths_more_strict(self, agent_name, part_index):
        copy = self.tokens[part_index]['agents'].copy()
        for name, path in copy.items():
            if name != agent_name:
                #se è == 2 vuol dire che l'agente è in attesa su una frontiera e non lo tocco
                if self.global_view['agents_to_areas'][name][0] == part_index and len(
                        self.global_view['agents_to_areas'][name]) != 2:
                    # dovrebbe tenere solo la pozione attuale
                    # non è più un path ends
                    self.update_ends(self.tokens[part_index]['agents'][name][-1], part_index)
                    self.tokens[part_index]['agents'][name] = path[:1]
                    self.tokens[part_index]['path_ends'].append(tuple(path[0]))
                # se invece l'agente dovrà arrivare in questa partizione, ma attualmente è in frontiera altrove
                elif self.global_view['agents_to_areas'][name][0] != part_index and len(
                        self.global_view['agents_to_areas'][name]) == 2:
                    self.update_ends(self.tokens[part_index]['agents'][name][-1], part_index)
                    self.tokens[part_index]['agents'].pop(name)
                    #self.global_view['agents_to_areas'][name] = self.global_view['agents_to_areas'][name][:1]
                    self.global_view['agents_to_areas'][name] = []
                    self.global_view['agents_to_areas'][name].append(self.find_partition(path[0]))
                    self.tokens[self.global_view['agents_to_areas'][name][0]]['agents'][name] = path[:1]
                    self.tokens[self.global_view['agents_to_areas'][name][0]]['path_ends'].append(tuple(path[0]))

    def on_a_frontier(self, agent_pos, actual_part):
        frontiers = self.tokens[actual_part]['own_frontiers']
        for part, front in frontiers.items():
            for f in front:
                if tuple(agent_pos) == f.entry_cell:
                    return part

        return -1

    #probailmente si può gestire diversamente
    def update_non_task_endpoints(self):
        self.global_view['occupied_non_task_endpoints'] = set()
        for part in range(self.number_of_areas):
            for agent_pos in self.tokens[part]['agents'].values():
                if tuple(agent_pos[0]) in self.non_task_endpoints:
                    self.global_view['occupied_non_task_endpoints'].add(tuple(agent_pos[0]))

    def remove_task_from_agents(self, agent_name):
        print('Rimozione del task dall\'', agent_name)
        task_name = self.global_view['pre_assignment_agents_tasks'][agent_name]['task_name']
        if task_name != 'safe_idle':
            self.global_view['discarded_tasks_for_agents'][agent_name].append(task_name)
        self.global_view['tasks'][task_name] = \
            [self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
             self.global_view['pre_assignment_agents_tasks'][agent_name]['goal']]
        self.global_view['pre_assignment_agents_tasks'].pop(agent_name)
        self.global_view['abstract_to_loc1'][agent_name] = []
        self.global_view['abstract_to_loc2'][agent_name] = []

    def pickup_in_partition(self, agent_name, agent_pos, pickup_position, all_idle_agents, part_index, time_start=0):
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
            closest_frontier = self.get_closest_frontier(pickup_position, frontiers_to_next_part)
            planned = self.compute_real_path_double(agent_name, agent_pos, pickup_position, closest_frontier.start_pos,
                                                    all_idle_agents, part_index, time_start)
        if not planned:
            self.remove_task_from_agents(agent_name)

        return planned

    def update_A_star_stats(self):
        self.sommaEspansioniAmaxTimestep += max(self.espansioniAstarXpart)
        self.espansioniAstarXpart = [0] * self.number_of_areas

    def time_forward(self):
        self.update_completed_tasks()
        self.collect_new_tasks()
        self.assign_tasks()

        # vedo gli agent con pre assegnamento, ma non hanno ancora un path assegnato
        #IN FUTURO PIANIFICANO PER PRIMI GLI AGENTI ALLA FRONTIERA
        agents_to_plan = self.get_agents_to_plan()
        agents_to_REplan = {}
        while len(agents_to_plan) > 0 or len(agents_to_REplan) > 0:

            if len(agents_to_REplan) > 0:
                agent_name = random.choice(list(agents_to_REplan.keys()))
                agent_pos = agents_to_REplan.pop(agent_name)[0]
                if agent_name in agents_to_plan.keys():
                    agents_to_plan.pop(agent_name)
            else:
                agent_name = random.choice(list(agents_to_plan.keys()))
                agent_pos = agents_to_plan.pop(agent_name)[0]

            agent_partition = self.global_view['agents_to_areas'][agent_name][0]

            local_idle_agents = self.tokens[agent_partition]['agents'].copy()
            local_idle_agents.pop(agent_name)

            #se non ha abstract path(s) calcolo
            if len(self.global_view['abstract_to_loc1'][agent_name]) == 0 and \
                    len(self.global_view['abstract_to_loc2'][agent_name]) == 0:
                self.compute_abstract_path(agent_pos,
                                           self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                           agent_name, 1)
                self.compute_abstract_path(self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                           self.global_view['pre_assignment_agents_tasks'][agent_name]['goal'],
                                           agent_name, 2)

            #-----------------------------PATH REALI--------------------------------
            if len(self.global_view['abstract_to_loc1'][agent_name]) > 1:
                on_frontier = self.on_a_frontier(agent_pos, agent_partition)
                if on_frontier != -1:
                    agents_to_REplan = self.migrazione(agent_name, agent_pos, agent_partition, on_frontier, 1,
                                                       agents_to_plan)
                else:
                    self.go_to_frontier(agent_name, agent_pos, local_idle_agents, agent_partition,
                                        self.global_view['abstract_to_loc1'][agent_name][1])

            #il pickup è nell'area in cui mi trovo
            elif len(self.global_view['abstract_to_loc1'][agent_name]) == 1:
                self.pickup_in_partition(agent_name, agent_pos,
                                         self.global_view['pre_assignment_agents_tasks'][agent_name]['start'],
                                         local_idle_agents, agent_partition)

            #da qui in giù abstract path 1 è vuoto quindi devo andare al delivery o al non task endpoint
            elif len(self.global_view['abstract_to_loc2'][agent_name]) > 1:
                on_frontier = self.on_a_frontier(agent_pos, agent_partition)
                if on_frontier != -1:
                    agents_to_REplan = self.migrazione(agent_name, agent_pos, agent_partition, on_frontier, 2,
                                                       agents_to_plan)
                else:
                    self.go_to_frontier(agent_name, agent_pos, local_idle_agents, agent_partition,
                                        self.global_view['abstract_to_loc2'][agent_name][1])

            elif len(self.global_view['abstract_to_loc2'][agent_name]) == 1:
                self.compute_real_path_single(agent_name, agent_pos,
                                              self.global_view['pre_assignment_agents_tasks'][agent_name]['goal'],
                                              local_idle_agents, agent_partition)
            else:
                print("Entrambi gli abstact path sono vuoti, errore? " + agent_name)

        #self.update_non_task_endpoints()
        self.update_A_star_stats()
