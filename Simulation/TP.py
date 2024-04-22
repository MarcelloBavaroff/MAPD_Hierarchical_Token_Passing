from math import fabs
import random
from Simulation.CBS.cbs import CBS, Environment


class TokenPassing(object):
    def __init__(self, agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, simulation, goal_endpoints, a_star_max_iter=4000):
        random.seed(1234)
        self.agents = agents
        self.dimensions = dimensions
        self.obstacles = set(obstacles)
        self.non_task_endpoints = non_task_endpoints
        self.number_of_areas = number_of_areas
        if len(agents) > len(non_task_endpoints):
            print('There are more agents than non task endpoints, instance is not well-formed.')
            exit(1)

        self.tokens = []
        self.simulation = simulation
        self.a_star_max_iter = a_star_max_iter
        self.chiamateCBS = 0
        self.chiamateCBS_recharge = 0
        self.sommaEspansioniA = 0
        self.goal_endpoints = goal_endpoints
        self.init_tokens(partitions)
        #vedi sotto

    #initialize a single token
    def init_token(self, index=0, partitions=None):
        self.tokens[index]['agents'] = {}
        self.tokens[index]['tasks'] = {}
        self.tokens[index]['start_tasks_times'] = {}
        self.tokens[index]['completed_tasks_times'] = {}
        self.tokens[index]['agents_to_tasks'] = {}
        self.tokens[index]['completed_tasks'] = 0
        self.tokens[index]['path_ends'] = set()
        self.tokens[index]['occupied_non_task_endpoints'] = set()
        for t in self.simulation.get_new_tasks():
            self.tokens[index]['tasks'][t['task_name']] = [t['start'], t['goal']]
            self.tokens[index]['start_tasks_times'][t['task_name']] = self.simulation.get_time()

        for a in self.agents:
            self.tokens[index]['agents'][a['name']] = [a['start']]

            if tuple(a['start']) in self.non_task_endpoints:
                self.tokens[index]['occupied_non_task_endpoints'].add(tuple(a['start']))
            else:
                self.tokens[index]['path_ends'].add(tuple(a['start']))

    #initialize all tokens
    def init_tokens(self, partitions):
        for t in range(self.number_of_areas):
            self.tokens.append({})
            self.init_token(t, partitions[t])

    #in teoria agenti in idle hanno il path verso la loro posizione attuale
    def get_idle_agents(self):
        agents = {}
        for name, path in self.tokens[0]['agents'].items():
            if len(path) == 1:
                agents[name] = path
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

    def get_moving_obstacles_agents(self, agents, time_start):
        obstacles = {}
        for name, path in agents.items(): #agents.item ritorna un dizionario con nome agente e coordinate dello stesso
            if len(path) > time_start and len(path) > 1:
                for i in range(time_start, len(path)):
                    k = i - time_start
                    obstacles[(path[i][0], path[i][1], k)] = name
                    if i == len(path) - 1:
                        obstacles[(path[i][0], path[i][1], -k)] = name
        return obstacles

    # def get_idle_obstacles_agents(self, agents_paths, time_start):
    #     obstacles = set()
    #     for path in agents_paths:
    #         if len(path) == 1:
    #             obstacles.add((path[0][0], path[0][1]))
    #         if 1 < len(path) <= time_start:
    #             obstacles.add((path[-1][0], path[-1][1]))
    #     return obstacles
    def get_idle_obstacles_agents(self, agents_paths, time_start, agent_name):

        obstacles = set()
        for g in self.goal_endpoints:
            obstacles.add(tuple(g))

        for agent in agents_paths:

            if agent != agent_name:
                # quelli nelle stazioni non li segno come ostacoli and tuple(path[0]) not in charging_stations_pos
                if len(agents_paths[agent]) == 1:
                    obstacles.add((agents_paths[agent][0][0], agents_paths[agent][0][1]))
                # presumo agenti che finiranno il loro percorso e si fermeranno? Quindi metto ultima
                # loro posizione
                if 1 < len(agents_paths[agent]) <= time_start:
                    obstacles.add((agents_paths[agent][-1][0], agents_paths[agent][-1][1]))
        return obstacles

    def check_safe_idle(self, agent_pos):
        for task_name, task in self.tokens[0]['tasks'].items():
            if tuple(task[0]) == tuple(agent_pos) or tuple(task[1]) == tuple(agent_pos):
                return False
        for start_goal in self.get_agents_to_tasks_starts_goals():
            if tuple(start_goal) == tuple(agent_pos):
                return False
        return True

    def get_closest_non_task_endpoint(self, agent_pos):
        dist = -1
        res = -1
        for endpoint in self.non_task_endpoints:
            if endpoint not in self.tokens[0]['occupied_non_task_endpoints']:
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

    def update_ends(self, agent_pos):
        if tuple(agent_pos) in self.tokens[0]['path_ends']:
            self.tokens[0]['path_ends'].remove(tuple(agent_pos))
        elif tuple(agent_pos) in self.tokens[0]['occupied_non_task_endpoints']:
            self.tokens[0]['occupied_non_task_endpoints'].remove(tuple(agent_pos))

    def get_agents_to_tasks_goals(self):
        goals = set()
        for el in self.tokens[0]['agents_to_tasks'].values():
            goals.add(tuple(el['goal']))
        return goals

    def get_agents_to_tasks_starts_goals(self):
        starts_goals = set()
        for el in self.tokens[0]['agents_to_tasks'].values():
            starts_goals.add(tuple(el['goal']))
            starts_goals.add(tuple(el['start']))
        return starts_goals

    def get_completed_tasks(self):
        return self.tokens[0]['completed_tasks']

    def get_completed_tasks_times(self):
        return self.tokens[0]['completed_tasks_times']


    def get_token(self):
        return self.tokens[0]

    def search(self, cbs, agent_name, moving_obstacles_agents):

        path = cbs.search()
        return path

    def go_to_closest_non_task_endpoint(self, agent_name, agent_pos, all_idle_agents):
        closest_non_task_endpoint = self.get_closest_non_task_endpoint(agent_pos)
        moving_obstacles_agents = self.get_moving_obstacles_agents(self.tokens[0]['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_non_task_endpoint}
        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents, moving_obstacles_agents,
                          a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_non_task_endpoint = self.search(cbs, agent_name, moving_obstacles_agents)
        if not path_to_non_task_endpoint:
            print("Solution to non-task endpoint not found for agent", agent_name, " instance is not well-formed.")

        else:
            print('No available task for agent', agent_name, ' moving to safe idling position...')
            self.update_ends(agent_pos)
            self.tokens[0]['occupied_non_task_endpoints'].add(tuple(closest_non_task_endpoint))
            self.tokens[0]['agents_to_tasks'][agent_name] = {'task_name': 'safe_idle', 'start': agent_pos,
                                                         'goal': closest_non_task_endpoint, 'predicted_cost': 0}
            self.tokens[0]['agents'][agent_name] = []
            for el in path_to_non_task_endpoint[agent_name]:
                self.tokens[0]['agents'][agent_name].append([el['x'], el['y']])

    def collect_new_tasks(self):
        for t in self.simulation.get_new_tasks():
            self.tokens[0]['tasks'][t['task_name']] = [t['start'], t['goal']]
            self.tokens[0]['start_tasks_times'][t['task_name']] = self.simulation.get_time()

    def update_copleted_tasks(self):
        # Update completed tasks
        for agent_name in self.tokens[0]['agents']:
            # pos = posizione attuale agente
            pos = self.simulation.actual_paths[agent_name][-1]

            # ---------------------CASO AGENTE ARRIVATO------------------
            # se agente assegnato ad un task E le sue coordinate attuali sono = al suo goal
            # E il suo path attuale lungo 1 ed il suo taks non è safe idle
            if agent_name in self.tokens[0]['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.tokens[0]['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.tokens[0]['agents'][agent_name]) == 1 and self.tokens[0]['agents_to_tasks'][agent_name][
                'task_name'] != 'safe_idle':
                self.tokens[0]['completed_tasks'] = self.tokens[0]['completed_tasks'] + 1
                self.tokens[0]['completed_tasks_times'][
                    self.tokens[0]['agents_to_tasks'][agent_name]['task_name']] = self.simulation.get_time()
                self.tokens[0]['agents_to_tasks'].pop(agent_name)
            if agent_name in self.tokens[0]['agents_to_tasks'] and (pos['x'], pos['y']) == tuple(
                    self.tokens[0]['agents_to_tasks'][agent_name]['goal']) \
                    and len(self.tokens[0]['agents'][agent_name]) == 1 and self.tokens[0]['agents_to_tasks'][agent_name][
                'task_name'] == 'safe_idle':
                self.tokens[0]['agents_to_tasks'].pop(agent_name)

    def find_available_tasks(self, agent_pos):
        available_tasks = {}
        for task_name, task in self.tokens[0]['tasks'].items():
            # se inizio e fine task non in path ends degli agenti (meno me) AND nemmeno in goals
            if tuple(task[0]) not in self.tokens[0]['path_ends'].difference({tuple(agent_pos)}) and tuple(
                    task[1]) not in self.tokens[0]['path_ends'].difference({tuple(agent_pos)}) \
                    and tuple(task[0]) not in self.get_agents_to_tasks_goals() and tuple(
                task[1]) not in self.get_agents_to_tasks_goals():
                available_tasks[task_name] = task
        return available_tasks

    def choose_task(self, agent_name, agent_pos, available_tasks, all_idle_agents):
        closest_task_name = self.get_closest_task_name(available_tasks, agent_pos)
        closest_task = available_tasks[closest_task_name]


        moving_obstacles_agents = self.get_moving_obstacles_agents(self.tokens[0]['agents'], 0)
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), 0)
        agent = {'name': agent_name, 'start': agent_pos, 'goal': closest_task[0]}

        env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                          moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
        cbs = CBS(env)
        path_to_task_start = self.search(cbs, agent_name, moving_obstacles_agents)
        if not path_to_task_start:
            print("Solution not found to task start for agent", agent_name, " idling at current position...")

        else:
            print("Solution found to task start for agent", agent_name, " searching solution to task goal...")
            cost1 = env.compute_solution_cost(path_to_task_start)
            # Use cost - 1 because idle cost is 1
            moving_obstacles_agents = self.get_moving_obstacles_agents(self.tokens[0]['agents'], cost1 - 1)
            idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents.values(), cost1 - 1)
            agent = {'name': agent_name, 'start': closest_task[0], 'goal': closest_task[1]}
            env = Environment(self.dimensions, [agent], self.obstacles | idle_obstacles_agents,
                              moving_obstacles_agents, a_star_max_iter=self.a_star_max_iter)
            cbs = CBS(env)
            path_to_task_goal = self.search(cbs, agent_name, moving_obstacles_agents)
            if not path_to_task_goal:
                print("Solution not found to task goal for agent", agent_name, " idling at current position...")

            else:
                print("Solution found to task goal for agent", agent_name, " doing task...")
                cost2 = env.compute_solution_cost(path_to_task_goal)
                if agent_name not in self.tokens[0]['agents_to_tasks']:
                    self.tokens[0]['tasks'].pop(closest_task_name)
                    task = available_tasks.pop(closest_task_name)
                else:
                    task = closest_task

                last_step = path_to_task_goal[agent_name][-1]
                self.update_ends(agent_pos)
                self.tokens[0]['path_ends'].add(tuple([last_step['x'], last_step['y']]))
                self.tokens[0]['agents_to_tasks'][agent_name] = {'task_name': closest_task_name, 'start': task[0],
                                                                 'goal': task[1], 'predicted_cost': cost1 + cost2}
                self.tokens[0]['agents'][agent_name] = []
                for el in path_to_task_start[agent_name]:
                    self.tokens[0]['agents'][agent_name].append([el['x'], el['y']])
                # Don't repeat twice same step, elimino ultimo elemento
                self.tokens[0]['agents'][agent_name] = self.tokens[0]['agents'][agent_name][:-1]
                for el in path_to_task_goal[agent_name]:
                    self.tokens[0]['agents'][agent_name].append([el['x'], el['y']])

    def compute_real_path(self, agent_name, agent_pos, closest_task, closest_task_name, all_idle_agents,
                          available_tasks):
        idle_obstacles_agents = self.get_idle_obstacles_agents(all_idle_agents, 0, agent_name)
        idle_obstacles_agents |= set(self.non_task_endpoints)
        idle_obstacles_agents = idle_obstacles_agents - {tuple(agent_pos), tuple(closest_task[1])}

    def time_forward(self):

        self.update_copleted_tasks()
        self.collect_new_tasks()
        idle_agents = self.get_idle_agents()

        while len(idle_agents) > 0:
            agent_name = random.choice(list(idle_agents.keys()))
            all_idle_agents = self.tokens[0]['agents'].copy()
            all_idle_agents.pop(agent_name)
            agent_pos = idle_agents.pop(agent_name)[0]
            available_tasks = self.find_available_tasks(agent_pos)

            if len(available_tasks) > 0:

                assigned = self.choose_task(agent_name, agent_pos, available_tasks, all_idle_agents)


            #righe 13-14 alg
            elif self.check_safe_idle(agent_pos):
                print('No available tasks for agent', agent_name, ' idling at current position...')
            #righe 15-16 alg
            else:
                self.go_to_closest_non_task_endpoint(agent_name, agent_pos, all_idle_agents)

