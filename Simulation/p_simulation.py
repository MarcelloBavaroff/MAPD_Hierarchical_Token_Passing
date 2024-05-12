import time
from Simulation.p_TP import TokenPassing
from Simulation.tasks_maker import *


class Simulation(object):
    def __init__(self, tasks, agents):
        self.tasks = tasks
        self.agents = agents
        self.time = 0
        self.start_times = [] #inizio dei task dei robot
        self.agents_pos_now = set()
        self.agents_moved = set()
        self.actual_paths = {}
        self.algo_time = 0
        self.initialize_simulation()

    def initialize_simulation(self):
        for t in self.tasks:
            self.start_times.append(t['start_time'])
        for agent in self.agents:
            #x e y del path sono presi da 'pickup' dell'agente (posizione 0 e 1)
            self.actual_paths[agent['name']] = [{'t': 0, 'x': agent['start'][0], 'y': agent['start'][1]}]
    def update_abstract_paths(self, agent_name, old_pos, new_pos, gb, algorithm):
        if old_pos == new_pos:
            return
        elif len(gb['abstract_to_loc1'][agent_name]) > 0:
            if new_pos == tuple(gb['pre_assignment_agents_tasks'][agent_name]['start']):
                gb['abstract_to_loc1'][agent_name] = []
            else:
                old_part = algorithm.find_partition(old_pos)
                new_part = algorithm.find_partition(new_pos)
                if old_part != new_part:
                    gb['abstract_to_loc1'][agent_name] = gb['abstract_to_loc1'][agent_name][1:]
        else:
            if new_pos == tuple(gb['pre_assignment_agents_tasks'][agent_name]['goal']):
                gb['abstract_to_loc2'][agent_name] = []
            else:
                old_part = algorithm.find_partition(old_pos)
                new_part = algorithm.find_partition(new_pos)
                if old_part != new_part:
                    gb['abstract_to_loc2'][agent_name] = gb['abstract_to_loc2'][agent_name][1:]
    def check_partition_change(self, agent_name, agent_pos, algorithm, actual_part):
        changed = False
        for i in range(algorithm.get_number_of_areas()):
            if agent_name in algorithm.get_token(i)['agents'] and i != actual_part:
                changed = True
                break

    def update_non_task_endpoints(self, old_pos, new_pos, gb):
        if old_pos == new_pos:
            return
        if old_pos in gb['occupied_non_task_endpoints']:
            gb['occupied_non_task_endpoints'].remove(old_pos)

    #controlla se gli agenti effettivamente si muovono
    #agents_to_move dovrebbe essere una lista
    def handle_loops(self, agents_to_move, algorithm, part_index):
        actual_pos = []
        for a in agents_to_move:
            actual_pos.append(tuple(algorithm.get_token(part_index)['agents'][a['name']][0]))

        correspondences = 0
        for a in agents_to_move:
            # next_pos è una lista, non una tupla
            next_pos = algorithm.get_token(part_index)['agents'][a['name']][1]
            if tuple(next_pos) in actual_pos:
                correspondences += 1
            else:
                break
        if correspondences == len(agents_to_move):
            return True
        else:
            return False

    def handle_agents_lenPath1(self, agents_to_move, algorithm, gb):
        for agent in agents_to_move:
            #ultimo elemento della lista dei path
            current_agent_pos = self.actual_paths[agent['name']][-1]
            #aggiorno posizione attuale agenti
            self.agents_pos_now.add(tuple([current_agent_pos['x'], current_agent_pos['y']]))
            #lunghezza del path dell'agente considerato

            partition = gb['agents_to_areas'][agent['name']][0]
            #così non cancello l'ultimo elemento e conosco la posizione attuale dell'agente (se è in più token allora non è il caso)
            if len(algorithm.get_token(partition)['agents'][agent['name']]) == 1 and len(gb['agents_to_areas'][agent['name']]) == 1:
                self.agents_moved.add(agent['name'])
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})
                #capire come sfruttare la cosa per il cambio di frontiera
            elif len(algorithm.get_token(partition)['agents'][agent['name']]) == 1 and len(gb['agents_to_areas'][agent['name']]) > 1:
                #tolgo dal vecchio token
                algorithm.get_token(partition)['agents'].pop(agent['name'])
                #cambio l'area di appartenenza dell'agente
                algorithm.get_global_view()['agents_to_areas'][agent['name']] = algorithm.get_global_view()['agents_to_areas'][agent['name']][1:]

    def update_actual_paths(self, agent, algorithm, x_new, y_new, current_agent_pos, gb):
        self.agents_moved.add(agent['name'])
        # dico che c'è un agente in questa posizione
        self.agents_pos_now.remove(tuple([current_agent_pos['x'], current_agent_pos['y']]))
        self.agents_pos_now.add(tuple([x_new, y_new]))

        for i in range(len(gb['agents_to_areas'][agent['name']])):
            partition = gb['agents_to_areas'][agent['name']][i]
            algorithm.get_token(partition)['agents'][agent['name']] = \
                algorithm.get_token(partition)['agents'][agent['name']][1:]

        self.update_abstract_paths(agent['name'], tuple([current_agent_pos['x'], current_agent_pos['y']]),
                                   tuple([x_new, y_new]), gb, algorithm)
        # aggiorno qui i non te e non in TP
        self.update_non_task_endpoints(tuple([current_agent_pos['x'], current_agent_pos['y']]), tuple([x_new, y_new]),
                                       gb)
        # aggiorno il path dell'agente
        self.actual_paths[agent['name']].append({'t': self.time, 'x': x_new, 'y': y_new})

    #viene chiamata per simulare un singolo timestep in avanti
    def time_forward(self, algorithm):
        self.time = self.time + 1
        print('Time:', self.time, " Completed tasks:", len(algorithm.get_completed_tasks()))
        start_time = time.time()
        algorithm.time_forward()
        self.algo_time += time.time() - start_time
        self.agents_pos_now = set()
        self.agents_moved = set()

        agents_to_move = self.agents
        random.shuffle(agents_to_move)

        gb = algorithm.get_global_view()
        self.handle_agents_lenPath1(agents_to_move, algorithm, gb)

        # Check moving agents doesn't collide with others
        agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]
        moved_this_step = -1
        while moved_this_step != 0:
            moved_this_step = 0
            
            for agent in agents_to_move:
                current_agent_pos = self.actual_paths[agent['name']][-1]
                partition = gb['agents_to_areas'][agent['name']][0]

                if len(algorithm.get_token(partition)['agents'][agent['name']]) > 1:

                    x_new = algorithm.get_token(partition)['agents'][agent['name']][1][0]
                    y_new = algorithm.get_token(partition)['agents'][agent['name']][1][1]
                    # se non corrisponde alla posizione attuale di un altro agente
                    if tuple([x_new, y_new]) not in self.agents_pos_now or \
                            tuple([x_new, y_new]) == tuple(tuple([current_agent_pos['x'], current_agent_pos['y']])):

                        self.update_actual_paths(agent, algorithm, x_new, y_new, current_agent_pos, gb)
                        moved_this_step = moved_this_step + 1

            agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]

        #ho un dizionario di agenti bloccati divisi per partizione
        blocked_agents_per_partition = {}
        for i in range(algorithm.get_number_of_areas()):
            blocked_agents_per_partition[i] = []
            for agent in agents_to_move:
                if agent['name'] in algorithm.get_token(i)['agents']:
                    blocked_agents_per_partition[i].append(agent)

        for i in range(algorithm.get_number_of_areas()):
            if len(blocked_agents_per_partition[i]) > 3:
                if self.handle_loops(blocked_agents_per_partition[i], algorithm, i):
                    for agent in agents_to_move:
                        current_agent_pos = self.actual_paths[agent['name']][-1]
                        if len(algorithm.get_token(i)['agents'][agent['name']]) > 1:
                            # accedo alla tupla della posizione
                            x_new = algorithm.get_token(i)['agents'][agent['name']][1][0]
                            y_new = algorithm.get_token(i)['agents'][agent['name']][1][1]

                            self.update_actual_paths(agent, algorithm, x_new, y_new, current_agent_pos, gb)
                else:
                    print("Qualcosa non va nella gestione del ciclo di agenti")

    def get_time(self):
        return self.time

    def get_algo_time(self):
        return self.algo_time

    def get_actual_paths(self):
        return self.actual_paths

    def get_new_tasks(self):
        new = []
        for t in self.tasks:
            if t['start_time'] == self.time:
                new.append(t)
        return new

