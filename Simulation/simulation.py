import argparse
import yaml
import json
import os
import time
from Simulation.TP import TokenPassing
import RoothPath
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
            if new_pos == gb['pre_assignment_agents_tasks'][agent_name]['start']:
                gb['abstract_to_loc1'][agent_name] = []
            else:
                old_part = algorithm.find_partition(old_pos)
                new_part = algorithm.find_partition(new_pos)
                if old_part != new_part:
                    gb['abstract_to_loc1'][agent_name] = gb['abstract_to_loc1'][agent_name][1:]
        else:
            if new_pos == gb['preassigned_tasks'][agent_name]['goal']:
                gb['abstract_to_loc2'][agent_name] = []
            else:
                old_part = algorithm.find_partition(old_pos)
                new_part = algorithm.find_partition(new_pos)
                if old_part != new_part:
                    gb['abstract_to_loc2'][agent_name] = gb['abstract_to_loc2'][agent_name][1:]





    #viene chiamata per simulare un singolo timestep in avanti
    def time_forward(self, algorithm):
        self.time = self.time + 1
        print('Time:', self.time)
        start_time = time.time()
        algorithm.time_forward()
        self.algo_time += time.time() - start_time
        self.agents_pos_now = set()
        self.agents_moved = set()

        agents_to_move = self.agents
        random.shuffle(agents_to_move)

        gb = algorithm.get_global_view()

        for agent in agents_to_move:
            #ultimo elemento della lista dei path
            current_agent_pos = self.actual_paths[agent['name']][-1]
            #aggiorno posizione attuale agenti
            self.agents_pos_now.add(tuple([current_agent_pos['x'], current_agent_pos['y']]))
            #lunghezza del path dell'agente considerato

            partition = gb['agents_to_areas'][agent['name']]
            if len(algorithm.get_token(partition)['agents'][agent['name']]) == 1:
                self.agents_moved.add(agent['name'])
                self.actual_paths[agent['name']].append(
                    {'t': self.time, 'x': current_agent_pos['x'], 'y': current_agent_pos['y']})

        # Check moving agents doesn't collide with others
        agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]
        moved_this_step = -1
        while moved_this_step != 0:
            moved_this_step = 0
            
            for agent in agents_to_move:
                current_agent_pos = self.actual_paths[agent['name']][-1]
                partition = gb['agents_to_areas'][agent['name']]

                if len(algorithm.get_token(partition)['agents'][agent['name']]) > 1:

                    x_new = algorithm.get_token(partition)['agents'][agent['name']][1][0]
                    y_new = algorithm.get_token(partition)['agents'][agent['name']][1][1]
                    # se non corrisponde alla posizione attuale di un altro agente
                    if tuple([x_new, y_new]) not in self.agents_pos_now or \
                            tuple([x_new, y_new]) == tuple(tuple([current_agent_pos['x'], current_agent_pos['y']])):
                        self.agents_moved.add(agent['name'])
                        # dico che c'è un agente in questa posizione
                        self.agents_pos_now.remove(tuple([current_agent_pos['x'], current_agent_pos['y']]))
                        self.agents_pos_now.add(tuple([x_new, y_new]))
                        moved_this_step = moved_this_step + 1

                        # cancello il primo
                        algorithm.get_token(partition)['agents'][agent['name']] = algorithm.get_token(partition)['agents'][agent['name']][1:]
                        self.update_abstract_paths(agent['name'], tuple([current_agent_pos['x'], current_agent_pos['y']]) ,tuple([x_new, y_new]), gb, algorithm)
                        # aggiorno il path dell'agente
                        self.actual_paths[agent['name']].append({'t': self.time, 'x': x_new, 'y': y_new})

            agents_to_move = [x for x in agents_to_move if x['name'] not in self.agents_moved]

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

