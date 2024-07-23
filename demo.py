import argparse
import subprocess
import sys

import yaml
import json
import os
from Simulation.p_TP import TokenPassing
import RoothPath
from Simulation.tasks_maker import *
from Simulation.p_simulation import Simulation
import ast
from Utils.Print_Matrix import PrintMatrix

def read_tasks():
    data_list = []
    with open('LastRun/test', 'r') as file:
        for line in file:
            try:
                # Valuta la stringa come un dizionario Python
                line_data = ast.literal_eval(line.strip())

                # Verifica che il dizionario abbia i campi richiesti
                if all(key in line_data for key in ['start_time', 'pickup', 'delivery', 'task_name']):
                    data_list.append(line_data)
                else:
                    print(f"Errore: La riga '{line.strip()}' non ha tutti i campi richiesti.")
            except SyntaxError:
                print(f"Errore nella lettura della riga: {line.strip()}")

    return data_list

if __name__ == '__main__':
    #random.seed(92332)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=2000, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int) #default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true', default=False)

    args = parser.parse_args()

    with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    args.param = os.path.join(RoothPath.get_root(), os.path.join(config['input_path'], config['input_name']))
    args.output = os.path.join(RoothPath.get_root(), 'output.yaml')

    # Read from input file
    with open(args.param, 'r') as param_file:
        try:
            param = yaml.load(param_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    dimensions = param['map']['dimensions']
    obstacles = param['map']['obstacles']
    non_task_endpoints = param['map']['non_task_endpoints']
    agents = param['agents']
    number_of_areas = param['map']['number_of_areas']
    partitions = param['map']['partitions']
    goal_endpoints = param['map']['delivery_locations']
    goal_endpoints = [tuple(x) for x in goal_endpoints]
    frontiers = param['map']['frontiers']

    if args.not_rand:
        tasks = read_tasks()
    else:
        # Genera i task
        tasks = gen_tasks(param['map']['pickup_locations'], param['map']['delivery_locations'],
                                             param['n_tasks'], param['task_freq'], 92332)
    param['tasks'] = tasks

    with open(args.param + config['visual_postfix'], 'w') as param_file:
        yaml.safe_dump(param, param_file)

    # Simulate
    simulation = Simulation(tasks, agents)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, simulation,
                      goal_endpoints, frontiers, a_star_max_iter=args.a_star_max_iter)
    while len(tp.get_completed_tasks()) != len(tasks) and simulation.time < 30000:
        simulation.time_forward(tp)

    vec = tp.get_vec_areas()
    for i in range(number_of_areas):
        print("Area", i, ":", sum(vec[i]))

    print("Espansioni totali:", tp.get_total_expansions())
    print("Espansioni totali per timestep:", tp.get_exp_sum_max_per_timestep())
    print("Parallel rounds:", tp.get_parallel_rounds())
    print("A* max:", tp.get_max_Astar())

    # parallel_exp = 0
    # #calcolo alternativo del costo
    # for i in range(simulation.time):
    #     parallel_exp = parallel_exp + max(vec[0][i], vec[1][i], vec[2][i], vec[3][i])
    #
    # print("Espansioni totali parallele:", parallel_exp)

    cost = 0
    for path in simulation.actual_paths.values():
        cost = cost + len(path)

    # matrix = PrintMatrix(simulation.actual_paths, dimensions, obstacles, agents, goal_endpoints)
    # matrix.plot_heatmap()
    #
    # output = {'schedule': simulation.actual_paths, 'cost': cost,
    #            'completed_tasks_times': tp.get_completed_tasks_times()}
    # with open(args.output, 'w') as output_yaml:
    #     yaml.safe_dump(output, output_yaml)
    #
    # #legge dal file di output
    # create = [sys.executable, '-m', 'Utils.Visualization.visualize', '-slow_factor', str(args.slow_factor)]
    # subprocess.call(create)
