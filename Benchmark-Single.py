import argparse
import yaml
import json
import os
# import random

import RoothPath
from Simulation.tasks_maker import *
from Simulation.TP import TokenPassing
from Simulation.simulation import Simulation

def parameters(seed):
    random.seed(seed)
    #random_seed = seed
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=500, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)  # default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true', default=False)
    args = parser.parse_args()

    with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    args.param = os.path.join(RoothPath.get_root(), os.path.join(config['input_path'], config['input_name']))
    args.output = os.path.join(RoothPath.get_root(), 'output.yaml')

    # Read from input file, metto tutto dentro param
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
    frontiers = param['map']['frontiers']


    tasks = gen_tasks(param['map']['pickup_locations'], param['map']['delivery_locations'],
                                             param['n_tasks'], param['task_freq'], seed)
    param['tasks'] = tasks

    # with open('Comparisons/seeds2.txt', 'a') as file:
    #     file.write(str(seed) + " ")

    return tasks, agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, goal_endpoints, frontiers, args.a_star_max_iter


def print_comparison(version, completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev,cbs_calls,
                     index_run, cbs_calls_recharge, random_seed=1234, file_name='Comparisons/Comp1/test3.txt', avg_espansioniA=0):
    with open(file_name, 'a') as file:
        file.write("\n\n" + str(index_run) + " " + version + " " + str(random_seed) + "\n")
        s_completed_tasks = "Number of completed tasks: ", completed_tasks, "/", n_tasks
        s_dead_agents = "Number of dead agents: ", dead_agents
        s_makespan = "Makespan: ", makespan

        s_average_service_time = "Average service time: ", average_service_time
        s_std_dev = "Standard deviation: ", std_dev
        s_cbs_calls_recharge = "Chiamate a CBS per stazioni di ricarica: ", cbs_calls_recharge
        s_cbs_calls = "Chiamate a CBS: ", cbs_calls
        s_avg_espansioniA = "Espansioni medie di A*: ", avg_espansioniA

        file.write(str(s_completed_tasks) + '\n' + str(s_dead_agents) + '\n' + str(s_makespan) + '\n' + str(
            s_average_service_time) + '\n' + str(s_std_dev) + '\n' + str(s_cbs_calls_recharge) + '\n' + str(s_cbs_calls) + '\n' + str(s_avg_espansioniA))


def single_run(index_run, random_seed, file_name):
    tasks, agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, goal_endpoints, frontiers, max_iter = parameters(random_seed)

    # Simulate
    simulation = Simulation(tasks, agents)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, simulation,
                      goal_endpoints, frontiers, max_iter)
    while tp.get_completed_tasks() != len(tasks) and simulation.get_time() < 10000:
        simulation.time_forward(tp)

    completed_tasks = tp.get_completed_tasks()
    n_agents = len(agents)
    n_tasks = len(tasks)
    makespan = simulation.get_time()

    delta_times = []
    for a in tp.get_completed_tasks():
        delta_times.append(tp.get_completed_tasks_times()[a] - tp.get_completed_tasks_times()['start_tasks_times'][a])

    service_time = sum(delta_times)
    average_service_time = service_time / len(tp.get_completed_tasks_times())
    variance = sum((x - average_service_time) ** 2 for x in delta_times) / len(tp.get_completed_tasks_times())
    std_dev = math.sqrt(variance)

    Astar_calls = tp.get_Astar_calls()
    #avg_espansioniA = tp.get_avg_espansioniA()
    Astar_total_expansions = tp.get_total_expansions()
    Astar_exp_sum_max_per_timestep = tp.get_exp_sum_max_per_timestep()


    print_comparison("VersioneQueue", completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev,cbs_calls,
                     index_run, cbs_calls_recharge, random_seed, file_name, avg_espansioniA)

    return completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev, cbs_calls, cbs_calls_recharge, avg_espansioniA  # , completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2, cbs_calls_recharge2


if __name__ == '__main__':

    run_complete1 = 0
    sum_completed_tasks1 = 0
    sum_makespan1 = 0
    sum_service_time1 = 0
    sum_std_dev1 = 0
    sum_cbs_calls1 = 0
    sum_cbs_calls_recharge1 = 0
    sum_dead_agents1 = 0
    sum_avg_espansioniA1 = 0

    file_name = 'Comparisons/TP/2.txt'

    with open('Comparisons/seeds1.txt', 'r') as file:
        # inserisci ogni riga in una lista
        seeds = file.read()
    seeds = seeds.split(" ")

    for i in range(20):
        print("Run numero: ", i + 1)
        # random_seed = random.randint(0, 100000)
        random_seed = int(seeds[i])
        completed_tasks, n_tasks, dead_agents, makespan, average_service_time, std_dev, cbs_calls, cbs_calls_recharge, avg_espansioniA = single_run(
            i, random_seed, file_name)

        if completed_tasks == n_tasks:
            run_complete1 += 1
            sum_makespan1 += makespan
            sum_service_time1 += average_service_time
            sum_std_dev1 += std_dev
            sum_cbs_calls1 += cbs_calls
            sum_cbs_calls_recharge1 += cbs_calls_recharge
            sum_avg_espansioniA1 += avg_espansioniA

        sum_completed_tasks1 += completed_tasks
        sum_dead_agents1 += dead_agents

    # print("\nVersioneChange")
    print("Numero di run completate: ", run_complete1)
    print("Numero medio di task completati: ", sum_completed_tasks1 / 20)
    print("Numero medio di agenti morti: ", sum_dead_agents1 / 20)
    try:
        print("Makespan medio: ", sum_makespan1 / run_complete1)
        print("Tempo medio di servizio: ", sum_service_time1 / run_complete1)
        print("Deviazione standard media: ", sum_std_dev1 / run_complete1)
        print("Chiamate a CBS per stazioni di ricarica: ", sum_cbs_calls_recharge1 / run_complete1)
        print("Chiamate a CBS totali: ", sum_cbs_calls1 / run_complete1)
        print("Espansioni medie di A*: ", sum_avg_espansioniA1 / run_complete1)
    except:
        print("0 run completate")

    with open(file_name, 'a') as file:
        file.write("\n\n" + "Numero di run completate: " + str(run_complete1) + "\n")
        file.write("Numero medio di task completati: " + str(sum_completed_tasks1 / 20) + "\n")
        file.write("Numero medio di agenti morti: " + str(sum_dead_agents1 / 20) + "\n")
        try:
            file.write("Makespan medio: " + str(sum_makespan1 / run_complete1) + "\n")
            file.write("Tempo medio di servizio: " + str(sum_service_time1 / run_complete1) + "\n")
            file.write("Deviazione standard media: " + str(sum_std_dev1 / run_complete1) + "\n")
            file.write("Chiamate a CBS per stazioni di ricarica: " + str(sum_cbs_calls_recharge1 / run_complete1) + "\n")
            file.write("Chiamate a CBS totali: " + str(sum_cbs_calls1 / run_complete1) + "\n")
            file.write("Espansioni medie di A*: " + str(sum_avg_espansioniA1 / run_complete1) + "\n")
        except:
            file.write("0 run completate")

