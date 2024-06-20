import argparse
import yaml
import json
import os
import numpy as np
# import random

import RoothPath
from Simulation.tasks_maker import *
from Simulation.p_TP import TokenPassing
from Simulation.p_simulation import Simulation

def parameters(seed):
    random.seed(seed)
    #random_seed = seed
    parser = argparse.ArgumentParser()
    parser.add_argument('-a_star_max_iter', help='Maximum number of states explored by the low-level algorithm',
                        default=500, type=int)
    parser.add_argument('-slow_factor', help='Slow factor of visualization', default=1, type=int)  # default=1
    parser.add_argument('-not_rand', help='Use if input has fixed tasks and delays', action='store_true', default=False)
    args = parser.parse_args()

    with open(os.path.join(RoothPath.get_root(), 'config2.json'), 'r') as json_file:
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
    goal_endpoints = [tuple(x) for x in goal_endpoints]
    frontiers = param['map']['frontiers']


    tasks = gen_tasks(param['map']['pickup_locations'], param['map']['delivery_locations'],
                                             1000, 0.2, seed)
    param['tasks'] = tasks

    # with open('Comparisons/seeds2.txt', 'a') as file:
    #     file.write(str(seed) + " ")

    return tasks, agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, goal_endpoints, frontiers, args.a_star_max_iter


def print_comparison(version, n_agents, completed_tasks, n_tasks, makespan, average_service_time, std_dev_st, Astar_calls,
                     Astar_total_expansions, Astar_exp_sum_max_per_timestep, index_run, random_seed, file_name):
    with open(file_name, 'a') as file:
        file.write("\n\n" + str(index_run) + " " + version + " " + str(random_seed) + "\n")
        s_n_agents = "Number of agents: ", n_agents
        s_completed_tasks = "Number of completed tasks: ", completed_tasks, "/", n_tasks
        s_makespan = "Makespan: ", makespan
        s_average_service_time = "Average service time: ", average_service_time
        s_std_dev_st = "Standard deviation: ", std_dev_st
        s_Astar_calls = "Total A* calls: ", Astar_calls
        s_Astar_total_expansions = "Total A* expansions: ", Astar_total_expansions
        s_Astar_exp_sum_max_per_timestep = "Espansioni considerando la parallelizzazione: ", Astar_exp_sum_max_per_timestep

        file.write(str(s_n_agents) + '\n' +str(s_completed_tasks) + '\n' + str(s_makespan) + '\n' + str(
            s_average_service_time) + '\n' + str(s_std_dev_st) + '\n' + str(s_Astar_calls) + '\n' + str(s_Astar_total_expansions) + '\n' + str(s_Astar_exp_sum_max_per_timestep))


def single_run(index_run, random_seed, file_name):
    tasks, agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, goal_endpoints, frontiers, max_iter = parameters(random_seed)

    # Simulate
    simulation = Simulation(tasks, agents)
    tp = TokenPassing(agents, dimensions, obstacles, non_task_endpoints, number_of_areas, partitions, simulation,
                      goal_endpoints, frontiers, max_iter)
    while len(tp.get_completed_tasks()) != len(tasks) and simulation.get_time() < 30000:
        simulation.time_forward(tp)

    completed_tasks = len(tp.get_completed_tasks())
    n_agents = len(agents)
    n_tasks = len(tasks)
    makespan = simulation.get_time()

    delta_times = []
    for a in tp.get_completed_tasks():
        delta_times.append(tp.get_completed_tasks_times()[a] - tp.get_start_tasks_times()[a])

    service_time = sum(delta_times)
    average_service_time = service_time / len(tp.get_completed_tasks_times())
    variance = sum((x - average_service_time) ** 2 for x in delta_times) / len(tp.get_completed_tasks_times())
    std_dev_st = math.sqrt(variance)

    Astar_calls = tp.get_Astar_calls()
    #avg_espansioniA = tp.get_avg_espansioniA()
    Astar_total_expansions = tp.get_total_expansions()
    Astar_exp_sum_max_per_timestep = tp.get_exp_sum_max_per_timestep()
    sum_of_costs = tp.get_sum_of_costs()
    maxAstar = tp.get_max_Astar()
    parallel_rounds = tp.get_parallel_rounds()

    print_comparison("Partition", n_agents, completed_tasks, n_tasks, makespan, average_service_time, std_dev_st, Astar_calls,
                     Astar_total_expansions, Astar_exp_sum_max_per_timestep, index_run, random_seed, file_name)

    return completed_tasks, n_tasks, makespan, average_service_time, std_dev_st, Astar_calls, Astar_total_expansions, Astar_exp_sum_max_per_timestep, sum_of_costs, maxAstar, parallel_rounds  # , completed_tasks2, n_tasks2, dead_agents2, makespan2, average_service_time2, cbs_calls2, cbs_calls_recharge2


if __name__ == '__main__':
    Astar_max = 1
    run_complete = 0
    array_completed_tasks = []
    array_makespan = []
    array_avg_service_time = []
    array_std_dev = []
    array_Astar_calls = []
    array_Astar_total_expansions = []
    array_Astar_exp_sum_max_per_timestep = []
    array_sum_of_costs = []
    array_maxAstar = []
    array_parallel_rounds = []

    file_name = 'Comparisons/Stern/21hp11+2a500.txt'

    with open('Comparisons/seeds1.txt', 'r') as file:
        # inserisci ogni riga in una lista
        seeds = file.read()
    seeds = seeds.split(" ")

    for i in range(20):
        print("Run numero: ", i + 1)
        #random_seed = random.randint(0, 100000)
        random_seed = int(seeds[i])
        #try:
        (completed_tasks, n_tasks, makespan, average_service_time,
         std_dev_st, Astar_calls, Astar_total_expansions, Astar_exp_sum_max_per_timestep, sum_of_costs, maxAstar,
         parallel_rounds) = \
            (single_run(i, random_seed, file_name))

        if completed_tasks == n_tasks:
            run_complete += 1
            array_completed_tasks.append(completed_tasks)
            array_makespan.append(makespan)
            array_avg_service_time.append(average_service_time)
            array_std_dev.append(std_dev_st)
            array_Astar_calls.append(Astar_calls)
            array_Astar_total_expansions.append(Astar_total_expansions)
            array_Astar_exp_sum_max_per_timestep.append(Astar_exp_sum_max_per_timestep)
            array_sum_of_costs.append(sum_of_costs)
            array_maxAstar.append(maxAstar)
            array_parallel_rounds.append(parallel_rounds)
        #except:
        #    print("Errore in run ", i)

    avg_completed_tasks = round(np.mean(array_completed_tasks), 2)
    avg_makespan = round(np.mean(array_makespan), 2)
    std_makespan = round(np.std(array_makespan), 2)
    avg_avg_service_time = round(np.mean(array_avg_service_time), 2)
    avg_std_dev = round(np.mean(array_std_dev), 2)
    avg_Astar_calls = round(np.mean(array_Astar_calls), 2)
    avg_Astar_total_expansions = round(np.mean(array_Astar_total_expansions), 2)
    avg_Astar_exp_sum_max_per_timestep = round(np.mean(array_Astar_exp_sum_max_per_timestep), 2)
    avg_sum_of_costs = round(np.mean(array_sum_of_costs), 2)
    avg_maxAstar = round(np.mean(array_maxAstar), 2) * Astar_max
    avg_parallel_rounds = round(np.mean(array_parallel_rounds), 2)

    # print("\nVersioneChange")
    print("Numero di run completate: ", run_complete)
    print("Numero medio di task completati: ", avg_completed_tasks)
    try:
        print("Makespan medio: ", avg_makespan)
        print("Deviazione standard del makespan: ", std_makespan)
        print("Tempo medio di servizio: ", avg_avg_service_time)
        print("Deviazione standard del service time: ", avg_std_dev)
        print("Chiamate a A* medie: ", avg_Astar_calls)
        print("Espansioni totali di A* in media: ", avg_Astar_total_expansions)
        print("Espansioni medie di A* considerando la parallelizzazione: ", avg_Astar_exp_sum_max_per_timestep)
        print("Sum of costs: ", avg_sum_of_costs)
        print("Max Astar: ", avg_maxAstar)
        print("Parallel rounds: ", avg_parallel_rounds)

        excel_string = (str(run_complete) + ";" + str(avg_completed_tasks) + ";" + str(avg_makespan) + "; ;" + str(
            std_makespan) + ";" + str(avg_avg_service_time) + "; ;" + str(avg_std_dev) + ";" + str(
            avg_Astar_calls) + ";" + str(avg_Astar_total_expansions) + ";" + str(avg_Astar_exp_sum_max_per_timestep)
                        + ";" + str(avg_sum_of_costs) + ";" + str(avg_maxAstar) + ";" + str(avg_parallel_rounds) + "\n")
        excel_string = excel_string.replace(".", ",")
        print("Excel: ", excel_string)
    except:
        print("0 run completate")
        excel_string = "0;0;0;0;0;0;0;0;0\n"

    with open(file_name, 'a') as file:
        file.write("\n\n" + "Numero di run completate: " + str(run_complete) + "\n")
        file.write("Numero medio di task completati: " + str(avg_completed_tasks) + "\n")
        file.write("Makespan medio: " + str(avg_makespan) + "\n")
        file.write("Deviazione standard del makespan: " + str(std_makespan) + "\n")
        file.write("Tempo medio di servizio: " + str(avg_avg_service_time) + "\n")
        file.write("Deviazione standard del service time: " + str(avg_std_dev) + "\n")
        file.write("Chiamate ad A* medie: " + str(avg_Astar_calls) + "\n")
        file.write("Espansioni totali di A* in media: " + str(avg_Astar_total_expansions) + "\n")
        file.write("Espansioni medie di A* considerando la parallelizzazione: " + str(avg_Astar_exp_sum_max_per_timestep) + "\n")
        file.write("Sum of costs: " + str(avg_sum_of_costs) + "\n")
        file.write("Max Astar: " + str(avg_maxAstar) + "\n")
        file.write("Parallel rounds: " + str(avg_parallel_rounds) + "\n")

        file.write("\n\n" + "Excel: ")
        file.write(excel_string)



