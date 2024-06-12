import yaml


def create_non_task_endpoints():
    # Inizializza la lista degli ostacoli
    non_task_endpoints = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(1, 83):
        n = [168, i]
        non_task_endpoints.append(n)

    # for i in range(47, 63):
    #     n = [159, i]
    #     non_task_endpoints.append(n)

    for non in non_task_endpoints:
        print("- !!python/tuple", non)


def create_delivery():
    # Inizializza la lista degli ostacoli
    delivery = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(1, 83):
        n = [1, i]
        delivery.append(n)

    for d in delivery:
        print("-    ", d)


def create_agents():
    # Apri il file yaml e carica i dati
    with open('/Users/bavaroff258/PycharmProjects/MAPD_partition/Environments/Stern/warehouse-170x84-3p_one_side.yaml',
              'r') as file:
        data = yaml.load(file, Loader=yaml.FullLoader)

    # Ottieni la lista di non_task_endpoints
    non_task_endpoints = data['map']['non_task_endpoints']

    # Genera gli agenti
    agents = []
    for i, endpoint in enumerate(non_task_endpoints):
        agent = {
            'name': f'agent{i}',
            'start': endpoint
        }
        agents.append(agent)

    # Stampa gli agenti generati
    for agent in agents:
        print("-    start: [", agent['start'][0], ",", agent['start'][1], "]")
        print("     name:", agent['name'])


def create_pickup():
    # Inizializza la lista degli ostacoli
    pickups = []

    # Itera attraverso le righe e le colonne specificate
    for k in range(10):
        for i in range(26 + 11 * k, 36 + 11 * k):
            for j in range(4, 61, 3):
                # Aggiungi un ostacolo alla lista degli ostacoli
                pick = [[i, j]]  #, [i, j + 1]]
                pickups.extend(pick)

    # Stampa gli ostacoli generati
    for p in pickups:
        print("-    ", p)


def create_frontiers_vertical():
    # Inizializza la lista degli ostacoli
    frontiers = []

    # Itera attraverso le righe e le colonne specificate
    k = 1
    for j in range(30, 140, 12):
        for i in range(1, 82, 4):
            # 31, 1, k+1, 30, 1, k
            f = [j, i, k, j + 1, i, k + 1]
            # 30, 2, k, 31, 2, k+1
            ff = [j + 1, i + 1, k + 1, j, i + 1, k]
            frontiers.append(f)
            frontiers.append(ff)
        #frontiers.append([j, 61, k, j + 1, 61, k + 1])
        k += 1

    # j = 18
    # k = 0
    # for i in range(1, 82, 2):
    #     # 31, 1, k+1, 30, 1, k
    #     f = [j, i, k, j + 1, i, k + 1]
    #     # 30, 2, k, 31, 2, k+1
    #     ff = [j + 1, i+1, k + 1, j, i + 1, k]
    #     frontiers.append(f)
    #     frontiers.append(ff)

    # Stampa gli ostacoli generati
    for f in frontiers:
        print("-    ", f)


def create_frontiers_horizontal():
    # Inizializza la lista degli ostacoli
    frontiers = []

    # Itera attraverso le righe e le colonne specificate
    #k = 0
    # for j in range(20, 41, 20):
    #     for i in range(135, 156, 2):
    #         # 31, 1, k+1, 30, 1, k
    #         f = [i, j+1, k+1, i, j, k]
    #         # 30, 2, k, 31, 2, k+1
    #         ff = [i+1, j, k, i+1, j+1, k+1]
    #         frontiers.append(f)
    #         frontiers.append(ff)
    #     k += 1
    # k = 0
    # for j in range(8, 55, 9):
    #     for i in range(4, 25, 2):
    #         # 31, 1, k+1, 30, 1, k
    #         f = [i, j + 1, k + 1, i, j, k]
    #         # 30, 2, k, 31, 2, k+1
    #         ff = [i + 1, j, k, i + 1, j + 1, k + 1]
    #         frontiers.append(f)
    #         frontiers.append(ff)
    #     k += 1


    # for i in range(4, 26, 2):
    #     frontiers.append([i, 16, 1, i, 15, 0])
    #     frontiers.append([i+1, 15, 0, i+1, 16, 1])
    #     frontiers.append([i, 32, 2, i, 31, 1])
    #     frontiers.append([i+1, 31, 1, i+1, 32, 2])
    #     frontiers.append([i, 52, 3, i, 51, 2])
    #     frontiers.append([i+1, 51, 2, i+1, 52, 3])
    #     frontiers.append([i, 68, 4, i, 67, 3])
    #     frontiers.append([i+1, 67, 3, i+1, 68, 4])

    for f in frontiers:
        print("-    ", f)


create_frontiers_vertical()
