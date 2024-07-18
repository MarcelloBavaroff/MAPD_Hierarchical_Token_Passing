import yaml


def create_non_task_endpoints():
    # Inizializza la lista degli ostacoli
    non_task_endpoints = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(4, 24):
        non_task_endpoints.append([i, 1])
        non_task_endpoints.append([i, 82])
        non_task_endpoints.append([i, 4])
        non_task_endpoints.append([i, 79])

    for i in range(146, 166):
        non_task_endpoints.append([i, 1])
        non_task_endpoints.append([i, 82])
        non_task_endpoints.append([i, 4])
        non_task_endpoints.append([i, 79])

    for non in non_task_endpoints:
        print("- !!python/tuple", non)


def create_delivery():
    # Inizializza la lista degli ostacoli
    delivery = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(1, 83):
        delivery.append([1, i])
        delivery.append([168, i])
    for d in delivery:
        print("-    ", d)


def create_agents():
    # Apri il file yaml e carica i dati
    with open(f'Environments/Stern/160aV2-warehouse-170x84.yaml',
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

def create_agents_independent():
    agents = []
    i=80
    for y in range(3, 80, 2):
        agent = {
            'name': f'agent{i}',
            'start': [10, y]
        }
        agents.append(agent)
        agent = {
            'name': f'agent{i+1}',
            'start': [159, y]
        }
        agents.append(agent)
        i+=2

    for agent in agents:
        print("-    start: [", agent['start'][0], ",", agent['start'][1], "]")
        print("     name:", agent['name'])


def create_pickup():
    # Inizializza la lista degli ostacoli
    pickups = []

    for x in range(26, 135, 12):
        for y in range(69, 78, 4):
            for i in range(x, x + 10):
                pickups.append([i, y])
                pickups.append([i, y + 1])

    # Stampa gli ostacoli generati
    for p in pickups:
        print("-    ", p)


def create_frontiers_vertical():
    # Inizializza la lista degli ostacoli
    frontiers = []

    #Itera attraverso le righe e le colonne specificate
    columns_internal = [30, 42, 54, 66, 78, 90, 102, 114, 126, 138 ]
    columns_external = [150, 162]

    k = 11
    for j in columns_internal:
        for i in range(41, 82, 4):
            frontiers.append([j, i, k, j+1, i, k + 1])
            frontiers.append([j + 1, i+1, k + 1, j, i + 1, k])
        k += 1

    # k = 12
    # for j in columns_external:
    #     for i in range(1, 82, 2):
    #         frontiers.append([j, i, k, j+1, i, k + 1])
    #         frontiers.append([j + 1, i+1, k + 1, j, i + 1, k])
    #     k += 1



    # Stampa gli ostacoli generati
    for f in frontiers:
        print("-    ", f)


def create_frontiers_horizontal():
    # Inizializza la lista degli ostacoli
    frontiers = []

    k = 0
    for j in range(39, 40, 1):
        for i in range(4, 26, 2):
            # 31, 1, k+1, 30, 1, k
            f = [i, j + 1, 11, i, j, 0]
            # 30, 2, k, 31, 2, k+1
            ff = [i + 1, j, 0, i + 1, j + 1, 11]
            frontiers.append(f)
            frontiers.append(ff)
        #k += 1

    k=0
    rows = [3, 7, 11, 15, 19, 23, 31, 39, 43, 51, 59, 63, 67, 71, 75, 79]
    rows2 = [18, 22, 26, 30, 34, 38, 46, 54, 58, 66, 74, 78, 82, 86, 90, 94]
    manual = [39]
    #for j in manual:
        #j = j+15
        # for i in range(144, 165, 2):
        #     # 31, 1, k+1, 30, 1, k
        #     f = [i, j + 1, 12, i, j, 11]
        #     # 30, 2, k, 31, 2, k+1
        #     ff = [i + 1, j, 11, i + 1, j + 1, 12]
        #     frontiers.append(f)
        #     frontiers.append(ff)
        # for i in range(4, 26, 2):
        #     # 31, 1, k+1, 30, 1, k
        #     f = [i, j + 1, 12, i, j, 1]
        #     # 30, 2, k, 31, 2, k+1
        #     ff = [i + 1, j, 1, i + 1, j + 1, 12]
        #     frontiers.append(f)
        #     frontiers.append(ff)
        # for i in range(36, 133, 12):
        #     # 31, 1, k+1, 30, 1, k
        #     f = [i, j + 1, 12+k, i, j, 1+k]
        #     # 30, 2, k, 31, 2, k+1
        #     ff = [i + 1, j, 1+k, i + 1, j + 1, 12+k]
        #     frontiers.append(f)
        #     frontiers.append(ff)
        #     k += 1

    for f in frontiers:
        print("-    ", f)


def create_obstacles():
    obstacles = []
    for x in range(2, 111, 12):
        for y in range(18, 95, 4):
            for i in range(x, x+10):
                obstacles.append([i, y])
                obstacles.append([i, y+1])

    for obs in obstacles:
        print("- !!python/tuple", obs)

create_frontiers_horizontal()

