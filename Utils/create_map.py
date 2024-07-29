import yaml


def create_non_task_endpoints():
    non_task_endpoints = []
    i = 0
    rows = [7, 15, 23, 59, 67, 75]
    for y in rows:
        for x in range(4, 24):
            non_task_endpoints.append([x, y])
            i += 1
        for x in range(146, 166):
            non_task_endpoints.append([x, y])
            i += 1

    for non in non_task_endpoints:
        print("- !!python/tuple", non)


def create_delivery():
    # Inizializza la lista degli ostacoli
    delivery = []

    # Itera attraverso le righe e le colonne specificate
    # for i in range(1, 83):
    #     delivery.append([1, i])
    #     delivery.append([168, i])

    # rows = [1, 82]
    # for j in rows:
    #     for i in range(4, 23):
    #         delivery.append([i, j])
    #     for i in range(147, 166):
    #         delivery.append([i, j])
    for x in range(26, 135, 12):
        for y in range(13, 78, 8):
            for i in range(x, x + 10):
                delivery.append([i, y])
                delivery.append([i, y + 1])

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
    i=0
    rows = [7, 15, 23, 59, 67, 75]
    for y in rows:
        for x in range(4, 24):
            agent = {
                'name': f'agent{i}',
                'start': [x, y]
            }
            agents.append(agent)
            i += 1
        for x in range(146, 166):
            agent = {
                'name': f'agent{i+120}',
                'start': [x, y]
            }
            agents.append(agent)
            i += 1



    for agent in agents:
        print("-    start: [", agent['start'][0], ",", agent['start'][1], "]")
        print("     name:", agent['name'])


def create_pickup():
    # Inizializza la lista degli ostacoli
    pickups = []

    for x in range(26, 135, 12):
        for y in range(9, 74, 8):
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
        
def create_frontiers_vertical_corridor1():
    # Inizializza la lista degli ostacoli
    frontiers = []

    #Itera attraverso le righe e le colonne specificate
    columns_internal = [30, 41, 63, 74]
    columns_internal_right = [85, 96, 118, 129]


    k = 4
    for x in columns_internal_right:
        for y in range(1, 62, 6):
            frontiers.append([x, y, k, x+1, y, k+1])
            frontiers.append([x+1, y+3, k+1, x, y+3, k])
            # frontiers.append([x+1, y, k+1, x, y, k])
            # frontiers.append([x, y+3, k, x+1, y+3, k+1])
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

    # k = 0
    # for j in range(39, 40, 1):
    #     for i in range(4, 26, 2):
    #         # 31, 1, k+1, 30, 1, k
    #         f = [i, j + 1, 11, i, j, 0]
    #         # 30, 2, k, 31, 2, k+1
    #         ff = [i + 1, j, 0, i + 1, j + 1, 11]
    #         frontiers.append(f)
    #         frontiers.append(ff)
    #     #k += 1

    k=0
    rows = [3, 11, 19, 27, 35, 47, 55, 63, 71, 79]
    #rows2 = [18, 22, 26, 30, 34, 38, 46, 54, 58, 66, 74, 78, 82, 86, 90, 94]
    #manual = [39]
    for j in rows:
        #j = j+15
        for i in range(144, 165, 2):
            # 31, 1, k+1, 30, 1, k
            f = [i, j + 1, k+1, i, j, k]
            # 30, 2, k, 31, 2, k+1
            ff = [i + 1, j, k, i + 1, j + 1, k+1]
            frontiers.append(f)
            frontiers.append(ff)
        for i in range(4, 26, 2):
            # 31, 1, k+1, 30, 1, k
            f = [i, j + 1, k+1, i, j, k]
            # 30, 2, k, 31, 2, k+1
            ff = [i + 1, j, k, i + 1, j + 1, k+1]
            frontiers.append(f)
            frontiers.append(ff)
        for i in range(36, 133, 12):
            # 31, 1, k+1, 30, 1, k
            f = [i, j + 1, k+1, i, j, k]
            # 30, 2, k, 31, 2, k+1
            ff = [i + 1, j, k, i + 1, j + 1, k+1]
            frontiers.append(f)
            frontiers.append(ff)
        k += 1

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


create_frontiers_vertical_corridor1()

