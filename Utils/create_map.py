import yaml


def create_non_task_endpoints():
    # Inizializza la lista degli ostacoli
    non_task_endpoints = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(20, 40):
        non_task_endpoints.append([i, 0])
        non_task_endpoints.append([i, 113])

    for i in range(82, 102):
        non_task_endpoints.append([i, 0])
        non_task_endpoints.append([i, 113])

    for non in non_task_endpoints:
        print("- !!python/tuple", non)


def create_delivery():
    # Inizializza la lista degli ostacoli
    delivery = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(40, 82):
        delivery.append([i, 0])
        delivery.append([i, 113])
    for d in delivery:
        print("-    ", d)


def create_agents():
    # Apri il file yaml e carica i dati
    with open('\\Users\marce\PycharmProjects\MAPD_partition\Environments\Stern\corridoi2-122x114-3p.yaml',
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

    for x in range(2, 111, 12):
        for y in range(20, 93, 4):
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
    k = 0
    for j in range(6, 115, 12):
        for i in range(16, 97, 4):
            frontiers.append([j, i, k, j+1, i, k + 1])
            frontiers.append([j + 1, i+1, k + 1, j, i + 1, k])
        k += 1

    # j = 103
    # k = 5
    # for i in range(16, 97, 4):
    #
    #     f = [j, i, k, j + 1, i, k + 1]
    #     ff = [j + 1, i+1, k + 1, j, i + 1, k]
    #     frontiers.append(f)
    #     frontiers.append(ff)

    # Stampa gli ostacoli generati
    for f in frontiers:
        print("-    ", f)


def create_frontiers_horizontal():
    # Inizializza la lista degli ostacoli
    frontiers = []

    #Itera attraverso le righe e le colonne specificate
    k = 5
    for j in range(62, 95, 8):
        for i in range(0, 121, 12):
            # 31, 1, k+1, 30, 1, k
            f = [i, j+1, k+1, i, j, k]
            # 30, 2, k, 31, 2, k+1
            ff = [i+1, j, k, i+1, j+1, k+1]
            frontiers.append(f)
            frontiers.append(ff)
        k += 1
    #k = 0
    # for j in range(8, 55, 9):
    #     for i in range(4, 25, 2):
    #         # 31, 1, k+1, 30, 1, k
    #         f = [i, j + 1, k + 1, i, j, k]
    #         # 30, 2, k, 31, 2, k+1
    #         ff = [i + 1, j, k, i + 1, j + 1, k + 1]
    #         frontiers.append(f)
    #         frontiers.append(ff)
    #     k += 1

    # k=7
    # j=98
    # for i in range(0, 121, 2):
    #     # 31, 1, k+1, 30, 1, k
    #     f = [i, j + 1, k + 1, i, j, k]
    #     # 30, 2, k, 31, 2, k+1
    #     ff = [i + 1, j, k, i + 1, j + 1, k + 1]
    #     frontiers.append(f)
    #     frontiers.append(ff)

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
