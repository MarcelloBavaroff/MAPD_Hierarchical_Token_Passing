import yaml

def create_non_task_endpoints():
    # Inizializza la lista degli ostacoli
    non_task_endpoints = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(1, 16):
        n = [159, i]
        non_task_endpoints.append(n)

    for i in range(47, 63):
        n = [159, i]
        non_task_endpoints.append(n)

    for non in non_task_endpoints:
        print("- !!python/tuple", non)


def create_delivery():
    # Inizializza la lista degli ostacoli
    delivery = []

    # Itera attraverso le righe e le colonne specificate
    for i in range(16, 47):
        n = [1, i]
        delivery.append(n)

    for d in delivery:
        print("-    ", d)


def create_agents():
    # Apri il file yaml e carica i dati
    with open('/Users/bavaroff258/PycharmProjects/MAPD_TP/Environments/Stern/warehouse-161x63p3.yaml', 'r') as file:
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
                pick = [[i, j]]#, [i, j + 1]]
                pickups.extend(pick)

    # Stampa gli ostacoli generati
    for p in pickups:
        print("-    ", p)


def create_frontiers():
    # Inizializza la lista degli ostacoli
    frontiers = []

    # Itera attraverso le righe e le colonne specificate

    for i in range(4, 62, 6):
        f = [63, i, 1, 64, i, 2]
        frontiers.append(f)

    # Stampa gli ostacoli generati
    for f in frontiers:
        print("-    ", f)


create_frontiers()
