import yaml


def create_agents():
    # Apri il file yaml e carica i dati
    with open('../MAPD_partition/Environments/Stern/warehouse-10-20-10-2-2.yaml', 'r') as file:
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
    obstacles = []

    # Itera attraverso le righe e le colonne specificate
    for k in range(10):
        for i in range(26 + 11*k, 36 + 11*k):
            for j in range(5, 81, 4):
                # Aggiungi un ostacolo alla lista degli ostacoli
                obstacle = [[i, j], [i, j + 1]]
                obstacles.extend(obstacle)

    # Stampa gli ostacoli generati
    for obstacle in obstacles:
        print("-    ", obstacle)


def create_frontiers():

    # Inizializza la lista degli ostacoli
    frontiers = []

    # Itera attraverso le righe e le colonne specificate

    for i in range(2, 84, 4):
        f = [102, i, 2, 103, i, 3]
        frontiers.append(f)



    # Stampa gli ostacoli generati
    for f in frontiers:
        print("-    ", f)

create_frontiers()