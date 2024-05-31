import RoothPath
import os
import re
import yaml
import json

if __name__ == '__main__':
    yaml_dic = {}
    map = 'warehouse-20-40-10-2-2'
    with open(os.path.join(os.path.join(RoothPath.get_root(), 'Environments/Stern'), (map+'.map'))) as ascii_map:
        ascii_map.readline()
        h = int(re.findall(r'\d+', ascii_map.readline())[0])
        w = int(re.findall(r'\d+', ascii_map.readline())[0])
        yaml_dic['agents'] = [{'start': [48, 10], 'name': 'agent0'}]
        yaml_dic['map'] = {'dimensions': [w, h], 'obstacles': [], 'non_task_endpoints': [[48, 10]],
                           'start_locations': [[50, 10]], 'goal_locations': [[54, 10]]}
        yaml_dic['n_tasks'] = 1
        yaml_dic['task_freq'] = 1
        yaml_dic['n_delays_per_agent'] = 10
        ascii_map.readline()
        for i in range(h - 1, -1, -1):
            line = ascii_map.readline()
            print(line)
            for j in range(w):
                if line[j] == '@' or line[j] == 'T':
                    yaml_dic['map']['obstacles'].append((j, i))

    with open(os.path.join(RoothPath.get_root(), 'config.json'), 'r') as json_file:
        config = json.load(json_file)
    with open(os.path.join(os.path.join(RoothPath.get_root(), config['input_path']), (map+'.yaml')), 'w') as param_file:
        yaml.dump(yaml_dic, param_file)
