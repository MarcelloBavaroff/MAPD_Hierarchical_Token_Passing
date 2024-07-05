import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np


class PrintMatrix(object):
    def __init__(self, all_paths, dimensions, obstacles, non_task_endpoints, goal_endpoints):

        self.dimensions = dimensions
        self.ratio = dimensions[0] / dimensions[1]
        self.heatmap = np.zeros((dimensions[0], dimensions[1]))
        self.mask = np.zeros((dimensions[0], dimensions[1]), dtype=bool)
        self.obstacles = obstacles
        self.goal_endpoints = goal_endpoints

        for o in obstacles:
            self.mask[o] = True

        for path in all_paths.values():
            for c in path:
                # non considero i NON-TE
                if tuple([c['x'], c['y']]) not in non_task_endpoints and tuple([c['x'], c['y']]) not in goal_endpoints:
                    self.heatmap[c['x'], c['y']] += 1
                # if c['x'] == 0 or c['x'] == dimensions[0] - 1 or c['y'] == 0 or c['y'] == dimensions[1] - 1:
                #     continue




    def plot_heatmap(self):
        # Crea una heatmap utilizzando seaborn
        plt.figure(figsize=(5*self.ratio, 5))
        rotated_data = np.rot90(self.heatmap, k=-1)
        rotated_mask = np.rot90(self.mask, k=-1)
        to_show = sns.heatmap(rotated_data, mask=rotated_mask, annot=False, cmap="YlGnBu", cbar=True, linewidths=.5, linecolor='gray')

        for o in self.obstacles:
            to_show.add_patch(plt.Rectangle((o[0], o[1]), 1, 1, fill=True, color='black', lw=0.5))
        # Mostra la heatmap
        to_show.set_yticklabels(to_show.get_yticklabels()[::-1])
        plt.title('Heatmap della matrice')
        plt.show()




