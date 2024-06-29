import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np


class PrintMatrix(object):
    def __init__(self, all_paths, dimensions):

        self.dimensions = dimensions
        self.heatmap = np.zeros((dimensions[0], dimensions[1]))
        for path in all_paths.values():
            for c in path:
                #non considero i NON-TE
                if c['x'] == 0 or c['x'] == dimensions[0] - 1 or c['y'] == 0 or c['y'] == dimensions[1] - 1:
                    continue
                self.heatmap[c['x'], c['y']] += 1



    def plot_heatmap(self):
        # Crea una heatmap utilizzando seaborn
        plt.figure(figsize=(10, 20))
        sns.heatmap(self.heatmap, annot=False, cmap="YlGnBu", cbar=True, linewidths=.5, linecolor='gray')

        # Mostra la heatmap
        plt.title('Heatmap della matrice')
        plt.show()




