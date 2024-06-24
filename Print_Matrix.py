import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np


class PrintMatrix(object):
    def __init__(self, all_paths, dimensions):

        self.dimensions = dimensions
        self.heatmap = np.zeros((dimensions[0], dimensions[1]))
        for path in all_paths.values():
            for c in path:
                self.heatmap[c['x'], c['y']] += 1



    def plot_heatmap(self):
        # Crea una heatmap utilizzando seaborn
        plt.figure(figsize=(10, 20))
        sns.heatmap(self.heatmap, annot=False, cmap="YlGnBu", cbar=True, linewidths=.5, linecolor='gray')

        # Mostra la heatmap
        plt.title('Heatmap della matrice')
        plt.show()




