import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

# Crea una matrice di esempio
matrix = np.array([
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9]
])

# Crea una heatmap utilizzando seaborn
plt.figure(figsize=(8, 6))
sns.heatmap(matrix, annot=True, cmap="YlGnBu", cbar=True, linewidths=.5, linecolor='gray')

# Mostra la heatmap
plt.title('Heatmap della matrice')
plt.show()
