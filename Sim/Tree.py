import pandas as pd
import numpy as np
import time
import csv

import matplotlib.pyplot as plt

# focPath = 'D:\\Thesis_results\\FamilyofCurves\\'
#     # bestPath = 'D:\\Masters\\Thesis\\Git\\optimising_TSHD_path\\Sim\\BestPath\\'
#     bestPath = 'D:\\Thesis_results\\BestPath\\'
#     optPath = 'D:\\Thesis_results\\opt\\'
#     Matrices = 'D:\\Thesis_results\\Matrices\\'
#     bestScores = 'D:\\Thesis_results\\bestScores\\'
    # bestMatrices = "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\bestMatrices\\"
    # distributionMat = "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\EvenDistribution\\"
    # plot_path = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\plots\\'
    # # plot_path = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\old\\2000_noMat_100bayes_greedyWdel\\'
    # res = load('C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl')
overlapMatrix = np.zeros((300,300))
Matrices = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\Matrices\\'
bestPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestPath\\'
res = load('D:\\Thesis_results\\check\\checkpoint.pkl')
plot_path = 'D:\\Thesis_results\\plots\\'
best_hyp = str(res.x[0]) + '_' + str(res.x[1]) + '_' + str(res.x[2]) + '_' +str(res.x[3])
saveme = best_hyp + str(numOfPaths) + '.png'
plot_convergence(res)
plt.savefig(plot_path+'convergence'+saveme)
plt.show()
print(f'Best path length: {res.fun}')
print(f'best hyperparameters: {res.x}')
print(f'set1:\n {res.x_iters}')
print(f'set1:\n {res.func_vals}')
_ = plot_evaluations(res)
plt.show()
plt.savefig(plot_path + 'evalutions'+ '.png')
_ = plot_objective(res, n_samples=500)
plt.show()
plt.savefig(plot_path + 'objectives'+ '.png')


# best_score = pd.read_csv(bestScores+best_hyp, delimiter=',', header=None)
# min_index = int(best_score.idxmin()
# print(int(min_index))

# if min_index == 0:
#     best_indexs = pd.read_csv(bestPath+best_hyp, header=None, sep=',', engine='python', index_col=None, nrows=1)
# else:
#     best_indexs = pd.read_csv(bestPath+best_hyp, header=None, sep=',', engine='python', index_col=None, skiprows=min_index, nrows=1)
best_inds = []
with open(bestPath+best_hyp, mode='r') as csv_file:
    reader = csv.reader(csv_file, delimiter=',')
    for i, row in enumerate(reader):
        best_inds.extend(row)
        # if i == min_index:
        #     best_inds = row
best_inds = list(map(int, best_inds))
full_set = pd.read_csv(focPath + best_hyp, delimiter=',', header=None)
# full_set.info()
opt_data = pd.read_csv(optPath + best_hyp, delimiter=',', names=['path_length'])
opt_data1 = pd.read_csv(optPath + best_hyp, delimiter=',', names=['path_length']).values.astype("int").squeeze()
initopt_data = opt_data1.tolist()
cov_length = []
true_index = []
overlapMatrix = np.zeros((300,300))
bestSzie = 0
bestind = 0
# opt_data1 = []
# with open(optPath + best_hyp, mode='r') as csv_file:
#     reader = csv.reader(csv_file, delimiter=',')
#     print(reader)
#     for k, l in enumerate(reader):
#         for j in l:
#             print(j)
#             opt_data1.extend(int(j))




shortest = int(opt_data['path_length'].idxmin())
id = opt_data['path_length'].iloc[shortest]
id1 = opt_data['path_length'].iloc[0:shortest]
id2 = int(id1.sum())
x = full_set.iloc[id2:id2+id, 0]
# y.append(full_set.iloc[236:236+237-5,1])
y = full_set.iloc[id2:id2+id, 1]
# x.append(full_set.iloc[:, 0])
# y.append(full_set.iloc[:, 1])
plt.plot(x, y, label=best_hyp)
# plt.legend()
plt.grid(True)
plt.xlabel("x co-ordinates")
plt.ylabel("y co-ordinates")
plt.xlim([-100, 800])
plt.ylim([-100, 800])
plt.viridis()
# plt.axis("equal")
plt.title("Shortest path from hyp_parameters: 18_30_2.317_0.015")
plt.savefig(plot_path + 'shortpaths' + saveme)
plt.show()

shortest = int(opt_data['path_length'].idxmax())
id = opt_data['path_length'].iloc[shortest]
id1 = opt_data['path_length'].iloc[0:shortest]
id2 = int(id1.sum())
x = full_set.iloc[id2:id2 + id, 0]
# y.append(full_set.iloc[236:236+237-5,1])
y = full_set.iloc[id2:id2 + id, 1]
# x.append(full_set.iloc[:, 0])
# y.append(full_set.iloc[:, 1])
plt.plot(x, y, label=best_hyp)
# plt.legend()
plt.grid(True)
plt.xlabel("x co-ordinates")
plt.ylabel("y co-ordinates")
plt.xlim([-100, 800])
plt.ylim([-100, 800])
plt.viridis()
# plt.axis("equal")
plt.title("Longest path from hyp_parameters: 18_30_2.317_0.015")
plt.savefig(plot_path + 'shortpaths' + saveme)
plt.show()

for ind, row in enumerate(best_inds):
    tmpMatrix = np.load(Matrices + best_hyp + '\\' + str(row) + '.npy')
    overlapMatrix = np.add(overlapMatrix, tmpMatrix)
    boolMap = np.where(tmpMatrix > 0)
    size = tmpMatrix[boolMap].size
    if size > bestSzie:
        bestSzie = size
        bestind = row


id1 = opt_data['path_length'].iloc[0:bestind]
bes = int(opt_data['path_length'].iloc[bestind])
id2 = int(id1.sum())
x = full_set.iloc[id2:id2+bes, 0]
# y.append(full_set.iloc[236:236+237-5,1])
y = full_set.iloc[id2:id2+bes, 1]
# x.append(full_set.iloc[:, 0])
# y.append(full_set.iloc[:, 1])
plt.plot(x, y, label=best_hyp)
# plt.legend()
plt.grid(True)
plt.xlabel("x co-ordinates")
plt.ylabel("y co-ordinates")
plt.xlim([-100, 800])
plt.ylim([-100, 800])
plt.viridis()
# plt.axis("equal")
plt.title("Path with most area covered from hyp_parameters: 18_30_2.317_0.015")
plt.savefig(plot_path + 'shortpaths' + saveme)
plt.show()


# try:
#     start_time = time.time()
#     overlapMatrix = np.load(bestMatrices+best_hyp+".npy")
#     print("--- %s seconds ---" % (time.time() - start_time))
# except:
#     overlapMatrix = np.zeros((300,300))
#     for i, chunk in enumerate(pd.read_csv(Matrices+best_hyp, delimiter=',', header=None, chunksize=300, index_col=None, low_memory = False)):
#         for j, k in enumerate(best_inds):
#             if k == i:
#                 arr = chunk.to_numpy()
#                 overlapMatrix = np.add(overlapMatrix, arr)
#                 # np.save(self.bestMatrices+str(k), arr)
#     # boolMap = np.where(overlapMatrix > 1)
#     # total_overlap = np.sum(overlapMatrix[boolMap])
#     np.save(bestMatrices+best_hyp, overlapMatrix)
plt.imshow(overlapMatrix)
plt.colorbar()
plt.title("Dredged Locations")
plt.savefig(plot_path + 'dredged_locations'+saveme)
plt.show()
orig_cmap = matplotlib.cm.viridis
shifted_cmap = shiftedColorMap(orig_cmap, midpoint=0, name='shifted')

plt.imshow(overlapMatrix, interpolation=None, cmap=shifted_cmap)
plt.colorbar()
plt.title("Dredged Locations shifted colourmap")
plt.savefig(plot_path + 'dredged_locations_shifted'+saveme)
plt.show()
tally=0
start_time = time.time()
for z in np.nditer(overlapMatrix):
    if z > 0:
        tally +=1
print("--- %s seconds ---" % (time.time() - start_time))
print(tally)
start_time = time.time()
boolMap = np.where(overlapMatrix > 0)
size = overlapMatrix[boolMap].size
print("--- %s seconds ---" % (time.time() - start_time))
print(f'size: {size}')