import pandas as pd
import numpy as np
import time
import csv
overlapMatrix = np.zeros((300,300))
Matrices = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\Matrices\\'
bestPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestPath\\'
import matplotlib.pyplot as plt

# test = pd.read_csv(Matrices+"30_10",  delimiter = ',', header = None, nrows=300).to_numpy()
# plt.imshow(test)
# plt.colorbar()
# plt.show()
# copySol = tmpSolution.copy()
# removeFromCopy = []
# boolMap = np.where(overlapMatrix > 1)
overlapMatrix = np.zeros((300, 300))
with open(bestPath + "30_10", mode='r') as csv_file:
    reader = csv.reader(csv_file, delimiter=',')
    for i, row in enumerate(reader):
        best_inds = row
best_inds = list(map(int, best_inds))
start_time = time.time()
x = 0
copySol = best_inds.copy()
for i, chunk in enumerate(pd.read_csv(Matrices+"30_10", delimiter=',', header=None, chunksize=300, index_col=None, low_memory = False)):
        for j, k in enumerate(copySol):
            if k == i:
                overlapMatrix = np.add(overlapMatrix, chunk.to_numpy())
            # if (k+1)*300 <= 300:
            #     slChunk = chunk.iloc[k * 300:((k + 1) * 300), :]
            #     try:
            #         overlapMatrix = np.add(overlapMatrix, slChunk.to_numpy())
            #         self.overlapStorage = self.overlapStorage.append(slChunk, ignore_index=True)
            #         removeFromCopy.append(j)
            #     except ValueError:
            #         print(slChunk.shape)
            #         print(k*300)
            #         continue

print("--- %s seconds ---" % (time.time() - start_time))
plt.imshow(overlapMatrix)
print(overlapMatrix)
plt.colorbar()
plt.show()

# if ind == 0:
            #     overlap = pd.read_csv(self.path, delimiter=',', header=None, nrows=300)
            # else:
            #     overlap = pd.read_csv(self.path,  delimiter = ',', header = None, skiprows=ind*300, nrows=300)
            # overlapMatrix = np.add(overlapMatrix, overlap.to_numpy())
            # overlapMatrix = np.add(overlapMatrix, self.overLap.iloc[ind*300:((ind+1)*300), :].to_numpy())
            # except ValueError:
            #     tmp = self.overLap.iloc[0*300:((1)*300), :].to_numpy()
            #     print(f'{ind*35}'
            #           f'\n {(ind+1)*35}'
            #           f'\n {(ind+1)*35-ind*35}'
            #           f'\n IND: {ind}'
            #           f'\n {tmp.shape}')
            # except ValueError:
            #     # print(ind)
            #     continue
            # if l%10 == 0:
            #     print(tmpCoverage)
            #     print(len(tmpSolution))
            #     print("--- %s seconds ---" % (time.time() - start_time))
        #TOdo:
        # start_time = time.time()
        # copySol = tmpSolution.copy()
        # removeFromCopy = []
        # # boolMap = np.where(overlapMatrix > 1)
        # start_time = time.time()
        # if copySol:
        #     for i, chunk in enumerate(pd.read_csv(self.path, delimiter=',', header=None, chunksize=self.sizeOfChunk)):
        #         print(chunk.shape)
        #         for j, k in enumerate(copySol):
        #             if (k+1)*300 <= self.sizeOfChunk:
        #                 slChunk = chunk.iloc[k * 300:((k + 1) * 300), :]
        #                 try:
        #                     overlapMatrix = np.add(overlapMatrix, slChunk.to_numpy())
        #                     self.overlapStorage = self.overlapStorage.append(slChunk, ignore_index=True)
        #                     removeFromCopy.append(j)
        #                 except ValueError:
        #                     print(slChunk.shape)
        #                     print(k*300)
        #                     continue
        #
        #         for e, r in enumerate(removeFromCopy):
        #             copySol.pop(r)
        #         removeFromCopy = []
        # print("--- %s seconds ---" % (time.time() - start_time))
        # print(f'copy should be empty: {copySol}')
        # print(f'Shape of appended matrices: {self.overlapStorage.shape}')
        # boolMap = np.where(overlapMatrix > 1)
        # total_overlap = np.sum(overlapMatrix[boolMap]) #each initial overlap gets a weight of two, subsquent overlaps increase this by 1
