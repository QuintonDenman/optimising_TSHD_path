"""
Simulated Annealing Class
"""
import random
import time
import pandas as pd
import numpy as np
class Greedy:
    def __init__(self, pathLen, path, coverage, iterationLimit, width):
        self.pathLen = pathLen
        self.path = path
        self.width = width
        self.coverage = coverage
        self.dataDim = len(self.coverage) - 1
        self.iteration = 0
        self.iterationLimit = iterationLimit
        self.solution = []
        self.solutionCoverage = 0
        self.sizeOfChunk = 30000
        self.overlapStorage = pd.DataFrame()

    # def intersectionLists(self, mainLst):
    #     '''Expects a list of lists [[], [], ..., []]'''
    #     count = 0
    #     print("started overlap calc")
    #     for a, b in itertools.combinations(mainLst, 2):
    #         temp = set(b)
    #         lst3 = [value for value in a if value in temp]
    #         count += len(lst3)
    #     print("finished overlap calc")
    #     return count
    def randomSolution(self):
        tmpCoverage = 0
        tmpPathLen = 0
        tmpSolution = []
        overlapMatrix = np.zeros((self.width, self.width))
        l=0
        while tmpCoverage <= 90:
            l+=1
            k=0
            ind = random.randint(0, self.dataDim)
            while(ind in tmpSolution):
                # k +=1
                # if k >= self.dataDim*2:
                #     print("bag too small??????????????????????????????????????????????????????????????")

                ind = random.randint(0, self.dataDim)
            tmpSolution.append(ind)
            tmpCoverage += self.coverage[ind]
            tmpPathLen += self.pathLen[ind]
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

        return tmpPathLen, tmpCoverage, tmpSolution, #overlapMatrix, total_overlap

    # def calculateOverlap(self, allPaths):
    #
    #     total = np.zeros(self.width, self.width)
    #     for i, j in enumerate(allPaths):
    #         tmp = pd.allPaths.iloc[j:(j+self.width), :].to_numpy()
    #         total = np.add(tmp, total)
    #     return total
    # def calcBestScore(self, path, cov, over):


    def runGreedy(self):
        # pathLen, coverage, overlap, trueIndexs, overlapMatrix = self.randomSolution()
        pathLen, coverage, trueIndexs = self.randomSolution()
        # bisMatrix = overlapMatrix
        bestScore = pathLen - (coverage*100)
        # bestScore = pathLen - (coverage*100) + overlap
        print(f' '
              f' \n Initial coverage: {coverage}'
              f' \n Initial pathlen: {pathLen}'
              f' \n Initial Overall Score: {bestScore}'
              f'')
        # bestIndexs = trueIndexs
        print(f' Initial indexes: {trueIndexs}')
        while self.iteration < self.iterationLimit:
            self.iteration += 1
            if self.iteration %100 ==0:
                print(f'Starting Greedy iteration {self.iteration} of {self.iterationLimit}')
            ind = random.randint(0, self.dataDim)
            k=0
            while ind in trueIndexs:
                k+=1
                ind = random.randint(0, self.dataDim)
                if k >= self.dataDim*2:
                    print("stopping early bag too small??????????????????????????????????????????????????????????????")
                    print(bisInd)
                    print(len(bisInd))
                    print(self.dataDim)

                    return bestScore, bisInd
            # if ind == 0:
            #     possibleOverlap = pd.read_csv(self.path, delimiter=',', header=None,
            #                           nrows=300)
            # else:
            #     possibleOverlap = pd.read_csv(self.path, delimiter=',', header=None, skiprows=ind * 300, nrows=300)
            #DOUBLE CHECK
            seen = set()
            for x in trueIndexs:
                if x not in seen:
                    seen.add(x)
                else:
                    print("Problem mate")
                    print(x)
            #TODO:
            # if ind == 0:
            #     possibleOverlap = pd.read_csv(self.path, delimiter=',', header=None, nrows=300)
            # else:
            #     possibleOverlap = pd.read_csv(self.path,  delimiter = ',', header = None, skiprows=ind*300, nrows=300)
            for i, j in enumerate(trueIndexs):
                tmpList = trueIndexs.copy()
                tmpCoverage = coverage
                tmpPathLen = pathLen
                tmpList[i] = ind    #update temp list with new index
                tmpCoverage -= self.coverage[j]     #remove coverage of path we're replacing
                tmpCoverage += self.coverage[ind]   #add coverage of new path
                # print(tmpPathLen)
                tmpPathLen -= self.pathLen[j]
                # print(tmpPathLen)
                tmpPathLen += self.pathLen[ind]
                # print(tmpPathLen)

                #Todo:
                # tmpOlMatrix = np.subtract(overlapMatrix, self.overlapStorage.iloc[i * 300:((i + 1) * 300), :].to_numpy())
                # tmpOlMatrix = np.add(tmpOlMatrix, possibleOverlap.to_numpy())
                # boolMap = np.where(tmpOlMatrix > 1)
                # total_overlap = np.sum(overlapMatrix[boolMap])



                # print(f'iteration {ind} total_overlap {total_overlap}')
                # overlapDiff = tmpOverLap - overlap      #smaller better
                # covDiff = tmpCoverage - coverage
                # pathDiff = tmpPathLen - pathLen         #smaller better?

                current = tmpPathLen - (tmpCoverage*100)
                # current = tmpPathLen - (tmpCoverage*100) + total_overlap
                if current < bestScore:
                    bestScore = current
                    bisInd = tmpList.copy()
                    bisCov = tmpCoverage
                    bisPath = tmpPathLen
                    #TODO:
                    # matIndex = i

                    # bisMatrix = tmpOlMatrix
                    # print(f'#####%%%%%%%%%%%%%%%%%%%%%%%%### Breaking inner loop ####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###')
                    # print(f'{i} / {len(trueIndexs)}')
            pathLen = bisPath
            coverage = bisCov
            trueIndexs = bisInd
            # self.overlapStorage.iloc[i * 300:((i + 1) * 300), :]
            # overlapMatrix = bisMatrix
            #TODO:
            # self.overlapStorage
        print(f''
              f' \n END coverage: {coverage}'
              f' \n END pathlen: {pathLen}'
              f' \n END Overall Score: {bestScore}'
              f'')
            # pathLen, coverage, overlap, trueIndexs, overlapMatrix = bisPath, bisCov, bisOL, bisInd, bisMatrix
        return bestScore, bisInd

