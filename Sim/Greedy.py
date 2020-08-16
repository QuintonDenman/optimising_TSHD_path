"""
Simulated Annealing Class
"""
import random
import time
import os
import pandas as pd
import numpy as np
class Greedy:
    def __init__(self, pathLen, path, coverage, iterationLimit, width, bestMatrices):
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
        self.bestMatrices = bestMatrices
        self.covpercent = 80

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
        noSolution = False
        overlapMatrix = np.zeros((self.width, self.width))
        percent_covered = 0
        while percent_covered <= self.covpercent:
            k=0
            ind = random.randint(0, self.dataDim)
            while(ind in tmpSolution):
                k +=1
                if k >= self.dataDim*5:
                    print("bag too small??????????????????????????????????????????????????????????????")
                    noSolution = True
                    return tmpPathLen, tmpCoverage, tmpSolution, noSolution
                ind = random.randint(0, self.dataDim)
            tmpSolution.append(ind)
            tmpCoverage += ((self.coverage[ind])*(300*300))/100
            tmpPathLen += self.pathLen[ind]
            percent_covered = (tmpCoverage/(300*300))*100
        #TODO
        # for i, chunk in enumerate(pd.read_csv(self.path, delimiter=',', header=None, chunksize=300, index_col=None, low_memory = False)):
        #     for j, k in enumerate(tmpSolution):
        #         if k == i:
        #             arr = chunk.to_numpy()
        #             overlapMatrix = np.add(overlapMatrix, arr)
        #             np.save(self.bestMatrices+str(k), arr)
        # boolMap = np.where(overlapMatrix > 1)
        # total_overlap = np.sum(overlapMatrix[boolMap])

        return tmpPathLen, tmpCoverage, tmpSolution, noSolution #, overlapMatrix, total_overlap

    # def calculateOverlap(self, allPaths):
    #
    #     total = np.zeros(self.width, self.width)
    #     for i, j in enumerate(allPaths):
    #         tmp = pd.allPaths.iloc[j:(j+self.width), :].to_numpy()
    #         total = np.add(tmp, total)
    #     return total
    # def calcBestScore(self, path, cov, over):


    def runGreedy(self):
        # pathLen, coverage, trueIndexs, noSolution, overlapMatrix, overlap = self.randomSolution()
        pathLen, coverage, trueIndexs, noSolution = self.randomSolution()
        if noSolution == True:
            return 900000000, [0]
        # bisMatrix = overlapMatrix
        bestScore = pathLen - coverage #+ overlap
        # bestScore = pathLen - (coverage*100) + overlap
        print(f' '
              f' \n Initial coverage: {(coverage/(self.width*self.width))*100}'
              f' \n Initial coverage: {coverage}'
              f' \n Initial pathlen: {pathLen}'
              f' \n Initial Overall Score: {bestScore}'
              f'')
        while self.iteration < self.iterationLimit:
            self.iteration += 1
            if self.iteration %100 ==0:
                print(f'Starting Greedy iteration {self.iteration} of {self.iterationLimit}')
            ind = random.randint(0, self.dataDim)
            k=0
            while ind in trueIndexs:
                k+=1
                ind = random.randint(0, self.dataDim)
                if k >= self.dataDim*3:
                    print("stopping early bag too small??????????????????????????????????????????????????????????????")
                    print(bisInd)
                    print(len(bisInd))
                    print(self.dataDim)

                    return bestScore, bisInd
            #TODO:
            # try:
            #     possibleOverlapnp = np.load(self.bestMatrices+str(ind))
            # except:
            #     if ind == 0:
            #         possibleOverlap = pd.read_csv(self.path, delimiter=',', header=None, nrows=300)
            #     else:
            #         possibleOverlap = pd.read_csv(self.path,  delimiter = ',', header = None, skiprows=(ind*300)-1, nrows=300)
            reached = False
            for i, j in enumerate(trueIndexs):
                tmpList = trueIndexs.copy()
                tmpCoverage = coverage
                tmpPathLen = pathLen
                tmpList[i] = ind    #update temp list with new index
                tmpCoverage -= ((self.coverage[j])*(300*300))/100   #remove coverage of path we're replacing
                tmpCoverage += ((self.coverage[ind])*(300*300))/100   #add coverage of new path
                # print(tmpPathLen)
                tmpPathLen -= self.pathLen[j]
                # print(tmpPathLen)
                tmpPathLen += self.pathLen[ind]
                # print(tmpPathLen)

                #Todo:
                # strt = time.time()
                # try:
                #     remove = np.load(self.bestMatrices+str(j))
                # except:
                #     if j == 0:
                #         remove = pd.read_csv(self.path, delimiter=',', header=None, nrows=300).to_numpy()
                #     else:
                #         remove = pd.read_csv(self.path,  delimiter = ',', header = None, skiprows=(j*300)-1, nrows=300).to_numpy()
                # tmpOlMatrix = np.subtract(overlapMatrix, remove)
                # try:
                #     tmpOlMatrix = np.add(tmpOlMatrix, possibleOverlapnp)
                # except:
                #     tmpOlMatrix = np.add(tmpOlMatrix, possibleOverlap.to_numpy())
                # boolMap = np.where(tmpOlMatrix > 1)
                # overlap = np.sum(overlapMatrix[boolMap])
                # print("matrix I/O--- %s seconds ---" % (time.time() - strt))


                # print(f'iteration {ind} total_overlap {total_overlap}')
                # overlapDiff = tmpOverLap - overlap      #smaller better
                # covDiff = tmpCoverage - coverage
                # pathDiff = tmpPathLen - pathLen         #smaller better?

                current = tmpPathLen - tmpCoverage #+ overlap
                # current = tmpPathLen - (tmpCoverage*100) + total_overlap
                if current < bestScore:
                    reached = True
                    bestScore = current
                    bisInd = tmpList.copy()
                    bisCov = tmpCoverage
                    bisPath = tmpPathLen
                    # bisMatrix = tmpOlMatrix.copy()


            # strt = time.time()
            if reached and (bisCov/(self.width*self.width))*100 > self.covpercent:
                for index, value in enumerate(bisInd):
                    xList = bisInd.copy()
                    xCoverage = bisCov
                    xPathLen = bisPath
                    xList.pop(index)
                    xCoverage -= ((self.coverage[value])*(300*300))/100
                    xPathLen -= self.pathLen[value]

                    # try:
                    #     remove = np.load(self.bestMatrices + str(value))
                    # except:
                    #     if value == 0:
                    #         remove = pd.read_csv(self.path, delimiter=',', header=None, nrows=300).to_numpy()
                    #     else:
                    #         remove = pd.read_csv(self.path, delimiter=',', header=None, skiprows=(value * 300) - 1,
                    #                              nrows=300).to_numpy()
                    # tmpOlMatrix = np.subtract(bisMatrix, remove)
                    # boolMap = np.where(tmpOlMatrix > 1)
                    # overlap = np.sum(overlapMatrix[boolMap])

                    xScore = xPathLen - xCoverage #+ overlap

                    if xScore < bestScore:
                        bestScore = xScore
                        minInd = xList.copy()
                        minCov = xCoverage
                        minPath = xPathLen
                        # minMatrix = tmpOlMatrix.copy()
            # print("removal of extra cov --- %s seconds ---" % (time.time() - strt))
            try:
                pathLen = minPath
                coverage = minCov
                trueIndexs = minInd.copy()
            except:
                pathLen = bisPath
                coverage = bisCov
                trueIndexs = bisInd.copy()
                # overlapMatrix = minMatrix.copy()
        print(f''
              f' \n END coverage: {(coverage/(self.width*self.width))*100}'
              f' \n END coverage: {coverage}'
              f' \n END pathlen: {pathLen}'
              f' \n END Overall Score: {bestScore}'
              f'')
            # pathLen, coverage, overlap, trueIndexs, overlapMatrix = bisPath, bisCov, bisOL, bisInd, bisMatrix
        return bestScore, bisInd

