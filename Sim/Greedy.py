"""
Simulated Annealing Class
"""
import random
import time
import os
import pandas as pd
import numpy as np
class Greedy:
    def __init__(self, pathLen, path, iterationLimit, width, bestMatrices):
        self.pathLen = pathLen
        self.path = path
        self.width = width
        self.dataDim = len(self.pathLen) - 1
        self.iteration = 0
        self.iterationLimit = iterationLimit
        self.solution = []
        self.solutionCoverage = 0
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
                    return tmpPathLen, tmpCoverage, tmpSolution, noSolution, 0, 0
                ind = random.randint(0, self.dataDim)
            tmpMatrix = np.load(self.path+str(ind)+'.npy')
            boolMap = np.where(tmpMatrix > 0)
            size = tmpMatrix[boolMap].size
            tmpSolution.append(ind)
            tmpCoverage += size
            tmpPathLen += self.pathLen[ind]

            overlapMatrix = np.add(overlapMatrix, tmpMatrix)
            boolMap = np.where(overlapMatrix > 0)
            cover = overlapMatrix[boolMap].size

            percent_covered = (cover/(300*300))*100

        #TODO
        # for i, chunk in enumerate(pd.read_csv(self.path, delimiter=',', header=None, chunksize=300, index_col=None, low_memory = False)):
        #     for j, k in enumerate(tmpSolution):
        #         if k == i:
        #             arr = chunk.to_numpy()
        #             overlapMatrix = np.add(overlapMatrix, arr)
        #             np.save(self.bestMatrices+str(k), arr)
        # boolMap = np.where(overlapMatrix > 1)
        # total_overlap = np.sum(overlapMatrix[boolMap])
        boolMap = np.where(overlapMatrix > 1)
        total_overlap = np.sum(overlapMatrix[boolMap])
        return tmpPathLen, tmpCoverage, tmpSolution, noSolution, overlapMatrix, total_overlap

    # def calculateOverlap(self, allPaths):
    #
    #     total = np.zeros(self.width, self.width)
    #     for i, j in enumerate(allPaths):
    #         tmp = pd.allPaths.iloc[j:(j+self.width), :].to_numpy()
    #         total = np.add(tmp, total)
    #     return total
    # def calcBestScore(self, path, cov, over):


    def runGreedy(self):
        pathLen, coverage, trueIndexs, noSolution, overlapMatrix, overlap = self.randomSolution()
        # pathLen, coverage, trueIndexs, noSolution = self.randomSolution()
        if noSolution == True:
            return 900000000, [0]
        # bisMatrix = overlapMatrix
        bestScore = pathLen - coverage + overlap
        # bestScore = pathLen - (coverage*100) + overlap
        print(f' '
              f' \n Initial coverage: {(coverage/(self.width*self.width))*100}'
              f' \n Initial coverage: {coverage}'
              f' \n Initial pathlen: {pathLen}'
              f' \n Initial Overall Score: {bestScore}'
              f'')
        while self.iteration < self.iterationLimit:
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
            possibleOverlap = np.load(self.path+str(ind)+'.npy')
            boolMap = np.where(possibleOverlap > 0)
            size = possibleOverlap[boolMap].size
            reached = False
            for i, j in enumerate(trueIndexs):
                tmpList = trueIndexs.copy()
                tmpCoverage = coverage
                tmpPathLen = pathLen
                tmpList[i] = ind    #update temp list with new index
                tmpOL = np.load(self.path+str(j)+'.npy')

                tmpBoolMap = np.where(tmpOL > 0)
                tmpSize = tmpOL[tmpBoolMap].size
                tmpCoverage -= tmpSize    #remove coverage of path we're replacing
                tmpPathLen -= self.pathLen[j]
                tmpOlMatrix = np.subtract(overlapMatrix, tmpOL)

                secBoolMap = np.where(tmpOlMatrix > 1)
                overlap = np.sum(tmpOlMatrix[secBoolMap])

                coverMap = np.where(tmpOlMatrix > 0)

                cover = tmpOlMatrix[coverMap].size
                percent_covered = (cover / (300 * 300)) * 100

                if percent_covered >= self.covpercent:
                    current = tmpPathLen - tmpCoverage + overlap
                    bestScore = current
                    bisInd = tmpList.copy()
                    bisCov = tmpCoverage
                    bisPath = tmpPathLen
                    bisMatrix = tmpOlMatrix.copy()
                    break
                tmpCoverage += size
                tmpPathLen += self.pathLen[ind]
                tmpOlMatrix = np.add(tmpOlMatrix, possibleOverlap)
                secBoolMap = np.where(tmpOlMatrix > 1)
                overlap = np.sum(tmpOlMatrix[secBoolMap])

                current = tmpPathLen - tmpCoverage + overlap

                if current < bestScore:
                    reached = True
                    bestScore = current
                    bisInd = tmpList.copy()
                    bisCov = tmpCoverage
                    bisPath = tmpPathLen
                    bisMatrix = tmpOlMatrix.copy()
            if reached == True:
                pathLen = bisPath
                coverage = bisCov
                trueIndexs = bisInd.copy()
                overlapMatrix = bisMatrix.copy()
            self.iteration += 1
        print(f''
              f' \n END coverage: {(coverage/(self.width*self.width))*100}'
              f' \n END coverage: {coverage}'
              f' \n END pathlen: {pathLen}'
              f' \n END Overall Score: {bestScore}'
              f'')
            # pathLen, coverage, overlap, trueIndexs, overlapMatrix = bisPath, bisCov, bisOL, bisInd, bisMatrix
        return bestScore, bisInd

