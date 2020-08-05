"""
Simulated Annealing Class
"""
import random
import math
import itertools
import pandas as pd
import numpy as np
class Greedy:
    def __init__(self, pathLen, overLap, coverage, iterationLimit, width):
        self.pathLen = pathLen
        self.overLap = overLap
        self.width = width
        self.coverage = coverage
        self.dataDim = len(self.coverage) - 1
        self.iteration = 0
        self.iterationLimit = iterationLimit
        self.solution = []
        self.solutionCoverage = 0

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
        print(f'################################ {self.overLap.shape} ################################'
              f'\n {self.overLap.shape[0]/35}'
              f'\n {self.overLap.shape[0]/36}')
        while tmpCoverage <= 50:
            ind = random.randint(0, self.dataDim)
            tmpSolution.append(ind)
            # print(self.coverage)
            tmpCoverage += self.coverage[ind]
            tmpPathLen += self.pathLen[ind]
            # try:
            try:
                overlapMatrix = np.add(overlapMatrix, self.overLap.iloc[ind*35:((ind+1)*35), :].to_numpy())
            except ValueError:
                tmp = self.overLap.iloc[0*35:((1)*35), :].to_numpy()
                print(f'{ind*35}'
                      f'\n {(ind+1)*35}'
                      f'\n {(ind+1)*35-ind*35}'
                      f'\n IND: {ind}'
                      f'\n {tmp.shape}')
            # except ValueError:
            #     # print(ind)
            #     continue
        boolMap = np.where(overlapMatrix > 1)
        total_overlap = np.sum(overlapMatrix[boolMap]) #each initial overlap gets a weight of two, subsquent overlaps increase this by 1

        return tmpPathLen, tmpCoverage, total_overlap, tmpSolution, overlapMatrix

    # def calculateOverlap(self, allPaths):
    #
    #     total = np.zeros(self.width, selff.width)
    #     for i, j in enumerate(allPaths):
    #         tmp = pd.allPaths.iloc[j:(j+self.width), :].to_numpy()
    #         total = np.add(tmp, total)
    #     return total

    def runGreedy(self):
        pathLen, coverage, overlap, trueIndexs, overlapMatrix = self.randomSolution()
        bisPath = pathLen
        bisCov = coverage
        bisOL = overlap
        bisInd = trueIndexs
        bisMatrix = overlapMatrix
        bestScore = pathLen - (coverage*100) + overlap
        print(f' Initial total overlap: {overlap}'
              f' \n Initial coverage: {coverage}'
              f' \n Initial pathlen: {pathLen}'
              f' \n Initial Overall Score: {bestScore}')
        # bestIndexs = trueIndexs
        print(self.coverage)
        print(self.pathLen)
        print(len(self.pathLen))
        while self.iteration < self.iterationLimit:
            self.iteration += 1
            print(f'Starting Greedy iteration {self.iteration} of {self.iterationLimit}')
            ind = random.randint(0, self.dataDim)
            while ind in trueIndexs:
                ind = random.randint(0, self.dataDim)
            print(f'selected index {ind} of {self.dataDim}')
            for i, j in enumerate(trueIndexs):
                tmpList = trueIndexs
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

                tmpOlMatrix = np.subtract(overlapMatrix, self.overLap.iloc[j*35:(j+1)*35, :].to_numpy())

                # boolMap = np.where(tmpOlMatrix > 1)
                # print(tmpOlMatrix)
                # print(tmpOlMatrix[boolMap])
                # total_overlap = np.sum(overlapMatrix[boolMap])
                # print(f'iteration {j} total_overlap {total_overlap}')

                tmpOlMatrix = np.add(tmpOlMatrix, self.overLap.iloc[ind*35:(ind+1)*35, :].to_numpy())
                boolMap = np.where(tmpOlMatrix > 1)
                total_overlap = np.sum(overlapMatrix[boolMap])
                # print(f'iteration {ind} total_overlap {total_overlap}')
                # overlapDiff = tmpOverLap - overlap      #smaller better
                # covDiff = tmpCoverage - coverage
                # pathDiff = tmpPathLen - pathLen         #smaller better?

                current = tmpPathLen - (tmpCoverage*100) + total_overlap
                if current < bestScore:
                    bestScore = current
                    bisInd = tmpList
                    bisCov = tmpCoverage
                    bisPath = tmpPathLen
                    bisMatrix = tmpOlMatrix
                    # print(f'#####%%%%%%%%%%%%%%%%%%%%%%%%### Breaking inner loop ####%%%%%%%%%%%%%%%%%%%%%%%%%%%%%###')
                    # print(f'{i} / {len(trueIndexs)}')
            pathLen  = bisPath
            coverage = bisCov
            overlap = bisOL
            trueIndexs = bisInd
            overlapMatrix = bisMatrix
            # pathLen, coverage, overlap, trueIndexs, overlapMatrix = bisPath, bisCov, bisOL, bisInd, bisMatrix
        return bestScore, bisInd

