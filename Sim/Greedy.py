"""
Simulated Annealing Class
"""
import random
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
                    boolMap = np.where(overlapMatrix > 1)
                    total_overlap = np.sum(overlapMatrix[boolMap])
                    return tmpPathLen, tmpCoverage, tmpSolution, noSolution, overlapMatrix, total_overlap
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
        boolMap = np.where(overlapMatrix > 0)
        cover = overlapMatrix[boolMap].size

        percent_covered = (cover / (300 * 300)) * 100
        if noSolution == True:
            # bestScore, bisInd, bisCov, bisPath
            return 900000000, trueIndexs, percent_covered, pathLen, overlap
        # bisMatrix = overlapMatrix
        bestScore = pathLen - coverage + overlap
        # bestScore = pathLen - (coverage*100) + overlap
        print(f' '
              f' \n Initial coverage percentage: {percent_covered}'
              f' \n Initial coverage: {coverage}'
              f' \n Initial pathlen: {pathLen}'
              f' \n Initial Overall Score: {bestScore}'
              f' \n Initial Overlap: {overlap}'
              f'')
        while self.iteration < self.iterationLimit:
            if self.iteration %100 ==0:
                print(f'Starting Greedy iteration {self.iteration} of {self.iterationLimit}')
            ind = random.randint(0, self.dataDim)
            k = 0
            while ind in trueIndexs:
                k += 1
                ind = random.randint(0, self.dataDim)
                if k >= self.dataDim*3:
                    print("stopping early bag too small??????????????????????????????????????????????????????????????")
                    return bestScore, trueIndexs, coverage, pathLen, overlap
            possibleOverlap = np.load(self.path+str(ind)+'.npy')
            boolMap = np.where(possibleOverlap > 0)
            size = possibleOverlap[boolMap].size
            reached = False
            reached2nd = False
            for i, j in enumerate(trueIndexs):
                tmpOL = np.load(self.path + str(j) + '.npy')
                tmpOlMatrix = np.subtract(overlapMatrix, tmpOL)
                tmpOlMatrix = np.add(tmpOlMatrix, possibleOverlap)
                tmpCovMap = np.where(tmpOlMatrix > 0)
                tmpCovSize = tmpOlMatrix[tmpCovMap].size
                tmpCovPerc = (tmpCovSize / (300 * 300)) * 100
                if tmpCovPerc < 79:
                    continue
                tmpList = trueIndexs.copy()
                tmpCoverage = coverage
                tmpPathLen = pathLen
                tmpList[i] = ind    #update temp list with new index

                tmpBoolMap = np.where(tmpOL > 0)
                tmpSize = tmpOL[tmpBoolMap].size
                tmpCoverage -= tmpSize    #remove coverage of path we're replacing
                tmpPathLen -= self.pathLen[j]
                tmpCoverage += size
                tmpPathLen += self.pathLen[ind]
                tmpOlMatrix = np.add(tmpOlMatrix, possibleOverlap)
                secBoolMap = np.where(tmpOlMatrix > 1)
                tmpOverlap = np.sum(tmpOlMatrix[secBoolMap])

                current = tmpPathLen - tmpCoverage + tmpOverlap

                if current < bestScore:
                    reached = True
                    bestScore = current
                    bisInd = tmpList.copy()
                    bisCov = tmpCoverage
                    bisPath = tmpPathLen
                    bisOverlap = tmpOverlap
                    bisMatrix = tmpOlMatrix.copy()
            if reached == True:
                coverMap = np.where(bisMatrix > 0)
                cover = bisMatrix[coverMap].size
                percent_covered = (cover / (300 * 300)) * 100
                while percent_covered > 80:
                    for ite, value in enumerate(bisInd):

                        tmpBisRM = np.load(self.path + str(value) + '.npy')
                        tmpBisOlMatrix = np.subtract(bisMatrix, tmpBisRM)
                        delBisCoverMap = np.where(tmpBisOlMatrix > 0)
                        delBisCover = tmpBisOlMatrix[delBisCoverMap].size
                        delBispercent_covered = (delBisCover / (300 * 300)) * 100
                        if delBispercent_covered < 79:
                            continue
                        tmpBisInd = bisInd.copy()
                        tmpBisCov = bisCov
                        tmpBisPathLen = bisPath
                        tmpBisBoolMap = np.where(tmpBisRM > 0)
                        tmpBisSize = tmpBisRM[tmpBisBoolMap].size
                        tmpBisCov -= tmpBisSize  # remove coverage of path we're replacing
                        tmpBisPathLen -= self.pathLen[value]
                        tmpBisOLboolmap = np.where(tmpBisOlMatrix > 1)
                        tmpBisOverlap = np.sum(tmpBisOlMatrix[tmpBisOLboolmap])
                        del tmpBisInd[ite]
                        current = tmpBisPathLen - tmpBisCov + tmpBisOverlap
                        if current < bestScore:
                            reached2nd = True
                            bestScore = current
                            delBisInd = tmpBisInd.copy()
                            delBisCov = tmpBisCov
                            delBisPath = tmpBisPathLen
                            delBisMatrix = tmpBisOlMatrix.copy()
                            delBisOverlap = tmpBisOverlap
                            percent_covered = delBispercent_covered

            if reached2nd == True:
                trueIndexs = delBisInd.copy
                coverage = delBisCov
                pathLen = delBisPath
                overlap = delBisOverlap
                overlapMatrix = delBisMatrix.copy
            elif reached == True:
                pathLen = bisPath
                coverage = bisCov
                trueIndexs = bisInd.copy()
                overlap = bisOverlap
                overlapMatrix = bisMatrix.copy()
            self.iteration += 1
        finalBoolMap = np.where(overlapMatrix > 0)

        finalCover = overlapMatrix[finalBoolMap].size

        percent_covered = (finalCover / (300 * 300)) * 100
        print(f''
              f' \n END coverage: {coverage}'
              f' \n END pathlen: {pathLen}'
              f' \n END Overall Score: {bestScore}'
              f'')
            # pathLen, coverage, overlap, trueIndexs, overlapMatrix = bisPath, bisCov, bisOL, bisInd, bisMatrix
        return bestScore, bisInd, percent_covered, bisPath, overlap

