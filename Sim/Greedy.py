"""
Simulated Annealing Class
"""
import random
import math
import itertools
class Greedy:
    def __init__(self, pathLen, overLap, coverage, iterationLimit):
        self.pathLen = pathLen
        self.overLap = overLap
        self.coverage = coverage
        self.dataDim = len(self.coverage - 1)
        self.iteration = 0
        self.iterationLimit = iterationLimit
        self.solution = []
        self.solutionCoverage = 0

    def intersectionLists(mainLst):
        '''Expects a list of lists [[], [], ..., []]'''
        count = 0
        for a, b in itertools.combinations(mainLst, 2):
            lst3 = [list(filter(sublist1, sublist2)) for sublist1, sublist2 in zip(a, b)]
            count += len(lst3)
            return count


    def randomSolution(self):
        tmpCoverage = 0
        tmpPathLen = 0
        tmpSolution = []
        while tmpCoverage <= 90:
            ind = random.randint(0, self.dataDim)
            tmpSolution.append(ind)
            tmpCoverage += self.coverage[ind]
            tmpPathLen += self.pathLen[ind]
        tmpOverLap = self.intersectionLists(tmpSolution)
        return tmpPathLen, tmpCoverage, tmpOverLap, tmpSolution


    def runGreedy(self):
        while self.iteration < self.iterationLimit:
            self.iteration += 1
            self.solution.append(self.randomSolution())


