"""
Simulated Annealing Class
"""
import random
import math

class SimulatedAnnealing:
    def __init__(self, data, initialTemp, iterationLimit, finalTemp, tempReduction,
                 iterationPerTemp=100, alpha=10, beta=5):
        self.data = data
        self.dataDim = len(data)-1
        self.solution = data[random.randint(0, self.dataDim)]
        self.currTemp = initialTemp
        self.finalTemp = 1e-8 if finalTemp == -1 else finalTemp
        self.iterationPerTemp = iterationPerTemp
        self.alpha = alpha
        self.beta = beta
        self.iteration = 0
        self.iterationLimit = iterationLimit
        self.top10 = [0,0,0,0,0,0,0,0,0,0]

        if tempReduction == "linear":
            self.decrementRule = self.linearTempReduction
        elif tempReduction == "geometric":
            self.decrementRule = self.geometricTempReduction
        elif tempReduction == "slowDecrease":
            self.decrementRule = self.slowDecreaseTempReduction
        else:
            self.decrementRule = tempReduction

    def linearTempReduction(self):
        self.currTemp -= self.alpha

    def geometricTempReduction(self):
        self.currTemp *= (1/self.alpha)

    def slowDecreaseTempReduction(self):
        self.currTemp = self.currTemp / (1 + self.beta * self.currTemp)

    def neighbourhood(self, current):
        x = random.randint(0, 1)
        if current.index() == 0:
            return 1
        elif current.index() == self.dataDim:
            return -1
        elif x == 0:
            return 1
        else:
            return -1

    def saveTop(self, new):
        for i, j in enumerate(self.top10):
            if new > j:
                self.top10.pop(i)
                self.top10.insert(i, new)
                break

    iteration = 0
    def run(self):
        while not iteration < self.iterationLimit or self.currTemp < self.finalTemp:
            iteration =+ 1
            # iterate that number of times
            for i in range(self.iterationPerTemp):
                neighbor = self.neighbourhood(self.solution)
                newSolution = self.data[self.solution.index()+neighbor]
                # get the cost between the two solutions
                self.saveTop(newSolution)
                cost = self.solution - newSolution
                # if the new solution is better, accept it
                if cost <= 0:
                    self.solution = newSolution
                # if the new solution is not better, accept it with a probability of e^(-cost/temp)
                else:
                    if random.uniform(0, 1) < math.exp(-cost / self.currTemp):
                        self.solution = newSolution
            # decrement the temperature
            self.decrementRule()


    def runGreedy(self):
        while self.iteration < self.iterationLimit:
            self.iteration += 1
            ind = random.randint(0, self.dataDim)
            newSolution = self.data[ind]
            # self.saveTop(newSolution)
            # print(f'solution and ne solution: {self.solution}, {newSolution}')
            cost = self.solution - newSolution
            if cost >= 0:
                self.solution = newSolution
                solutionIndex = ind
            # else:
            #     if random.uniform(0, 1) < math.exp(-cost / self.currTemp):
            #         self.solution = newSolution
            #         solutionIndex = ind
            #         print(self.solution)
        return self.solution*-1, solutionIndex

