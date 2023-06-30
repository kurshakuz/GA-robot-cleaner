# Genetic algorihtm class for genomes purely with integer genes
# will represent real number ranges from x0 to x1, with fixed accuracy in n-steps
import time
import numpy as np
from NeuralNet import Net
# from model import Model
# from view import draw_simulation
# import environments

import matplotlib.pyplot as plt

class GeneticAlgorithm:
    # Jannik + Shyngyskhan
    def __init__(self, pop_size, genome_template, fitness_func, tournament_size, mutation, elitism):
        self.population = []
        self.mutation_rate = mutation
        self.tournament_size = tournament_size
        self.elitism = elitism
        self.fitness_func = fitness_func
        self.genome_template = genome_template
        self.stepsizes = [] # gene stepsizes of values
        self.history = {'avg_fit': [], 'best_fit': [], 'best_cand': [],  'diversity': []}
        for p in range(pop_size):
            rnd_chrom = []
            for (fr, to, n) in genome_template:
                rnd_chrom.append(np.round(np.random.choice(np.linspace(fr, to, n)),8))
                if p == 0:
                    self.stepsizes.append(np.round(np.linspace(fr, to, n)[1] - np.linspace(fr, to, n)[0], 8))
            self.population.append(rnd_chrom)
        self.population = np.array(self.population)
        #print(self.population)
    
    # Jannik
    def nextGeneration(self):        
        # track diversity
        self.determineDiversity()

        # evalutate population
        self.evaluate()

        if self.elitism > 0:
            elite = self.population[:self.elitism]

        # select parents
        self.selection()

        # Reproduction from selected parents
        self.crossover_and_mutate()

        # overwrite elite over first n children
        if self.elitism > 0:
            self.population[:self.elitism] = elite
            

        print("average:" + str(self.avg_fit))
        print("best:" + str(self.best_fit))

        # show new best
        #if len(self.history['best_fit']) == 0 or self.best_fit > np.max(self.history['best_fit']):
            #cleaningPerformance(elite[0], self.timesteps, True)

        self.history['avg_fit'].append(self.avg_fit)
        self.history['best_fit'].append(self.best_fit)
        self.history['best_cand'].append(self.best_cand)

    def determineDiversity(self):
        diversity = 0
        for gene in range(len(self.population[0])):
            tmp = []
            for p in self.population:
                tmp.append(p[gene])
            diversity += np.std(tmp)
        self.history['diversity'].append(diversity)

    # Jannik + Shyngyskhan
    def evaluate(self):
        self.fitness = [self.fitness_func(p) for p in self.population]
        self.fitness = np.array(self.fitness)
        idx = np.argsort(self.fitness)

        # sort from best to worst
        self.fitness = self.fitness[idx[::-1]]
        self.population = self.population[idx[::-1]]

        self.avg_fit = np.average(self.fitness)
        self.best_fit = np.max(self.fitness)
        self.best_cand = self.population[np.argmax(self.fitness)]

    # Jannik
    def selection(self):
        # tournament selection
        selection = []
        k = int(self.tournament_size*len(self.population)) # higher k -> higher selection pressure
        assert k > 1, "Please choose a larger tournament size."

        for _ in range(len(self.population)):
            # pick randomly k individuals from the population
            tournament_pop = np.random.choice(range(len(self.population)), size=k, replace=False)
            winner_idx = min(tournament_pop)  # assuming sorted list!!!
            # winner_idx = tournament_pop[np.argmax(self.fitness[tournament_pop])]
            selection.append(self.population[winner_idx])

        self.population = selection

    # Jannik
    def crossover_and_mutate(self):
        children = []

        # iterate over parents
        for i in range(0, len(self.population), 2):
            # crossover
            for child in self.crossover(self.population[i], self.population[i+1]):
                # mutation
                children.append(self.mutate(child))
        self.population = np.array(children)

    # Jannik
    def mutate(self, child):
        for gene_idx in range(len(child)):
            rnd = np.random.rand()
            if rnd >= (1-self.mutation_rate):
                magnitude_factor = int(np.random.standard_normal()*self.genome_template[gene_idx][-1]/10) # no of steps added/subtracted
                child[gene_idx] = np.round(child[gene_idx]+self.stepsizes[gene_idx]*magnitude_factor, 8)
                
        return child

    # Jannik
    def crossover(self, p0, p1):
        # 1st child, just arithmetic average of both
        #parents = np.array([p0, p1])
        #c1 = np.average((parents/self.stepsizes), axis=0)
        #c1 = np.array([int(c1[i]) for i in range(len(c1))])
        #c1 = c1 * self.stepsizes
        both = np.vstack((p0, p1))
        bitmask = np.array([int(np.round(np.random.random())) for _ in range(len(p0))])
        c1 = [both[bitmask[i]][i] for i in range(len(p0))]
        # limit to range
        c1 = [ np.min((np.max((c1[i], self.genome_template[i][0])),self.genome_template[i][1])) for i in range(len(c1))]
        bitmask = np.array([int(np.round(np.random.random())) for _ in range(len(p0))])
        c2 = [both[bitmask[i]][i] for i in range(len(p0))]
        c2 = [ np.min((np.max((c2[i], self.genome_template[i][0])),self.genome_template[i][1])) for i in range(len(c1))]
        return [c1, c2]
