# Code usage from: https://machinelearningmastery.com/simple-genetic-algorithm-from-scratch-in-python/

import subprocess
import numpy as np
import os
from numpy.random import randint
from numpy.random import rand
import signal
import sys

dll_bw_min = 0.1
dll_bw_max = 10.0

pll_bw_min = 1.0
pll_bw_max = 100.0

fll_bw_min = 1.0
fll_bw_max = 100.0

population_size = 10

fnum = 0
fname = "population-log-" + str(fnum) + ".txt"
while os.path.exists(fname):
    fnum += 1
    fname = "population-log-" + str(fnum) + ".txt"

log_file = open(fname, "w")
log_file.write("Generation,dll_bw,pll_bw,fll_bw,Score\n")

def signal_handler(sig, frame):
    print('Exiting!')
    log_file.close()
    sys.exit(0)

def compute_score(gen, member):
    # Run simulation
    output = subprocess.check_output(['build/Debug/TrackerSim.exe', str(member[0]), str(member[1]), str(member[2])])
    scores_text = str(output).replace("b'", "").replace("\\r\\n'", "").split(",")
    print(scores_text)

    # Extract scores
    scores = np.zeros(3)
    for i in range(0, 3):
        scores[i] = float(scores_text[i])

    # Compute score
    score = np.average(scores)

    print(gen, member[0], member[1], member[2], score)
    log_file.write(str(gen) + "," + str(member[0]) + "," + str(member[1]) + "," + str(member[2]) + "," + str(score) + "\n")
    return score

# tournament selection
def selection(scores, k=3):
	# first random selection
	selection_ix = randint(len(scores))
	for ix in randint(0, len(scores), k-1):
		# check if better (e.g. perform a tournament)
		if scores[ix] > scores[selection_ix]:
			selection_ix = ix
	return selection_ix


# crossover two parents to create two children
def crossover(p1, p2, r_cross):
    # children are copies of parents by default
    c1, c2 = p1.copy(), p2.copy()
    # check for recombination
    if rand() < r_cross:
        # select crossover point that is not on the end of the string
        pt = randint(1, len(p1))
        # perform crossover
        c1 = [*p1[:pt], *p2[pt:]]
        c2 = [*p2[:pt], *p1[pt:]]
    return [c1, c2]

# mutation operator
def mutation(child, r_mut):
    for i in range(len(child)):
        # check for a mutation
        if rand() < r_mut:
            # reset gene to a random value
            if(i == 0): 
                child[i] = np.random.uniform(dll_bw_min, dll_bw_max)
            elif(i == 1):
                child[i] = np.random.uniform(pll_bw_min, pll_bw_max)
            else:
                child[i] = np.random.uniform(fll_bw_min, fll_bw_max)
    return child

def create_original_population(population_size):	
    # Initial population
    population = np.zeros((population_size, 3))
    for i in range(0, 10):
        # Randomly generate population
        population[i, 0] = np.random.uniform(dll_bw_min, dll_bw_max)
        population[i, 1] = np.random.uniform(pll_bw_min, pll_bw_max)
        population[i, 2] = np.random.uniform(fll_bw_min, fll_bw_max)
    return population

def generation(gen, population):
    population_score = [compute_score(gen, population[i]) for i in range(population_size)]
    parents = [population[selection(population_score)] for _ in range(population_size)]

    # create the next generation
    children = list()
    for i in range(0, population_size, 2):
        # get selected parents in pairs
        p1, p2 = parents[i], parents[i+1]
        # crossover and mutation
        for c in crossover(p1, p2, 0.95):
            # mutation
            c = mutation(c, 0.15)
            # store for next generation
            children.append(c)

    # replace population
    population = np.array(children)
    return population

# Run simulation
signal.signal(signal.SIGINT, signal_handler)
population = create_original_population(population_size)
for i in range(0, 10):
    population = generation(i, population)
log_file.close()
    
