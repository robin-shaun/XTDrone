import numpy as np
import random
import time
import os


class GA2():
    def __init__(self, vehicle_num, vehicles_speed, target_num, targets, time_lim):
        # vehicles_speed,targets in the type of narray
        self.vehicle_num = vehicle_num
        self.vehicles_speed = vehicles_speed
        self.target_num = target_num
        self.targets = targets
        self.time_lim = time_lim
        self.map = np.zeros(shape=(target_num+1, target_num+1), dtype=float)
        self.pop_size = 50
        self.p_cross = 0
        self.p_cross1 = 0.9
        self.p_cross2 = 0.6
        self.p_mutate = 0
        self.p_mutate1 = 0.1
        self.p_mutate2 = 0.005
        self.average_ff = 0
        self.max_ff = 1
        for i in range(target_num+1):
            self.map[i, i] = 0
            for j in range(i):
                self.map[j, i] = self.map[i, j] = np.linalg.norm(
                    targets[i, :2]-targets[j, :2])
        self.pop = np.zeros(
            shape=(self.pop_size, vehicle_num-1+target_num-1), dtype=np.int32)
        self.ff = np.zeros(self.pop_size, dtype=float)
        #use vihecle_num-1 numbers to cut the targets into pieces
        #use target_num-1 numbers to code the queue of targets
        #encoder()
        for i in range(self.pop_size):
            for j in range(vehicle_num-1):
                self.pop[i, j] = random.randint(0, target_num)
            for j in range(target_num-1):
                self.pop[i, vehicle_num+j-1] \
                               = random.randint(0, target_num-j-1)
            self.ff[i] = self.fitness(self.pop[i, :])
        self.tmp_pop = np.array([])
        self.tmp_ff = np.array([])
        self.tmp_size = 0

    def fitness(self, gene):
        #to record the break point
        ins = np.zeros(self.target_num+1, dtype=np.int32)
        #to decode the queue of targets
        seq = np.zeros(self.target_num, dtype=np.int32)
        #ins[self.target_num] = 1
        for i in range(self.vehicle_num-1):
            ins[gene[i]] += 1
        rest = np.array(range(1, self.target_num+1))
        for i in range(self.target_num-1):
            seq[i] = rest[gene[i+self.vehicle_num-1]]
            rest = np.delete(rest, gene[i+self.vehicle_num-1])
        seq[self.target_num-1] = rest[0]
        vehicle_num_i = 0  # index of vehicle
        pre = 0  # index of the last target
        post = 0  # index of instant point
        cost = 0
        reward = 0
        while vehicle_num_i < self.vehicle_num:
            if ins[post] > 0:
                vehicle_num_i += 1
                ins[post] -= 1
                pre = 0
                cost = 0
            else:
                cost += self.targets[pre, 3]
                time_cost = self.map[pre, seq[post]]/self.vehicles_speed[vehicle_num_i]
                cost += time_cost
                if cost < self.time_lim:
                    reward += self.targets[seq[post], 2]
                pre = seq[post]
                #print("seq[post]",pre)
                post += 1
                if post > self.target_num-1:
                    break
        return reward

    def selection(self):
        roll = np.zeros(self.tmp_size, dtype=float)
        roll[0] = self.tmp_ff[0]
        for i in range(1, self.tmp_size):
            roll[i] = roll[i-1]+self.tmp_ff[i]
        for i in range(self.pop_size):
            xx = random.uniform(0, roll[self.tmp_size-1])
            j = 0
            while xx > roll[j]:
                j += 1
            self.pop[i, :] = self.tmp_pop[j, :]
            self.ff[i] = self.tmp_ff[j]
            # adaptive ag
        self.average_ff = np.mean(self.ff)
        self.max_ff = max(self.ff)
        if self.ff[i] >= self.average_ff:
            self.p_mutate = self.p_mutate1 - \
                            (self.p_mutate1 - self.p_mutate2) * (self.max_ff - self.ff[i]) / (
                                        self.max_ff - self.average_ff)
        else:
            self.p_mutate = self.p_mutate1

    def crossover(self):
        new_pop = []
        new_ff = []
        new_size = 0
        ff_bigger = 0
        for i in range(0, self.pop_size, 2):
            #adaptive ag
            ff_bigger = max(self.ff[i], self.ff[i+1])
            if ff_bigger >= self.average_ff:
                self.p_cross = self.p_cross1 -\
                               (self.p_cross1-self.p_cross2)/(self.max_ff-self.average_ff)
            else:
                self.p_cross = self.p_cross1
            if random.random() < self.p_cross:
                x1 = random.randint(0, self.vehicle_num-2)
                x2 = random.randint(0, self.target_num-2)+self.vehicle_num
                g1 = self.pop[i, :]
                g2 = self.pop[i+1, :]
                g1[x1:x2] = self.pop[i+1, x1:x2]
                g2[x1:x2] = self.pop[i, x1:x2]
                new_pop.append(g1)
                new_pop.append(g2)
                new_ff.append(self.fitness(g1))
                new_ff.append(self.fitness(g2))
                new_size += 2
        self.tmp_size = self.pop_size+new_size
        self.tmp_pop = np.zeros(
            shape=(self.tmp_size, self.vehicle_num-1+self.target_num-1), dtype=np.int32)
        self.tmp_pop[0:self.pop_size, :] = self.pop
        self.tmp_pop[self.pop_size:self.tmp_size, :] = np.array(new_pop)
        self.tmp_ff = np.zeros(self.tmp_size, dtype=float)
        self.tmp_ff[0:self.pop_size] = self.ff
        self.tmp_ff[self.pop_size:self.tmp_size] = np.array(new_ff)


    def mutation(self):
        for i in range(self.tmp_size):
            flag = False
            for j in range(self.vehicle_num-1):
                if random.random() < self.p_mutate:
                    self.tmp_pop[i, j] = random.randint(0, self.target_num)
                    flag = True
            for j in range(self.target_num-1):
                if random.random() < self.p_mutate:
                    self.tmp_pop[i, self.vehicle_num+j-1
                                 ] = random.randint(0, self.target_num-j-1)
                    flag = True
            if flag:
                self.tmp_ff[i] = self.fitness(self.tmp_pop[i, :])



    def run(self):
        #print("GA start, pid: %s" % os.getpid())
        start_time = time.time()
        gene = np.zeros(
            shape=(1, self.vehicle_num+self.target_num-1), dtype=np.int32)
        cut = 0
        count = 0

        while count < 300:
            self.crossover()
            self.mutation()
            self.selection()
            new_cut = self.tmp_ff.max()
            if cut < new_cut:
                cut = new_cut
                count = 0
                gene = self.tmp_pop[np.argmax(self.tmp_ff)]
            else:
                count += 1

        ins = np.zeros(self.target_num + 1, dtype=np.int32)
        seq = np.zeros(self.target_num, dtype=np.int32)
        #ins[self.target_num] = 1
        for i in range(self.vehicle_num-1):
            ins[gene[i]] += 1
        rest = np.array(range(1, self.target_num + 1))
        for i in range(self.target_num - 1):
            seq[i] = rest[gene[i + self.vehicle_num-1]]
            rest = np.delete(rest, gene[i + self.vehicle_num-1])
        seq[self.target_num - 1] = rest[0]
        #print('sequence : ', seq)
        #print('ins[]: ', ins)
        task_assignment = [[] for i in range(self.vehicle_num)]
        i = 0  # index of vehicle
        pre = 0  # index of last target
        post = 0  # index of ins/seq
        t = 0
        reward = 0
        while i < self.vehicle_num:
            if ins[post] > 0:
                i += 1
                ins[post] -= 1
                pre = 0
                t = 0
            else:
                t += self.targets[pre, 3]
                past = self.map[pre, seq[post]] / self.vehicles_speed[i]
                t += past
                task_assignment[i].append(seq[post])
                if t < self.time_lim:
                    reward += self.targets[seq[post], 2]
                pre = seq[post]
                post += 1
                if post > self.target_num - 1:
                    break
        self.reward = reward
        # print("GA result2:", task_assignment)
        end_time = time.time()
        # print("GA time:", end_time - start_time)
        # print("total reward2:", reward)
        return task_assignment, end_time - start_time

