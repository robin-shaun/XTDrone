# coding: utf-8
import numpy as np
import random
import math
import cmath
import time
import os
# ----------------------Optimization scheme----------------------------------
# Optimization ideas：
# 1. Increase the convergence factor k；
# 2. Dynamic change of inertia factor W；
# 3. Using PSO local search algorithm(Ring method)
# 4. The probability of position variation is added
# ----------------------Set PSO Parameter---------------------------------


class PSO():
    def __init__(self, uav_num, target_num, targets, vehicles_speed, time_lim):
        self.uav_num = uav_num
        self.dim = target_num
        self.targets = targets
        self.vehicles_speed = vehicles_speed
        self.time_all = time_lim
        self.pN = 2*(self.uav_num+self.dim)  # Number of particles
        self.max_iter = 0  # Number of iterations
        # Target distance list (dim+1）*（dim+1)
        self.Distance = np.zeros((target_num+1, target_num+1))
        self.Value = np.zeros(target_num+1)   # Value list of targets 1*dim+1
        self.Stay_time = []
        # UAV flight speed matrix
        self.w = 0.8
        self.c1 = 2
        self.c2 = 2
        self.r1 = 0.6
        self.r2 = 0.3
        self.k = 0   # Convergence factor
        self.wini = 0.9
        self.wend = 0.4

        self.X = np.zeros((self.pN, self.dim+self.uav_num-1)
                          )  # Position of all particles
        self.V = np.zeros((self.pN, self.dim+self.uav_num-1)
                          )  # Velocity of all particles
        # The historical optimal position of each individual
        self.pbest = np.zeros((self.pN, self.dim+self.uav_num-1))
        self.gbest = np.zeros((1, self.dim+self.uav_num-1))
        # Global optimal position
        self.gbest_ring = np.zeros((self.pN, self.dim+self.uav_num-1))
        # Historical optimal fitness of each individual
        self.p_fit = np.zeros(self.pN)
        self.fit = 0  # Global optimal fitness
        self.ring = []
        self.ring_fit = np.zeros(self.pN)
        # variation parameter
        self.p1 = 0.4  # Probability of mutation
        self.p2 = 0.5  # Proportion of individuals with variation in population
        self.p3 = 0.5  # Proportion of locations where variation occurs
        self.TEST = []
        self.test_num = 0
        self.uav_best = []

        self.time_out = np.zeros(self.uav_num)
        
        self.cal_time = 0
    # ------------------Get Initial parameter------------------

    def fun_get_initial_parameter(self):
        # self.max_iter=1000
        self.max_iter = 40*(self.uav_num+self.dim)
        if self.max_iter > 4100:
            self.max_iter = 4100

        # self.test_num=10000
        # Get Stay_time Arrary & Distance Arrary & Value Arrary
        Targets = self.targets
        self.Stay_time = Targets[:, 3]
        self.Distance = np.zeros((self.dim+1, self.dim+1))
        self.Value = np.zeros(self.dim+1)
        for i in range(self.dim+1):
            self.Value[i] = Targets[i, 2]
            for j in range(i):
                self.Distance[i][j] = (
                    Targets[i, 0]-Targets[j, 0])*(Targets[i, 0]-Targets[j, 0])
                self.Distance[i][j] = self.Distance[i][j] + \
                    (Targets[i, 1]-Targets[j, 1])*(Targets[i, 1]-Targets[j, 1])
                self.Distance[i][j] = math.sqrt(self.Distance[i][j])
                self.Distance[j][i] = self.Distance[i][j]
    # ------------------Transfer_Function---------------------

    def fun_Transfer(self, X):
        # Converting continuous sequence X into discrete sequence X_path
        X1 = X[0:self.dim]
        X_path = []
        l1 = len(X1)
        for i in range(l1):
            m = X1[i]*(self.dim-i)
            m = math.floor(m)
            X_path.append(m)
        # Converting the continuous interpolation sequence X into discrete interpolation sequence X_rank
        X2 = X[self.dim:]
        l1 = len(X2)
        X_rank = []
        for i in range(l1):

            m = X2[i]*(self.dim+1)

            m1 = math.floor(m)
            X_rank.append(m1)
        # Rank and Complement
        c = sorted(X_rank)
        l1 = len(c)
        Rank = []
        Rank.append(0)
        for i in range(l1):
            Rank.append(c[i])
        Rank.append(self.dim)
        # Get Separate_Arrary
        Sep = []
        for i in range(l1+1):
            sep = Rank[i+1]-Rank[i]
            Sep.append(sep)
        return X_path, Sep

    # -------------------Obtain the Real Flight Path Sequence of Particles--------------------------
    def position(self, X):
        Position_All = list(range(1, self.dim+1))
        X2 = []
        for i in range(self.dim):
            m1 = X[i]
            m1 = int(m1)
            X2.append(Position_All[m1])
            del Position_All[m1]
        return X2
    # ---------------------Fitness_Computing Function-----------------------------

    def function(self, X):
        X_path, Sep = self.fun_Transfer(X)

        # Obtain the Real Flight Path Sequence of Particles
        X = self.position(X_path)
        # Get the search sequence of each UAV
        UAV = []
        l = 0
        for i in range(self.uav_num):
            UAV.append([])
            k = Sep[i]
            for j in range(k):
                UAV[i].append(X[l])
                l = l+1

        # Calculate Fitness
        fitness = 0
        for i in range(self.uav_num):
            k = Sep[i]
            t = 0
            for j in range(k):
                m1 = UAV[i][j]

                if j == 0:
                    t = t+self.Distance[0, m1] / \
                        self.vehicles_speed[i]+self.Stay_time[m1]
                else:
                    m1 = UAV[i][j]
                    m2 = UAV[i][j-1]
                    t = t+self.Distance[m1][m2] / \
                        self.vehicles_speed[i]+self.Stay_time[m1]
                if t <= self.time_all:
                    fitness = fitness+self.Value[m1]
        return fitness
    # ----------------------------variation-------------------------------------------

    def variation_fun(self):
        p1 = np.random.uniform(0, 1)  # Probability of mutation
        if p1 < self.p1:
            for i in range(self.pN):
                # Proportion of individuals with variation in population
                p2 = np.random.uniform(0, 1)
                if p2 < self.p2:
                    # Numbers of locations where variation occurs
                    m = int(self.p3*(self.dim+self.uav_num-1))
                    for j in range(m):
                        replace_position = math.floor(
                            np.random.uniform(0, 1)*(self.dim+self.uav_num-1))
                        replace_value = np.random.uniform(0, 1)
                        self.X[i][replace_position] = replace_value
            # Update pbest & gbest
            for i in range(self.pN):
                temp = self.function(self.X[i])
                self.ring_fit[i] = temp
                if temp > self.p_fit[i]:
                    self.p_fit[i] = temp
                    self.pbest[i] = self.X[i]
                    # Update gbest
                    if self.p_fit[i] > self.fit:
                        self.gbest = self.X[i]
                        self.fit = self.p_fit[i]

    # ---------------------Population Initialization----------------------------------

    def init_Population(self):
        # Initialization of position(X), speed(V), history optimal(pbest) and global optimal(gbest)
        for i in range(self.pN):
            x = np.random.uniform(0, 1, self.dim+self.uav_num-1)
            self.X[i, :] = x
            v = np.random.uniform(0, 0.4, self.dim+self.uav_num-1)
            self.V[i, :] = v
            self.pbest[i] = self.X[i]

            tmp = self.function(self.X[i])
            self.p_fit[i] = tmp
            if tmp > self.fit:
                self.fit = tmp
                self.gbest = self.X[i]
        # Calculate the convergence factor k
        phi = self.c1+self.c2
        k = abs(phi*phi-4*phi)
        k = cmath.sqrt(k)
        k = abs(2-phi-k)
        k = 2/k
        self.k = k
        # Initialize ring_matrix
        for i in range(self.pN):
            self.ring.append([])
            self.ring[i].append(i)
        # Initialize test_set
        self.TEST = np.zeros((self.test_num, self.dim+self.uav_num-1))
        for i in range(self.test_num):
            test = np.random.uniform(0, 1, self.dim+self.uav_num-1)
            self.TEST[i, :] = test

    # ----------------------Update Particle Position----------------------------------

    def iterator(self):
        fitness = []
        fitness_old = 0
        k = 0
        for t in range(self.max_iter):
            w = (self.wini-self.wend)*(self.max_iter-t)/self.max_iter+self.wend
            self.w = w
            # Variation
            self.variation_fun()
            l1 = len(self.ring[0])
            # Local PSO algorithm
            # Update ring_arrary
            if l1 < self.pN:
                if not(t % 2):
                    k = k+1
                    for i in range(self.pN):
                        m1 = i-k
                        if m1 < 0:
                            m1 = self.pN+m1
                        m2 = i+k
                        if m2 > self.pN-1:
                            m2 = m2-self.pN
                        self.ring[i].append(m1)
                        self.ring[i].append(m2)
                # Update gbest_ring
                l_ring = len(self.ring[0])
                for i in range(self.pN):
                    fitness1 = 0
                    for j in range(l_ring):
                        m1 = self.ring[i][j]
                        fitness2 = self.ring_fit[m1]
                        if fitness2 > fitness1:
                            self.gbest_ring[i] = self.X[m1]
                            fitness1 = fitness2
                # Update velocity
                for i in range(self.pN):
                    self.V[i] = self.k*(self.w * self.V[i] + self.c1 * self.r1 * (self.pbest[i] - self.X[i])) + \
                        self.c2 * self.r2 * (self.gbest_ring[i] - self.X[i])
                # Update position
                    self.X[i] = self.X[i] + self.V[i]

            # Global PSO algorithm
            else:
                # Update velocity
                for i in range(self.pN):
                    self.V[i] = self.k*(self.w * self.V[i] + self.c1 * self.r1 * (self.pbest[i] - self.X[i])) + \
                        self.c2 * self.r2 * (self.gbest - self.X[i])
                # Update position
                    self.X[i] = self.X[i] + self.V[i]

            # Set position boundary
            for i in range(self.pN):
                for j in range(self.dim+self.uav_num-1):
                    if self.X[i][j] >= 1:
                        self.X[i][j] = 0.999
                    if self.X[i][j] < 0:
                        self.X[i][j] = 0
            # Update pbest & gbest
            for i in range(self.pN):
                temp = self.function(self.X[i])
                self.ring_fit[i] = temp
                if temp > self.p_fit[i]:
                    self.p_fit[i] = temp
                    self.pbest[i] = self.X[i]
                    # Update gbest
                    if self.p_fit[i] > self.fit:
                        self.gbest = self.X[i]
                        self.fit = self.p_fit[i]
                        self.uav_best = self.fun_Data()

            # print
            fitness.append(self.fit)
            if self.fit == fitness_old:
                continue
            else:
                fitness_old = self.fit
        return fitness

    # ---------------------Data_Processing Function---------------------------
    def fun_Data(self):
        X_path, Sep = self.fun_Transfer(self.gbest)
        # Obtain the Real Flight Path Sequence of Particles
        X = self.position(X_path)
        # Get the search sequence of each UAV
        UAV = []
        l = 0
        for i in range(self.uav_num):
            UAV.append([])
            k = Sep[i]
            for j in range(k):
                UAV[i].append(X[l])
                l = l+1
        # Calculate UAV_Out
        UAV_Out = []
        for i in range(self.uav_num):
            k = Sep[i]
            t = 0
            UAV_Out.append([])
            for j in range(k):
                m1 = UAV[i][j]
                if j == 0:
                    t = t+self.Distance[0, m1] / \
                        self.vehicles_speed[i]+self.Stay_time[m1]
                else:
                    m2 = UAV[i][j-1]
                    t = t+self.Distance[m2][m1] / \
                        self.vehicles_speed[i]+self.Stay_time[m1]
                if t <= self.time_all:
                    UAV_Out[i].append(m1)
                    self.time_out[i] = t
        return UAV_Out
    # ---------------------TEST Function------------------------------

    def fun_TEST(self):
        Test_Value = []
        for i in range(self.test_num):
            Test_Value.append(self.function(self.TEST[i]))
        return Test_Value
    # ---------------------Main----------------------------------------

    def run(self):
        print("PSO start, pid: %s" % os.getpid())
        start_time = time.time()
        self.fun_get_initial_parameter()
        self.init_Population()
        fitness = self.iterator()
        end_time = time.time()
        #self.cal_time  = end_time - start_time
        #self.task_assignment = self.uav_best
        print("PSO result:", self.uav_best)
        print("PSO time:", end_time - start_time)
        return self.uav_best, end_time - start_time
        


        # -------------Result-------------------------
if __name__ == '__main__':
    # ----------------------------------TEST--------------------------------
    '''
    Test_Value=my_pso.fun_TEST()
    l1=len(fitness)
    k=0
    Test_Value_out=[]
    for i in range(test_num):
        if Test_Value[i]>fitness[l1-1]:
            k=k+1
            Test_Value_out.append(Test_Value[i])
    print("测试结果超过优化后的目标值的个数是：",k,"个")
    if k > 0:
        print("这些测试结果分别是：",Test_Value_out)
    #print("测试的适应值是",Test_Value)
    env.run(my_pso.uav_best,'pso')
    print(env.vehicles_time)
    print('time_limit:',env.time_lim)
    '''
