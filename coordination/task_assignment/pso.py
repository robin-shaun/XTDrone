# coding: utf-8
import numpy as np
import random
import matplotlib.pyplot as plt
import math
import cmath
from environment import Env
# ----------------------优化方案----------------------------------
# 发现粒子群算法优化结果不好，收敛没有到全局最优
# 优化思路：1. 增加收敛因子k；2. 动态改变惯性因子w

# ----------------------PSO参数设置---------------------------------
class PSO():
    def __init__(self, pN, dim, max_iter,uav_num,Distance,
    v,Value,test_num,time_all):
        self.w = 0.8
        self.c1 = 2
        self.c2 = 2
        self.r1 = 0.6
        self.r2 = 0.3
        self.pN = pN  # 方案数量
        self.dim = dim  # 方案维度
        self.max_iter = max_iter  # 迭代次数
        self.X = np.zeros((self.pN, self.dim))  # 所有粒子的位置
        self.V = np.zeros((self.pN, self.dim))  # 所有粒子的速度
        self.pbest = np.zeros((self.pN, self.dim))  # 个体经历的最佳位置
        self.gbest = np.zeros((1, self.dim))   # 个体经历的全局最佳位置
        self.p_fit = np.zeros(self.pN)  # 每个个体的历史最佳适应值
        self.fit = 0 # 全局最佳适应值
        self.uav_num=uav_num
        self.time_all=time_all
        self.Distance=Distance   # Distance是(dim+1）*（dim+1)的对称矩阵
        self.v=v   # 无人机真正飞行速度 m/s
        self.Value=Value   # 目标位置的价值数组 1*dim
        self.TEST=[]
        self.test_num=test_num
        self.dim1=dim
        self.add_num=0
        self.time_all=time_all
        self.k=0   # 收敛因子
        self.wini=0.9
        self.wend=0.4
    # --------------------取整---------------------------------
    def fun_Ceil(self):
        num1=self.dim/self.uav_num
        num1=math.ceil(num1)
        dim1=num1*self.uav_num
        num1=dim1-self.dim
        self.add_num=num1
        self.dim1=dim1
    # -------------------排序式转排列式--------------------------
    def position(self,X):
        Position_All=list(range(1,self.dim+1))
        X2=[]
        for i in range(self.dim):
            m1=X[i]
            m1=int(m1)
            X2.append(Position_All[m1-1])
            del Position_All[m1-1]
        return X2


    
    
    
    
    
    
    # ---------------------目标函数-----------------------------
    def function(self, X):
    # X 是一个方案，x[i]
        # 由无人机排序式转为排列式
        X=self.position(X)
        # 把X扩充到可以整除的数组
        for i in range(self.add_num):
            X.append(i+self.dim+1)
        # 由方案导出五个无人机各自搜索路径
        UAV=[]
        k=0
        for i in range(self.uav_num):
            UAV.append([])
            for j in range(self.dim1//self.uav_num):
                UAV[i].append(X[k])
                k=k+1
        # 计算某一个方案的目标函数值
        l1=len(UAV[0])
        value=0
        for i in range(self.uav_num):
            t=self.Distance[0][UAV[i][0]]/self.v
            for j in range(l1-1):
                if t<=self.time_all:
                    d1=UAV[i][j]
                    d2=UAV[i][j+1]
                    if d2<=self.dim:
                        distance=self.Distance[d1][d2]
                        t=t+distance/self.v
                        value=value+self.Value[UAV[i][j]-1]
        return value






    # ---------------------初始化种群----------------------------------
    def init_Population(self):
        # 取整
        self.fun_Ceil()
        # 初始化位置和历史最优、全局最优
        for i in range(self.pN):
            for j in range(self.dim):
                self.X[i][j] = int(random.randint(1,self.dim-j))
                #self.V[i][j] = random.uniform(0, 1)
            self.pbest[i] = self.X[i]
            tmp = self.function(self.X[i])
            self.p_fit[i] = tmp
            if tmp > self.fit:
                self.fit = tmp
                self.gbest = self.X[i]
        # 初始化速度
        for i in range(self.pN):
            for j in range(self.dim):
               self.V[i][j] =random.uniform(-0.3*(self.dim-j),0.3*(self.dim-j))
               self.V[i][j]=int(self.V[i][j])
        # 计算收敛因子k
        phi=self.c1+self.c2
        k=abs(phi*phi-4*phi)
        k=cmath.sqrt(k)
        k=abs(2-phi-k)
        k=2/k
        self.k=k
       # 初始化测试集
        for i in range(self.test_num):
            self.TEST.append([])
            for j in range(self.dim):
                self.TEST[i].append(int(random.randint(1,self.dim-j))) 

                #self.V[i][j] = random.uniform(0, 1)
        
        
    # ----------------------更新粒子位置----------------------------------

    def iterator(self):
        fitness = []
        for t in range(self.max_iter):
            # 更新惯性因子w
            w=(self.wini-self.wend)*(self.max_iter-t)/self.max_iter+self.wend   # w的变化规律是线性递减的
            self.w=w   # 用计算出的w更新属性
            for i in range(self.pN):  # 更新gbest\pbest
                temp = self.function(self.X[i])
                if temp > self.p_fit[i]:  # 更新个体最优
                    self.p_fit[i] = temp
                    self.pbest[i] = self.X[i]
                    if self.p_fit[i] > self.fit:  # 更新全局最优
                        self.gbest = self.X[i]
                        self.fit = self.p_fit[i]
               
            for i in range(self.pN):
                # 速度更新
                self.V[i] = self.k*(self.w * self.V[i] + self.c1 * self.r1 * (self.pbest[i] - self.X[i])) + \
                            self.c2 * self.r2 * (self.gbest - self.X[i])
                # 设置速度边界
                for j in range(self.dim):
                    v_max=int(0.3*(self.dim-j))
                    v_min=-v_max
                    self.V[i][j]=int(self.V[i][j])
                    if self.V[i][j] > v_max:
                        self.V[i][j] = v_max
                    if self.V[i][j] < v_min:
                        self.V[i][j] = v_min
                    
                # 位置更新
                self.X[i] = self.X[i] + self.V[i]
                # 设置位置边界
                for j in range(self.dim):
                    self.X[i][j]=int(self.X[i][j])
                    if self.X[i][j] > self.dim-j:
                        self.X[i][j] = self.dim-j
                    if self.X[i][j] < 1:
                        self.X[i][j] = 1

            
            fitness.append(self.fit)
            '''
            print(self.X[0], end=" ")
            print(self.fit)  # 输出最优值
            '''
        self.gbest=self.position(self.gbest)
        return fitness
    # ---------------------数据处理---------------------------
    def fun_Data(self):
        for i in range(self.add_num):
            self.gbest.append(i+self.dim+1)
        UAV=[]
        k=0
        for i in range(self.uav_num):
            UAV.append([])
            for j in range(self.dim1//self.uav_num):
                UAV[i].append(self.gbest[k])
                k=k+1
        l1=len(UAV[0])
        best=[]
        for i in range(self.uav_num):
            t=self.Distance[0][UAV[i][0]]/self.v
            best.append([])
            for j in range(l1-1):
                if t<=self.time_all:
                    best[i].append(UAV[i][j])
                    d1=UAV[i][j]
                    d2=UAV[i][j+1]
                    if d2<=self.dim:
                        distance=self.Distance[d1][d2]
                        t=t+distance/self.v
        for i in range(self.uav_num):
            l1=len(best[i])
            k=0
            for j in range(l1):
                if best[i][k]>self.dim:
                    del best[i][k]
                    k=k-1
                k=k+1
        return best
    
    
    # ---------------------测试------------------------------
    def fun_TEST(self):
        Test_Value=[]
        for i in range(self.test_num):
            Test_Value.append(self.function(self.TEST[i]))
        return Test_Value
# ----------------------程序执行-----------------------
# --------------------定义初始参数---------------------
uav_num = 10
target_num = 45
env = Env(uav_num,target_num,100,1000)
env.targets
env.vehicles_speed
# 定义总搜索时间
time_all=np.random.randint(0.6*10*dim,0.8*10*dim)
Value=[]
for i in range(dim):
    Value.append(np.random.randint(0,200))
Distance=np.zeros((dim+1,dim+1))
for i in range(dim+1):
    for j in range(i+1,dim+1):
        Distance[i][j]=np.random.uniform(200,400)
        Distance[j][i]=Distance[i][j]
# -----------------------PSO优化----------------------
my_pso = PSO(pN, dim, max_iter,uav_num,Distance,
    v,Value,test_num,time_all)
my_pso.init_Population()
fitness = my_pso.iterator()
best=my_pso.fun_Data()
# --------------------------测试---------------------

print("fitness is",fitness)
for i in range(uav_num):
    print("第",i+1,"架无人机的搜索路径为:",best[i])
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

# ------------------测试---------------------






# -------------------画图--------------------
'''
plt.figure(1)
plt.title("Figure1")
plt.xlabel("iterators", size=14)
plt.ylabel("fitness", size=14)
t = np.array([t for t in range(0, 100)])
fitness = np.array(fitness)
plt.plot(t, fitness, color='b', linewidth=3)
plt.show()
'''