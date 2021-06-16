# encoding: utf-8
import numpy as np
import random
import matplotlib.pyplot as plt
import math
import os

UAV_NUM = 33 #无人机数量-1，0 0 0位置不动
POP_SIZE = 200 #种群数量
N_GENERATIONS  = 200 #迭代次数，迭代50轮
CROSSOVER_RATE = 0.8
# MUTATION_RATE = 0.2

formation_dict_my = {}
# formation_dict_my["origin34"] = [ # XTDrone中简化仿真的初始位置
#             # [0, 0, 0],
#             [0, 1, 0], [0, 2, 0], [1, 0, 0], [1, 1, 0], [1, 2, 0],
#             [2, 0, 0], [2, 1, 0], [2, 2, 0], [3, 0, 0], [3, 1, 0], [3, 2, 0],
#             [4, 0, 0], [4, 1, 0], [4, 2, 0], [5, 0, 0], [5, 1, 0], [5, 2, 0],
#             [6, 0, 0], [6, 1, 0], [6, 2, 0], [7, 0, 0], [7, 1, 0], [7, 2, 0], 
#             [8, 0, 0], [8, 1, 0], [8, 2, 0], [9, 0, 0], [9, 1, 0], [9, 2, 0], 
#             [10, 0, 0], [10, 1, 0], [10, 2, 0], [11, 0, 0] ]
formation_dict_my["origin34"] = [ # XTDrone中gazebo仿真的初始位置
            # [0, 3, 0],
                       [3, 3, 0], [6, 3, 0], [9, 3, 0], [12, 3, 0], 
            [0, 6, 0], [3, 6, 0], [6, 6, 0], [9, 6, 0], [12, 6, 0], 
            [0, 9, 0], [3, 9, 0], [6, 9, 0], [9, 9, 0], [12, 9, 0], 
            [0, 12, 0], [3, 12, 0], [6, 12, 0], [9, 12, 0], [12, 12, 0], 
            [0, 15, 0], [3, 15, 0], [6, 15, 0], [9, 15, 0], [12, 15, 0], 
            [0, 18, 0], [3, 18, 0], [6, 18, 0], [9, 18, 0], [12, 18, 0], 
            [0, 21, 0], [3, 21, 0], [6, 21, 0], [9, 21, 0] ]
formation_dict_my["CUBE"] = [   #自定义位置1，立方体
            # [0, 0, 0],
            [0, 0, 0],
            [-15, -15, 15], [-15, 15, 15], [15, 15, 15], [15, -15, 15], #上顶点
            [-15, -15, -15], [-15, 15, -15], [15, 15, -15], [15, -15, -15], #下顶点
            [-15, -5, 15], [-15, 5, 15], [-5, 15, 15], [5, 15, 15], #上棱
            [15, 5, 15], [15, -5, 15], [5, -15, 15], [-5, -15, 15], #上棱
            [-15, -5, -15], [-15, 5, -15], [-5, 15, -15], [5, 15, -15], #下棱
            [15, 5, -15], [15, -5, -15], [5, -15, -15], [-5, -15, -15], #下棱
            [-15, -15, -5], [-15, -15, 5], [15, -15, -5], [15, -15, 5], #中棱
            [-15, 15, -5], [-15, 15, 5], [15, 15, -5], [15, 15, 5] ] #中棱
formation_dict_my["HEART"] = [   #自定义位置2，心形
            # [-0.00000000e+00, 0.00000000e+00, 0.00000000e+00],
            [ 1.00000000e+01, 0.00000000e+00, 0.00000000e+00],
            [ 8.33179844e+00, 1.31962721e+00, 0.00000000e+00],
            [ 6.57163890e+00, 2.13525495e+00, 0.00000000e+00],
            [ 4.86498026e+00, 2.47883127e+00, 0.00000000e+00],
            [ 3.33488737e+00, 2.42293749e+00, 0.00000000e+00],
            [ 2.07106782e+00, 2.07106781e+00, 0.00000000e+00],
            [ 1.12256994e+00, 1.54508498e+00, 0.00000000e+00],
            [-1.12256995e+00, 1.54508497e+00, 0.00000000e+00],
            [-2.07106781e+00, 2.07106781e+00, 0.00000000e+00],
            [-3.33488736e+00, 2.42293751e+00, 0.00000000e+00],
            [-4.86498028e+00, 2.47883124e+00, 0.00000000e+00],
            [-6.57163891e+00, 2.13525491e+00, 0.00000000e+00],
            [-8.33179843e+00, 1.31962724e+00, 0.00000000e+00],
            [-1.00000000e+01, 3.58979303e-08, 0.00000000e+00],
            [-1.14219684e+01,-1.80906211e+00, 0.00000000e+00],
            [-1.24494914e+01,-4.04508498e+00, 0.00000000e+00],
            [-1.29551502e+01,-6.60097872e+00, 0.00000000e+00],
            [-1.28454526e+01,-9.33276749e+00, 0.00000000e+00],
            [-1.20710678e+01,-1.20710678e+01, 0.00000000e+00],
            [-1.06331351e+01,-1.46352549e+01, 0.00000000e+00],
            [-8.62130925e+00,-1.69202720e+01, 0.00000000e+00],
            [-6.15270291e+00,-1.89360728e+01, 0.00000000e+00],
            [-3.28150749e+00,-2.07186232e+01, 0.00000000e+00],
            [-8.84786534e-09,-2.30000000e+01, 0.00000000e+00],
            [ 3.28150747e+00,-2.07186232e+01, 0.00000000e+00],
            [ 6.15270308e+00,-1.89360727e+01, 0.00000000e+00],
            [ 8.62130924e+00,-1.69202720e+01, 0.00000000e+00],
            [ 1.06331351e+01,-1.46352549e+01, 0.00000000e+00],
            [ 1.20710678e+01,-1.20710679e+01, 0.00000000e+00],
            [ 1.28454526e+01,-9.33276750e+00, 0.00000000e+00],
            [ 1.29551502e+01,-6.60097873e+00, 0.00000000e+00],
            [ 1.24494914e+01,-4.04508499e+00, 0.00000000e+00],
            [ 1.14219684e+01,-1.80906212e+00, 0.00000000e+00] ]
formation_dict_my["520"] = [   #自定义位置3，520
            # [0, 0, 0],
            [-3, 0, 0], [3, 0, 0], [3, -3, 0], [3, -6, 0], [0, -6, 0],#2
            [-3, -6, 0], [-3, -9, 0], [-3, -12, 0], [0, -12, 0], [3, -12, 0],
            [-9, 0, 0],  [-12, 0, 0], [-15, 0, 0], [-15, -3, 0], [-15, -6, 0], [-12, -6, 0], #5
            [-9, -6, 0], [-9, -9, 0], [-9, -12, 0], [-12, -12, 0], [-15, -12, 0],
            [9, 0, 0], [12, 0, 0], [15, 0, 0], [15, -3, 0], [15, -6, 0], [15, -9, 0], #0
            [15, -12, 0], [12, -12, 0], [9, -12, 0], [9, -9, 0], [9, -6, 0], [9, -3, 0] ]
formation_dict_my["NUDT"] = [    #自定义位置4，国防科大
            # [0, 0, 0],
            [0, -4, 0], [0, -8, 0], [-1.2, -11, 0], #U
            [-7, 0, 0], [-7, -4, 0], [-7, -8, 0], [-5.8, -11, 0], [-3.5, -12, 0],
            [-12, 0, 0], [-12, -6, 0], [-12, -12, 0], #N
            [-18, 0, 0], [-18, -6, 0], [-18, -12, 0],
            [-16.5, -3, 0], [-15, -6, 0], [-13.5, -9, 0],
            [5, 0, 0], [5, -3, 0], [5, -6, 0], [5, -9, 0], [5, -12, 0], #D
            [8, -0.5, 0], [9.5, -3, 0], [10.5, -6, 0], [9.5, -9, 0], [8, -11.5, 0],
            [15, 0, 0], [18, 0, 0], [21, 0, 0], [18, -4, 0], [18, -8, 0], [18, -12, 0] ]#T
formation_dict_my["八一"] = [     #自定义位置5，八一军徽
            # [0, 0, 0],
            [-2, 0, 0], [2, 0, 0], [-2, -3, 0], [-2.5, -4, 0], [-3.5, -4.5, 0],
            [2, -1, 0], [2.5, -2, 0], [3, -3, 0], [4, -4.5, 0], #八
            [-3, -8, 0], [-1, -8, 0], [1, -8, 0], [3, -8, 0], #一
            [-22, 2, -5], [-13.4, 2, -5], [-5.2, 2, -5], [-2.6, 10, -5], [0, 18, -5],
            [2.6, 10, -5], [5.2, 2, -5], [13.4, 2, -5], [22, 2, -5], #上
            [15.2, -3, -5], [8.4, -8, -5], [11, -16, -5], [13.4, -24, -5], [6.8, -19, -5],
            [0, -14, -5], [-6.8, -19, -5], [-13.6, -24, -5], [-11, -16, -5], [-8.4, -8, -5], [-15.2, -3, -5] ] #下


def plot_uav_pos():
###画无人机位置
    pre_x = np.zeros((1, UAV_NUM)) #初始位置
    pre_y = np.zeros((1, UAV_NUM))
    pre_z = np.zeros((1, UAV_NUM))
    past_x = np.zeros((1, UAV_NUM)) #编队后位置
    past_y = np.zeros((1, UAV_NUM))
    past_z = np.zeros((1, UAV_NUM))
    for i in range(UAV_NUM):
        pre_x[0][i] = UAV_pre[i][0]
        pre_y[0][i] = UAV_pre[i][1]
        pre_z[0][i] = UAV_pre[i][2]
        past_x[0][i] = UAV_past[i][0]
        past_y[0][i] = UAV_past[i][1]
        past_z[0][i] = UAV_past[i][2]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlim(-10, 10)#坐标轴范围
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)
    ax.set_xlabel('X Axes')
    ax.set_ylabel('Y Axes')
    ax.set_zlabel('Z Axes')
    sc = ax.scatter3D(pre_x, pre_y, pre_z, color='r', alpha=0.7, marker='1', linewidth = 10) #初始位置 红
    sc = ax.scatter3D(past_x, past_y, past_z, color='y', alpha=0.7, marker='1', linewidth = 10) #编队后位置 黄
#画无人机位置的辅助线
    #编队后位置
    # DING_LIST = [[-4, -3], [6, -3], [6, 9], [-4, 9]]
    # DingX, DingY = [], []
    # for each in DING_LIST:
    #     DingX.append(each[0])
    #     DingY.append(each[1])
    # DingX.append(DING_LIST[0][0]) #X和Y行程闭环
    # DingY.append(DING_LIST[0][1])
    # ax.plot(DingX, DingY, color = 'black', linestyle = ':')
    # #初始位置
    # r = 5
    # a, b = (0,0)
    # x = np.arange( a-r, a+r+1e-3, 0.1)
    # y = b + np.sqrt( r**2 - (x - a)**2)
    # ax.plot(x, y, color = 'black', linestyle = ':') # 上半部
    # ax.plot(x, -y, color = 'black', linestyle = ':') # 下半部
    # plt.show()


###计算全部无人机从初始位置分别到终止位置的距离
def calu_pre2past_dis():
    distance = []
    for i in range(UAV_NUM):
        single_pre =  UAV_pre[i] 
        pre = np.tile(single_pre, (UAV_NUM ,1 ))#矩阵增广，用于将 原先的位置－编队后的位置
        dis_pre2past = np.sqrt( np.dot( (pre - UAV_past)**2, np.array([ [1],[1],[1] ]) ) ) #计算无人机编队前后的距离
        #编队前位置-编队后位置，再对x和y分别平方，再相加，最后开方。两点间的距离
        dis_pre2past = dis_pre2past.reshape(-1,) #(10,1)转化为(10,)
        distance.append( dis_pre2past ) #每一行代表初始的一架无人机，变换到编队后所有位置的距离
    distance = np.array(distance) # distance[x][y]表示初始第x号飞到编队后第y号位置的距离
    return distance

###初始化种群
def initpop():
    pop = np.zeros((POP_SIZE, UAV_NUM), int)  
    #100行 * 10列，用来表示初始的x架无人机0~9编队后到达的位置的下标0~9
    # print(pop)
    # print(type(pop[1][1]))
    for i in range(POP_SIZE):
        select = [x for x in range( UAV_NUM )] #随机产生种群的每个个体
        random.shuffle(select)  #产生一种随机的分配方案
        pop[i, :] = select
    return pop
# print(pop)

###交叉 变异
def crossover_and_mutation(pop):
    pre_pop = pop.copy() #prechoose是不经过交叉变异的，newchoose后期经过交叉变异
    for row in range(0, (np.shape(pop))[0], 2): # 两个为一组互相交叉
        if CROSSOVER_RATE > random.random():
            gen_1 = pop[row, :].copy()
            gen_2 = pop[row + 1, :].copy()
            select = [x for x in range(UAV_NUM)]
            random.shuffle(select)
            p1 = select[0] #保证两个交叉点的位置不一样
            p2 = select[1]
            p1, p2 = min(p1, p2), max(p1, p2) # 保证 p1 < p2
            cross1, cross2 = gen_1[p1:p2 + 1].copy(), gen_2[p1:p2 + 1].copy() #两个基因的交叉片段
            # print(p1, p2, cross1, cross2)
            for cro_pos_i in range(p2 - p1 + 1): # 遍历交叉片段
                gen_pos1 = np.where(gen_1 == cross2[cro_pos_i])
                gen_1[gen_pos1] = (gen_1[p1:p2 + 1])[cro_pos_i].copy()
                # a = gen_1
            for cro_pos_i in range(p2 - p1 + 1):
                gen_pos2 = np.where(gen_2 == cross1[cro_pos_i])
                gen_2[gen_pos2] = (gen_2[p1:p2 + 1])[cro_pos_i].copy()
                # b = gen_2
            gen_1[p1:p2 + 1] = cross2.copy() #gen2的交叉片段给gen1
            gen_2[p1:p2 + 1] = cross1.copy()
            pop[row, :] = gen_1.copy() #放回到种群中
            pop[row + 1, :] = gen_2.copy()

    for row in range(0, (np.shape(pop))[0]): #遍历每个基因，以一定概率变异
        if MUTATION_RATE > random.random():
            select = [x for x in range(UAV_NUM)]
            random.shuffle(select)
            p1 = select[0] #保证变异的两个点位不一样
            p2 = select[1]
            gen = pop[row, :].copy()
            gen[p1], gen[p2] = gen[p2], gen[p1] #交换两个变异的点位
            pop[row, :] = gen.copy() #放回到种群中
            # print(p1,p2,gen[p1],gen[p2])
    # print(pop,'--------')
    # print(pre_pop)
    pop = np.append(pre_pop, pop, 0)
    #经过交叉变异的种群和未经过的，组合成一个两倍于POPSIZE的种群，下一轮会选择前POPSIZE个
    return pop
    # print(pop.shape)

###计算每一种分配情况，最远距离和最近距离之差
def calu_allocate_dis(pop, dist):
    all_pop_dis = [] #种群100个个体分配情况的时间，列表，行向量

    for allocate_i in pop: #遍历每一种分配方案
        distances = np.zeros(UAV_NUM)
        for i in range(UAV_NUM):
            uav_no_ = allocate_i[i]
            distances[i] = dist[uav_no_][i]
        all_pop_dis.append(max(distances) - min(distances))
    all_pop_dis = np.array(all_pop_dis).reshape(-1, 1) # 列表转列向量
    # unselect_pop_n = all_pop_dis.shape[0]
    return all_pop_dis #每种分配方案的距离
    # print(all_pop_dis)
    
# ###计算每一种分配的总距离
# def calu_allocate_dis(pop, dist):
#     all_pop_dis = [] #种群100个个体分配情况的时间，列表，行向量
#     for allocate_i in pop: #遍历每一种分配方案
#         single_array_sum_time = 0  #一种分配情况的总时间
#         for i in range(UAV_NUM):
#             uav_no_ = allocate_i[i]
#             single_array_sum_time += dist[uav_no_][i]
#         all_pop_dis.append(single_array_sum_time)
#     all_pop_dis = np.array(all_pop_dis).reshape(-1, 1) # 列表转列向量
#     # unselect_pop_n = all_pop_dis.shape[0]
#     return all_pop_dis #每种分配方案的距离


###排序并计算适应度
def get_fitness(all_pop_dis, pop):
    index = np.argsort(all_pop_dis, 0) #升序排列，下标
    all_pop_dis = all_pop_dis[index].reshape(all_pop_dis.shape[0], 1)[0:POP_SIZE, :] 
    #距离升序排列，种群经过交叉变异，数量扩大一倍，这里选择前POP_SIZE个(距离最短的前n个)
    pop = pop[index].reshape(pop.shape[0], UAV_NUM)[0:POP_SIZE, :] # 距离短的前n个种群个体
    '''计算适值和适应度'''
    fit = 1000 - all_pop_dis[:]  # 适应度，时间的反比，时间短=适应度大
    fitplus = np.cumsum(fit).reshape(-1, 1)  # 适值向下叠加
    # print(fitplus[POP_SIZE-1, :])
    fitlevelplus = fitplus[:] / fitplus[POP_SIZE - 1, :]  # 适应度向下叠加
    fitlevelplus = np.insert(fitlevelplus, 0, np.array([0]), 0)  # 在第一行添加0
    # print(fitlevelplus)
    # a = (np.shape(fitlevelplus))[0]
    return fitlevelplus, all_pop_dis, pop

###选择
def select(pop, fitlevelplus, all_pop_dis):
    newchoose_pop = [] #根据适应度选择的新种群
    new_pop_dis = [] #新种群的每个个体，分配方案的距离
    for _ in range(POP_SIZE):
        rand = random.random()
        for row in range(POP_SIZE):
            if rand > fitlevelplus[row, :] and rand < fitlevelplus[row + 1, :]:
                newchoose_pop.append(pop[row, :])
                new_pop_dis.append(all_pop_dis[row, :])
                break
    newchoose_pop = np.array(newchoose_pop)
    new_pop_dis = np.array(new_pop_dis)
    return newchoose_pop, new_pop_dis
    # print(newchoose)
    # print((np.shape(newchoose))[0])
    # print(new_pop_dis)

#将种群和每个分配方案的距离排序
def sortdis(sortpop, dist):
    index = np.argsort(dist, 0) #升序排列，下标
    dist = dist[index].reshape(dist.shape[0], 1)
    #距离升序排列，种群经过交叉变异，数量扩大一倍，这里选择前POP_SIZE个(距离最短的前n个)
    sortpop = sortpop[index].reshape(sortpop.shape[0], UAV_NUM)
    return sortpop, dist

def plot_allocate(best_allc):#best_allc是0~9个编队后位置分给初始的第几号无人机
    plot_uav_pos()
    for i in range(UAV_NUM):
        uav_x = []
        uav_y = []
        uav_z = []
        uav_x.append( UAV_past[i][0] ) #编队后的位置给初始的第几号无人机
        uav_y.append( UAV_past[i][1] ) 
        uav_z.append( UAV_past[i][2] ) 
        uav_id_pre = best_allc[i] 
        uav_x.append( UAV_pre[uav_id_pre][0] ) 
        uav_y.append( UAV_pre[uav_id_pre][1] ) 
        uav_z.append( UAV_pre[uav_id_pre][2] ) 
        plt.plot(uav_x, uav_y, uav_z, color = 'black', linestyle = ':')
    plt.show()

def outputfile(best_allc, form_name):
    allo_index = np.argsort(best_allc, 0) #升序排列，下标。排序后的下标，是初始无人机0~9到达终止位置的第几号
    # 编队结果：6 3 5 9 0 1 7 2 8 4
    # 排序下标：4 5 7 1 9 2 0 6 8 3
    # 0号对应编队第4号位置， 1号对应编队第5号位置
    filename = 'my_formation_dict.py'
    filepath = os.getcwd() + '\\' + filename # 输出的文件所在完整路径
    if not os.path.exists(filepath): #如果文件不存在，则在文件里添加第一行
        with open(filename, 'w') as f:
            f.write( "import numpy as np\n\n" )
            f.write( "formation_dict_my = {}\n" )
    with open(filename, 'a') as f:
        f.write( 'formation_dict_my["' + form_name + '"] = np.array([ ' )
        for i in range(0, UAV_NUM-1):  
            f.write( str(UAV_past[allo_index[i]]) + ",")
        f.write( str(UAV_past[allo_index[UAV_NUM-1]]) + "])\n")
        f.write( 'formation_dict_my["' + form_name + '"] = np.transpose(formation_dict_my["' + form_name + '"]*1)\n')

def plot_info(best_distances, dist, pop):
    best_distances = np.array(best_distances)
    # print(best_distances)
    print(dist[0, :]) #最短距离
    # print(pop[0, :]) #对应的分配方案

    x = [a for a in range(1, N_GENERATIONS+1)]
    y = best_distances
    plt.plot(x, y)
    plt.xlabel('迭代次数')
    plt.ylabel('距离之差')
    plt.rcParams['font.sans-serif'] = ['SimHei']
    plt.show()

def calu_best_dis(best_allc, dis):
    best_dis = 0
    # allo_index = np.argsort(best_allc, 0) #升序排列，下标
    for i in range(UAV_NUM):
        uav_no_ = best_allc[i]
        best_dis += dis[uav_no_][i]
        # pos_past = UAV_past[allo_index[i]]
        # pos_pre = UAV_pre[i]
        # curr_dis = math.sqrt((pos_pre[0] - pos_past[0]) ** 2 + (pos_pre[1] - pos_past[1]) ** 2)
        # best_dis += curr_dis
    print(best_dis)

if __name__ == "__main__":
    dic_i = 0
    for key in formation_dict_my.keys():
        MUTATION_RATE = 0.2 # 每开始新的一轮循环，新的编队
        form_name = key
        UAV_past = formation_dict_my[key]
        if dic_i:
            plot_uav_pos()
            plt.show() #画无人机初始位置和编队后的位置
            distance = calu_pre2past_dis() #计算全部无人机从初始位置分别到终止位置的距离
            pop = initpop() #初始化种群
            best_distances = [] #每一轮最短的距离，用于绘图
            for generation_n in range(N_GENERATIONS): #迭代N代
                if generation_n == int(N_GENERATIONS/2):#迭代到一半时，变异率扩大一倍
                    MUTATION_RATE = MUTATION_RATE*3
                pop = crossover_and_mutation(pop) #先交叉变异，pop扩大为原来的两倍
                all_pop_dis = calu_allocate_dis(pop, distance) #每种分配方案的距离，分配方案的数量
                fitlevelplus, all_pop_dis, pop = get_fitness(all_pop_dis, pop) #种群排序，取前n，求适应度
                # pop, all_pop_dis = select(pop, fitlevelplus, all_pop_dis) #选择
                # sort_pop, sort_pop_dis = sortdis(pop, all_pop_dis)
                best_distances.append(all_pop_dis[0, :]) #当前种群里，最短距离
            plot_allocate(pop[0, :]) #画无人机前后分配线
            outputfile(pop[0, :], form_name) #无人机位置输出到文件
            plot_info(best_distances, all_pop_dis, pop) #绘制最短距离随迭代次数变化曲线
            # calu_best_dis(pop[0, :], distance) 
        UAV_pre  = formation_dict_my[key] 
        dic_i += 1
