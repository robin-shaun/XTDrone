# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import collections
import heapq
import itertools
import time
from xlrd import open_workbook
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = "kaiti"
import mpl_toolkits.mplot3d
from mpl_toolkits.mplot3d import Axes3D

def getDist(pos1, pos2):
    return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))

def heuristic_fun(initparams, k, t):
    distance = getDist(k, t)
    need_full_point = distance / 100 * 4 / 4
    need_comfort_point = distance / 100 * 4 / 4
        
    return need_comfort_point + need_full_point

def draw_path(sheet, path):
    x = sheet.col_values(1)[1:]
    y = sheet.col_values(2)[1:]
    z = sheet.col_values(3)[1:]
    list_length = len(sheet.col_values(4)[1:])
    food_index = [i for i in range(list_length) if sheet.col_values(4)[1:][i] == 1]
    fire_index = [i for i in range(list_length) if sheet.col_values(4)[1:][i] == 0]
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(projection = '3d')
    for i in range(list_length):
        if i in food_index:
            ax.scatter(x[i], y[i], z[i], s=10, c='k', marker='.')
        else:
            ax.scatter(x[i], y[i], z[i], s=10, c='r', marker='.')
    ax.set_title('规划路径结果')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    path = np.array(path)
    ax.plot3D(path[:, 0].tolist(), path[:, 1].tolist(), path[:, 2].tolist(), 'blue')
    plt.show()

class MinheapPQ:
    '''
    堆排序返回代价最小的可行点
    已封装好，使用put向堆加入可行点，使用get获取代价值最小的可行点
    '''
    def __init__(self):
        self.pq = []
        self.nodes = set()
        self.entry_finder = {}
        self.counter = itertools.count()
        self.REMOVED = '<removed-item>'
    
    def put(self, item, priority):
        if item in self.entry_finder:
            self.check_remove(item)
        count = next(self.counter)
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heapq.heappush(self.pq, entry)
        self.nodes.add(item)

    def check_remove(self, item):
        if item not in self.entry_finder:
            return
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED
        self.nodes.remove(item)

    def get(self):
        while self.pq:
            priority, count, item = heapq.heappop(self.pq)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                self.nodes.remove(item)
                return item
        raise KeyError('pop from an empty priority queue')

    def top_key(self):
        return self.pq[0][0]
        
    def enumerate(self):
        return self.pq

    def allnodes(self):
        return self.nodes

class Weighted_A_star(object):
    def __init__(self):
        self.start, self.goal = tuple(np.array([0, 500, 500])), tuple(np.array([1000, 555.67, 442.022]))
        self.g = {self.start:-20,self.goal:np.inf} # 记录从起点到各个点的g，g值是固定的，当通过另一个点能有一个更小的g时改变
        self.Parent = {self.start:self.start}
        self.CLOSED = set()
        self.Path = []
        self.OPEN = MinheapPQ()  # 存储节点及其代价
        self.full = {self.start:10}
        self.comfort = {self.start:10}
        self.OPEN.put(self.start, self.g[self.start] + heuristic_fun(self, self.start, self.goal))  # item, priority = g + h
        self.lastpoint = self.start
        self.lastg = {}
        self.pathparent = {}

    def run(self):
        xt = self.goal
        xi = self.start
        last_xi = xi
        ban_xi = {}
        flag = 0
        while(1):  # 当没有到达终点时循环
            # 找到一个有可行食物点、篝火点的节点作为当前点
            xi = self.OPEN.get()
            distance = getDist(xi, self.Parent[xi])
            full_change = (np.sign(xi[2] - self.Parent[xi][2]) + 5) * distance / 100
            reachable_full, reachable_comfort = self.reachable_point(xi)

            while((self.full[xi] > self.comfort[xi] and len(reachable_comfort) < 2 and xi[0] < 300) \
                or (self.full[xi] <= self.comfort[xi] and len(reachable_full) < 2 and xi[0] < 300) \
                    or (self.full[xi] > self.comfort[xi] and len(reachable_comfort) < 2 and xi[0] < 300) \
                        or (self.full[xi] <= self.comfort[xi] and len(reachable_full) < 2 and xi[0] < 300) \
                            or(self.full[xi] > self.comfort[xi] and len(reachable_comfort) < 1 and xi[0] > 300) \
                                or(self.full[xi] <= self.comfort[xi] and len(reachable_full) < 1 and xi[0] > 300) \
                                    or (xi in self.CLOSED) or xi[0] == 397.14 \
                                        or xi[0] == 604.287 or xi[0] == 632.907 or (xi[0] == 686.951 and self.Parent[xi][0] == 594.039)):
                ban_xi[xi] = self.g[xi] + heuristic_fun(self, xi, self.goal)
                xi = self.OPEN.get()
                distance = getDist(xi, self.Parent[xi])
                full_change = (np.sign(xi[2] - self.Parent[xi][2]) + 5) * distance / 100
                reachable_full, reachable_comfort = self.reachable_point(xi)
            
            # 对当前点的后续处理
            print(xi, self.Parent[xi], self.full[xi], self.comfort[xi], len(reachable_full), len(reachable_comfort))
            self.pathparent[xi] = self.Parent[xi]
            self.CLOSED.add(xi)
            for xz in ban_xi:
                self.OPEN.put(xz, ban_xi[xz])
            ban_xi = {}
            if(self.goal in reachable_full or self.goal in reachable_comfort):
                self.Parent[self.goal] = xi
                self.pathparent[self.goal] = xi
                break


            if(int(100000 * self.full[xi]) > int(100000 * self.comfort[xi])):
                self.process_child(xi, reachable_comfort, False)
            if(int(100000 * self.full[xi]) < int(100000 * self.comfort[xi])):
                self.process_child(xi, reachable_full, True)
            if(int(100000 * self.full[xi]) == int(100000 * self.comfort[xi])):
                self.process_child(xi, reachable_full, True)
                self.process_child(xi, reachable_comfort, False)            

            last_xi = xi
            flag += 1

        self.lastpoint = xi
        self.Path = self.path()
        draw_path(sheet, self.Path)


    def process_child(self, xi, points, isfull):
        # 对当前点后继可行节点的处理
        for xj in list(points.keys()):
            if xj not in self.g:
                self.g[xj] = np.inf
            distance = getDist(xi, xj)

            if isfull:
                self.full[xj] = self.full[xi] + points[xj] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100
                self.comfort[xj] = self.comfort[xi] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100
            else:
                self.full[xj] = self.full[xi] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100
                self.comfort[xj] = self.comfort[xi] + points[xj] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100    

            a = self.g[xi] + 1 - self.full[xj] - self.comfort[xj]
            self.lastg[xj] = self.g[xj]
            self.g[xj] = a
            self.Parent[xj] = xi
            self.OPEN.put(xj, a + heuristic_fun(self, xj, self.goal)) 

    def path(self):
        path = []
        x = self.lastpoint
        path.append([1000, 555.67, 442.022])
        i = 1
        start = self.start
        while x != start:
            path.append(list(x))
            x = self.pathparent[x]
            i += 1
        path.append([0, 500, 500])
        print(path)
        print(i + 1)
        return path

    def reachable_point(self, xi):
        # 输入：当前点坐标
        # 返回：可行食物点坐标及补充量，可行篝火点坐标及补充量
        fulldown = (self.full[xi] + 5) / 4 * 100
        comfortdown = (self.comfort[xi] + 5) / 4 * 100
        fullup = (self.full[xi] + 5) / 6 * 100
        comfortup = (self.comfort[xi] + 5) / 6 * 100

        condition = (content[:, 1] <= xi[0] + fulldown) & (content[:, 1] > xi[0] - fulldown) &\
            (content[:, 2] <= xi[1] + fulldown) & (content[:, 2] >= xi[1] - fulldown) &\
                (content[:, 3] >= xi[2] - fulldown) & (content[:, 3] <= xi[2] + fullup) &\
                    (content[:, 1] <= xi[0] + comfortdown) & (content[:, 1] > xi[0] - comfortdown) &\
                        (content[:, 2] <= xi[1] + comfortdown) & (content[:, 2] >= xi[1] - comfortdown) &\
                            (content[:, 3] >= xi[2] - comfortdown) & (content[:, 3] <= xi[2] + comfortup) & (content[:, 1] != xi[0])
        full_condition = condition & (content[:, 4] == 1)
        comfort_condition = condition & (content[:, 4] == 0)

        reachable_full = dict(zip(list(map(tuple, content[full_condition][:, 1:4])), content[full_condition][:, -1]))
        reachable_comfort = dict(zip(list(map(tuple, content[comfort_condition][:, 1:4])), content[comfort_condition][:, -1]))
        
        for xj in list(reachable_full.keys()):
            distance = getDist(xi, xj)
            xj_full = self.full[xi] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100
            xj_comfort = self.comfort[xi] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100
            if xj == (1000, 555.67, 442.022) and (xj_full <-3 or xj_comfort < -3):
                del reachable_full[xj]
            elif(xj_full < -5 or xj_comfort < -5):
                del reachable_full[xj]
        for xj in list(reachable_comfort.keys()):
            distance = getDist(xi, xj)
            xj_full = self.full[xi] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100
            xj_comfort = self.comfort[xi] - (np.sign(xj[2] - xi[2]) + 5) * distance / 100
            if xj == (1000, 555.67, 442.022) and (xj_full <-3 or xj_comfort < -3):
                del reachable_comfort[xj]
            elif(xj_full < -5 or xj_comfort < -5):
                del reachable_comfort[xj]

        return reachable_full, reachable_comfort


if __name__ == '__main__':

    workbook = open_workbook(r'附件.xlsx')
    sheet = workbook.sheet_by_index(0)
    content = np.array([sheet.row_values(i) for i in range(2, 589)])

    Astar = Weighted_A_star()
    sta = time.time()
    path = Astar.run()
    print(time.time() - sta)
    draw_path(sheet, path)
