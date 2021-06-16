# -*- coding: UTF-8 -*-
"""
Fuzzy PID Controller

* To control all UAV in Gazebo(simulation environment) using MPI

Before running this code, you need to launch your simulation Environment and px4 via:
$ from fuzzy_pid import FuzzyPID()    # 3 UAVs in simulation

And then, you can run this code via:
$ mpiexec -n 3 python formation_sim.py            # 3 UAVs in simulation
"""

import math

#fuzzy table
CIC_FUZZY_TABLE = [[0,1,2,3],
                   [1,2,3,4],
                   [3,4,4,5],
                   [5,5,6,6]]
# kp table
CIC_KP_TABLE = [0.5,0.6,0.8,0.9,1.0,1.1,1.2]

class FuzzyPID():
    def __init__(self):

        self.fuzzy_table = CIC_FUZZY_TABLE
        self.kp_table = CIC_KP_TABLE
        self.ex_range = 5.0
        self.ey_range = 1.0

    def get_kp(self, ex, ey):
        vx = self.get_x_approximation(ex)
        vy = self.get_x_approximation(ey)
        vx1 = math.floor(vx)
        vy1 = math.floor(vy)
        vx2 = vx1 + 1
        vy2 = vy1 + 1
        if vx1 > 3:
            vx1 = 3
        if vy1 > 3:
            vy1 = 3
        if vx2 > 3:
            vx2 = 3
        if vy2 > 3:
            vy2 = 3
        X2Y = (self.fuzzy_table[vx1][vy2] - self.fuzzy_table[vx1][vy1]) * (vy - vy1) + self.fuzzy_table[vx1][vy1]
        X1Y = (self.fuzzy_table[vx2][vy2] - self.fuzzy_table[vx2][vy1]) * (vy - vy1) + self.fuzzy_table[vx2][vy1]
        Y2X = (self.fuzzy_table[vx2][vy1] - self.fuzzy_table[vx1][vy1]) * (vx - vx1) + self.fuzzy_table[vx1][vy1]
        Y1X = (self.fuzzy_table[vx2][vy2] - self.fuzzy_table[vx1][vy2]) * (vx - vx1) + self.fuzzy_table[vx1][vy2]
        kp_approximation = (X2Y + X1Y + Y2X + Y1X)/4.0
        kp1 = math.floor(kp_approximation)
        kp2 = kp1 + 1
        if kp1 > 6:
            kp1 = 6
        if kp2 > 6:
            kp2 = 6
        return (self.kp_table[kp2]-self.kp_table[kp1])*(kp_approximation-kp1)+self.kp_table[kp1]

    def get_x_approximation(self, ex):
        if ex < 0:
            ex = -ex
        return 3*ex/self.ex_range   #divide into 3 parts

    def get_y_approximation(self, ey):
        if ey < 0:
            ey = -ey
        return 3*ey/self.ey_range   #divide into 3 parts


if __name__ == '__main__':
    fuzzy_pid = FuzzyPID()
    kp = fuzzy_pid.get_kp(10,50)
    print(kp)