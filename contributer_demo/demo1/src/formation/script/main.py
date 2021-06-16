from enviroment import Env
from ga1 import GA1
import numpy as np
import matplotlib.pyplot as plt
from ga2 import GA2
if __name__ == '__main__':
    map_size_x = 80
    map_size_y = 80
    map_size = [map_size_x, map_size_y]
    vehicle_num = 3
    target_num = 10
    for i in range(5):
        env = Env(vehicle_num, target_num, map_size, True)
        for j in range(10):
            ga1 = GA1(vehicle_num, env.vehicles_speed, target_num, env.targets, env.time_limit)
            ga2 = GA2(vehicle_num, env.vehicles_speed, target_num, env.targets, env.time_limit)
            ga1_result = ga1.run()
            #print("result:", ga1_result[0])
            ga2_result = ga2.run()

            env.run(ga1_result[0], 'GA', i, j)
            env.run(ga2_result[0], 'adaptGA', i, j)




            #f_cost = np.zeros(21, dtype=np.int32)
            #for i in range(10, 210, 10):
            #    ga1.run(i)
            #    f_cost[i//10]=ga1.reward
            #plt.plot(np.linspace(10, 200, 21),f_cost,'s-',color = 'r',label = 'iteration_influence')
            #plt.savefig("iteration_influence")
            #plt.show()
