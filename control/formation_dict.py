import numpy as np

formation_dict_6 = {"T":np.array([[2,0,0],[-2,0,0],[0,0,-2],[0,0,-4],[0,0,-6]]) , "diamond": np.array([[2,2,-2],[2,-2,-2],[-2,-2,-2],[-2,2,-2],[0,0,-4]]), "triangle": np.array([[-3,0,-3],[3,0,-3],[-1.5,0,-1.5],[1.5,0,-1.5],[0,0,-3]]),"waiting":np.zeros([3,5])}
formation_dict_6["T"] = np.transpose(formation_dict_6["T"]*2)
formation_dict_6["diamond"] = np.transpose(formation_dict_6["diamond"]*2)
formation_dict_6["triangle"] = np.transpose(formation_dict_6["triangle"]*2)

formation_dict_9 = {"cube": np.array([[2,2,2],[-2,2,2],[-2,-2,2],[2,-2,2],[2,2,-2],[-2,2,-2],[-2,-2,-2],[2,-2,-2]]), "pyramid": np.array([[2,2,-2],[-2,2,-2],[-2,-2,-2],[2,-2,-2],[4,4,-4],[-4,4,-4],[-4,-4,-4],[4,-4,-4]]), "triangle": np.array([[0,4,-4],[0,0,-2],[0,0,-4],[0,-2,-2],[0,2,-4],[0,2,-2],[0,-4,-4],[0,-2,-4]]),"waiting":np.zeros([3,8])}
formation_dict_9["cube"] = np.transpose(formation_dict_9["cube"]*1)
formation_dict_9["pyramid"] = np.transpose(formation_dict_9["pyramid"]*1)
formation_dict_9["triangle"] = np.transpose(formation_dict_9["triangle"]*1)

formation_dict_18 = {"cube":np.array([[0, 2, 0], [2, 0, 0], [2, 2, 0], [4, 0, 0], [4, 2, 0], [0, 0, 2], [0, 2, 2], [2, 0, 2], [2, 2, 2], [4, 0, 2], [4, 2, 2], [0, 0, 4], [0, 2, 4], [2, 0, 4], [2, 2, 4], [4, 0, 4], [4, 2, 4]]
) , "sphere": np.array([[0.5857864376269, -1.4142135623731, 0.0], [2.0, -2.0, 0.0], [3.4142135623731003, -1.4142135623731, 0.0], [4.0, 0.0, 0.0], [3.4142135623731003, 1.4142135623731, 0.0], [2.0, 2.0, 0.0], [0.5857864376269, 1.4142135623731, 0.0], [1.0, -1.0, -1.4142135623731], [3.0, -1.0, -1.4142135623731], [3.0, 1.0, -1.4142135623731], [1.0, 1.0, -1.4142135623731], [1.0, -1.0, 1.4142135623731], [3.0, -1.0, 1.4142135623731], [3.0, 1.0, 1.4142135623731], [1.0, 1.0, 1.4142135623731], [2.0, 0.0, 2.0], [2.0, 0.0, -2.0]]
), "diamond": np.array([[2.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 2.0, 0.0], [4.0, 4.0, 0.0], [2.0, 4.0, 0.0], [0.0, 4.0, 0.0], [0.0, 2.0, 0.0], [1.0, 1.0, 1.0], [3.0, 1.0, 1.0], [3.0, 3.0, 1.0], [1.0, 3.0, 1.0], [1.0, 1.0, -1.0], [3.0, 1.0, -1.0], [3.0, 3.0, -1.0], [1.0, 3.0, -1.0], [2.0, 2.0, 2.0], [2.0, 2.0, -2.0]]
),"waiting":np.zeros([3,5])}
formation_dict_18["cube"] = np.transpose(formation_dict_18["cube"])
formation_dict_18["sphere"] = np.transpose(formation_dict_18["sphere"])
formation_dict_18["diamond"] = np.transpose(formation_dict_18["diamond"])