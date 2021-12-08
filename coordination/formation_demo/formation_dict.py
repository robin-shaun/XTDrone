import numpy as np

formation_dict_6 = {"origin":np.array([[3,0,0],[0,3,0],[3,3,0],[0,6,0],[3,6,0]]),"T":np.array([[4,0,0],[2,0,0],[2,0,-2],[2,0,-4],[2,0,-6]]) , "diamond": np.array([[2,2,-2],[2,-2,-2],[-2,-2,-2],[-2,2,-2],[0,0,-4]]), "triangle": np.array([[-3,0,-3],[3,0,-3],[-1.5,0,-1.5],[1.5,0,-1.5],[0,0,-3]])}
formation_dict_6["origin"] = np.transpose(formation_dict_6["origin"])
formation_dict_6["T"] = np.transpose(formation_dict_6["T"])
formation_dict_6["diamond"] = np.transpose(formation_dict_6["diamond"])
formation_dict_6["triangle"] = np.transpose(formation_dict_6["triangle"])

formation_dict_9 = {"origin":np.array([[3,0,0],[6,0,0],[0,3,0],[3,3,0],[6,3,0],[0,6,0],[3,6,0],[6,6,0]]), "cube": np.array([[2,2,2],[-2,2,2],[-2,-2,2],[2,-2,2],[2,2,-2],[-2,2,-2],[-2,-2,-2],[2,-2,-2]]), "pyramid": np.array([[2,2,-2],[-2,2,-2],[-2,-2,-2],[2,-2,-2],[4,4,-4],[-4,4,-4],[-4,-4,-4],[4,-4,-4]]), "triangle": np.array([[0,4,-4],[0,0,-2],[0,0,-4],[0,-2,-2],[0,2,-4],[0,2,-2],[0,-4,-4],[0,-2,-4]])}
formation_dict_9["origin"] = np.transpose(formation_dict_9["origin"])
formation_dict_9["cube"] = np.transpose(formation_dict_9["cube"])
formation_dict_9["pyramid"] = np.transpose(formation_dict_9["pyramid"])
formation_dict_9["triangle"] = np.transpose(formation_dict_9["triangle"])

formation_dict_18 = {"origin":np.array([[3,0,0],[6,0,0],[0,3,0],[3,3,0],[6,3,0],[0,6,0],[3,6,0],[6,6,0],[0,9,0],[3,9,0],[6,9,0],[0,12,0],[3,12,0],[6,12,0],[0,15,0],[3,15,0],[6,15,0]]), "cuboid":2*np.array([[0, 2, 0], [2, 0, 0], [2, 2, 0], [4, 0, 0], [4, 2, 0], [0, 0, 2], [0, 2, 2], [2, 0, 2], [2, 2, 2], [4, 0, 2], [4, 2, 2], [0, 0, 4], [0, 2, 4], [2, 0, 4], [2, 2, 4], [4, 0, 4], [4, 2, 4]]
) , "sphere": 3*np.array([[0.5857864376269, -1.4142135623731, 0.0], [2.0, -2.0, 0.0], [3.4142135623731003, -1.4142135623731, 0.0], [4.0, 0.0, 0.0], [3.4142135623731003, 1.4142135623731, 0.0], [2.0, 2.0, 0.0], [0.5857864376269, 1.4142135623731, 0.0], [1.0, -1.0, -1.4142135623731], [3.0, -1.0, -1.4142135623731], [3.0, 1.0, -1.4142135623731], [1.0, 1.0, -1.4142135623731], [1.0, -1.0, 1.4142135623731], [3.0, -1.0, 1.4142135623731], [3.0, 1.0, 1.4142135623731], [1.0, 1.0, 1.4142135623731], [2.0, 0.0, 2.0], [2.0, 0.0, -2.0]]
), "diamond": 2*np.array([[2.0, 0.0, 0.0], [4.0, 0.0, 0.0], [4.0, 2.0, 0.0], [4.0, 4.0, 0.0], [2.0, 4.0, 0.0], [0.0, 4.0, 0.0], [0.0, 2.0, 0.0], [1.0, 1.0, 1.0], [3.0, 1.0, 1.0], [3.0, 3.0, 1.0], [1.0, 3.0, 1.0], [1.0, 1.0, -1.0], [3.0, 1.0, -1.0], [3.0, 3.0, -1.0], [1.0, 3.0, -1.0], [2.0, 2.0, 2.0], [2.0, 2.0, -2.0]]
)}
formation_dict_18["origin"] = np.transpose(formation_dict_18["origin"])
formation_dict_18["cuboid"] = np.transpose(formation_dict_18["cuboid"])
formation_dict_18["sphere"] = np.transpose(formation_dict_18["sphere"])
formation_dict_18["diamond"] = np.transpose(formation_dict_18["diamond"])