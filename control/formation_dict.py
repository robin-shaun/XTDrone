import numpy as np

formation_dict_6 = {"T":np.array([[2,0,2],[-2,0,2],[0,0,2],[0,0,-2],[0,0,-4]]) , "diamond": np.array([[2,2,-2],[2,-2,-2],[-2,-2,-2],[-2,2,-2],[0,0,-4]]), "triangle": np.array([[-3,0,0],[3,0,0],[-1.5,0,1.5],[1.5,0,1.5],[0,0,3]]),"waiting":np.zeros([3,5])}
formation_dict_6["T"] = np.transpose(formation_dict_6["T"])
formation_dict_6["diamond"] = np.transpose(formation_dict_6["diamond"])
formation_dict_6["triangle"] = np.transpose(formation_dict_6["triangle"])

formation_dict_9 = {"cube": np.array([[2,2,2],[-2,2,2],[-2,-2,2],[2,-2,2],[2,2,-2],[-2,2,-2],[-2,-2,-2],[2,-2,-2]]), "pyramid": np.array([[2,2,-2],[-2,2,-2],[-2,-2,-2],[2,-2,-2],[4,4,-4],[-4,4,-4],[-4,-4,-4],[4,-4,-4]]), "triangle": np.array([[0,4,-4],[0,0,-2],[0,0,-4],[0,-2,-2],[0,2,-4],[0,2,-2],[0,-4,-4],[0,-2,-4]]),"waiting":np.zeros([3,8])}
formation_dict_9["cube"] = np.transpose(formation_dict_9["cube"])
formation_dict_9["pyramid"] = np.transpose(formation_dict_9["pyramid"])
formation_dict_9["triangle"] = np.transpose(formation_dict_9["triangle"])