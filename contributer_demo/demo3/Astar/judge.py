from xlrd import open_workbook
import numpy as np
import copy

def getDist(pos1, pos2):
    return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))

def judge_path(path):
    full = 10
    comfort = 10
    for i in range(1, len(path)):
        distance = getDist(path[i], path[i - 1])
        full = full - (np.sign(path[i][2] - path[i-1][2]) + 5) * distance / 100
        comfort = comfort - (np.sign(path[i][2] - path[i-1][2]) + 5) * distance / 100
        if(full <= -5 or comfort <= -5):
            return 0
        if(content[np.where(content[:, 1] == path[i][0]), 4] == 0):
            comfort = comfort + content[np.where(content[:, 1] == path[i][0]), 5]
        else:
            full = full + content[np.where(content[:, 1] == path[i][0]), 5]
    if(full > -3 and comfort > -3):
        if(len(path) == 22):
            if(path not in all_paths):
                all_paths.append(path)
        return 1
    else:
        return 0


def compute(all_paths):
    for w in range(len(all_paths)):

        thispath = all_paths[w]
        print('**********************************************')
        print(len(thispath))
        print(thispath)
        path = thispath

        full = 10
        comfort = 10
        for i in range(1, len(thispath)):
            distance = getDist(path[i], path[i - 1])
            full = full - (np.sign(path[i][2] - path[i-1][2]) + 5) * distance / 100
            comfort = comfort - (np.sign(path[i][2] - path[i-1][2]) + 5) * distance / 100
            print(full, comfort)
            if(content[np.where(content[:, 1] == path[i][0]), 4] == 0):
                comfort = comfort + content[np.where(content[:, 1] == path[i][0]), 5]
            else:
                full = full + content[np.where(content[:, 1] == path[i][0]), 5]

        print(full, comfort)


if __name__ == '__main__':

    all_paths = []
    workbook = open_workbook(r'附件.xlsx')
    sheet = workbook.sheet_by_index(0)
    content = np.array([sheet.row_values(i) for i in range(1, 589)])

    path = [[0, 500, 500], [49.2701, 501.378, 474.378], [66.4539, 493.45, 485.579], [205.93, 530.015, 462.304], [325.373, 562.813, 450.432], [341.06, 554.258, 440.06], [396.166, 557.686, 463.285], [414.847, 550.557, 458.839], [432.744, 544.693, 464.194], [466.047, 540.082, 447.639], [497.857, 539.374, 473.068], [518.037, 544.067, 478.518], [594.039, 532.547, 476.9], [649.996, 543.743, 471.353], [668.799, 533.409, 475.24], [686.951, 534.936, 468.021], [703.063, 520.512, 467.263], [739.278, 513.132, 452.663], [758.889, 512.304, 450.352], [803.157, 544.765, 446.707], [866.403, 557.739, 448.524], [904.524, 552.094, 445.804], [965.271, 559.438, 441.67], [979.678, 552.128, 452.517], [1000, 555.67, 442.022]]
    for k in range(0, len(path) - 2):
        index1 = int(np.where(content[:, 1] == path[k][0])[0]) + 1
        index2 = int(np.where(content[:, 1] == path[k + 2][0])[0])
        change_nodes = content[index1:index2, 1:4].tolist()
        for j in range(len(change_nodes)):
            path_copy1 = path.copy()
            path_copy1[k+1] = change_nodes[j]
            flag = judge_path(path_copy1)
            if(flag == 0):
                continue

            for z in range(1, len(path_copy1) - 1):
                pathj = path_copy1.copy()
                del pathj[z]
                flag = judge_path(pathj)
                if(flag == 0):
                    continue

                for a in range(1, len(pathj) - 1):
                    patha = pathj.copy()
                    del patha[a]
                    flag = judge_path(patha)
                    if(flag == 0):
                        continue

                    for b in range(1, len(patha) - 1):
                        pathb = patha.copy()
                        del pathb[b]
                        judge_path(pathb)
                        if(flag == 0):
                            continue
    
    print('************************************************************')
    for i in all_paths:
        print(i)
    print(len(all_paths))
    compute(all_paths)





            

            


