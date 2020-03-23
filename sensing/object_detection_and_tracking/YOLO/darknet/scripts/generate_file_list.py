import os
import glob

trainPath = "~/catkin_ws/src/darknet/xtdrone/train/"
testPath = "~/catkin_ws/src/darknet/xtdrone/test/"

names = [trainPath+"train.txt", testPath+"test.txt"]
flag = 0

def gen_img_path_list(path):
    dirlist = glob.glob(os.path.join(path, '*.jpg'))
    with open(names[flag], "w") as fp:
        for name in dirlist:
            fp.write(name)
            fp.write("\n")
        fp.close()


gen_img_path_list(trainPath)
flag = 1
gen_img_path_list(testPath)




