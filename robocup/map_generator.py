import random
import math
import os

# order:1_66,1_67,1_146,1_146_clone,2_71,2_125,2_126,3_68,3_156,3_157,3_158,73(gas-station),93(fast-food)
# 'gas_station_73','fast_food_93','house_1_66','house_1_67','house_1_146','house_1_146_clone','house_2_71','house_2_125','house_2_126','house_3_68','house_3_156','house_3_157','house_3_158']
# 'house_1_146','house_3_156','house_3_157','house_3_158','gas_station_73','fast_food_93','house_1_66','house_1_67','house_1_146_clone','house_2_71','house_2_125','house_2_126','house_3_68']
# size[x,y] of them

## file path
output_path = os.path.expanduser('~/PX4_Firmware/Tools/sitl_gazebo/worlds/')
rover_num = 20


def rand_x(num):
    correct=[[0,0],[5,5],[5,5],[-15,15],[15,-15],[0,0],[2,2]]
    if num > 0.33 and num < 0.66:
        x = random.uniform(61.3+correct[map_num][0], 95.5)
    elif num > 0.66:
        x = random.uniform(3.55, 30.8-correct[map_num][1])
    else:
        x = random.uniform(-30.7, -29.7)
    return x


def create_point():
    center_box = [[-27.5, 25.5], [-33.4, -21.4], [-28.2, -33], [-25.8, -19.4]]
    center_box_judge = [[86.6, -30], [-27, 14.65], [15.9, -13.6], [5.6, 35.7]]
    if map_num==6:
        center_box = [[-28.64, 25.56], [-32.5, 1.8], [-28.2, -22.7], [-28.1, -33]]
        center_box_judge = [[86.6, -30], [-27, 14.65], [14.2, -20.6], [5.6, 35.7]]
    size_box_judge = [[16, 7], [1, 1], [2, 1], [2, 1], [20, 30], [30, 17], [16, 12], [16, 12], [16, 16], [11, 9],
                      [11, 9], [11, 9], [17, 6]]
    in_obstacle = True
    count=0
    find_solution=True
    for m in range(9):
        while in_obstacle and count<1000:
            if m == 0:
                py = random.uniform(0, 1)
                if py > 0.5:
                    y = -32#map_num=6时，y=1.4,map_num=5时，y=39
                else:
                    y = 14
                if map_num==6:
                    y=18
                if map_num==5:
                    y=-18
                p = random.uniform(0.33, 1)
                x = int(rand_x(p))
            elif m == 1:
                y = int(random.choice((-1, 1)) * random.uniform(13, 31))
                if map_num==6:
                    y=random.uniform(-31,6);
                    if y >=-22:
                        y+=25
                if map_num==5:
                    y=random.uniform(-31,6);
                    if y >=-2:
                        y+=25
                p = random.uniform(0.33, 1)
                x = int(rand_x(p))
            else:
                y = int(random.choice((-1, 1)) * random.uniform(14, 30))
                p = random.uniform(0.33, 1)
                x = int(rand_x(p))
                if map_num==6:
                    y=random.uniform(-30,3);
                    if y >=-23:
                        y+=27
                if map_num==5:
                    y=random.uniform(-30,3);
                    if y >=-3:
                        y+=27
            box_len = len(center_box_judge)
            for i in range(box_len):
                if (abs(x - center_box_judge[i][0]) > (size_box_judge[i][0] + size_box_judge[box_len][0]) / 2 + 3) or (
                        abs(y - center_box_judge[i][1]) > (size_box_judge[i][1] + size_box_judge[box_len][1]) / 2 + 3):
                    in_obstacle = False
                else:
                    in_obstacle = True
                    count=count+1
                    break
        if count==1000:
            find_solution=False
        center_box_judge.append([x, y])
        center_box.append([x, y])
        in_obstacle = True
    return center_box,find_solution


def obstacle_list(center_list):
    # generate obstacle list for human avoidance
    size_box = [[16, 12], [5, 17], [17, 5], [5, 17], [20, 15], [30, 25], [16, 12], [18, 18], [18, 18], [11, 9], [11, 9],
                [11, 9], [14, 5], [2, 2]]
    black_box = [[[-35.375, -34.825], [-4, -3]], [[-3.275, -2.725], [-4, -3]], [[20.725, 21.275], [-4, -3]], \
                 [[57.725, 58.275], [-4, -3]], [[83.725, 84.275], [-4, -3]], [[-25.475, -24.925], [3, 4]],
                 [[8.725, 9.275], [3, 4]], \
                 [[26.725, 27.275], [3, 4]], [[70.725, 71.275], [3, 4]], [[90.225, 90.775], [3, 4]]]
    for i in range(len(center_list)):
        j = i
        if i > len(size_box) - 1:
            j = len(size_box) - 1
        x_min = center_list[i][0] - size_box[j][0] / 2
        x_max = center_list[i][0] + size_box[j][0] / 2
        y_min = center_list[i][1] - size_box[j][1] / 2
        y_max = center_list[i][1] + size_box[j][1] / 2
        black_box.append([[x_min, x_max], [y_min, y_max]])
    return black_box


def change_list(list_o, d):
    list_result = []
    k = 1.0 * d / 2.0
    count = 0
    for item in list_o:
        count += 1
        y = item[1][0] + k
        while (y + k) <= item[1][1]:
            x = item[0][0] + k
            if count == 1:
                print('obstacle.txt generated!')
            # list_result.append([round(xx, 4),round(yy, 4)])
            list_result.append([x, y])
            while (x + k) <= item[0][1]:
                x += d
                # list_result.append([round(x, 4),round(yy, 4)])
                list_result.append([x, y])
            y += d
    return list_result


def create_human_point(black_box):
    in_obstacle = True
    count=1
    while in_obstacle and count<500:
        a = random.uniform(-50, 140)
        b = random.uniform(-50, 50)
        for i in range(int(len(black_box))):
            if (a > (black_box[i][0][0] - 3)) and (a < (black_box[i][0][1] + 3)) or (b > (black_box[i][1][0] - 3)) and (
                    b < (black_box[i][1][1] + 3)):
                in_obstacle = True
                count=count+1
                break
            else:
                in_obstacle = False
    if count==100:
        a= random.choice((-50, 140))
        b= random.choice((-50, 50))
        print('AAA')
    return int(a), int(b)


def rand_rover(p):
    if p == 0:
        x = random.uniform(-48, 122)
        y = random.uniform(-48, -42)
    elif p == 1:
        x = random.uniform(-48, 122)
        y = random.uniform(-3, 3)
    elif p == 2:
        x = random.uniform(-48, 122)
        y = random.uniform(48, 42)
    elif p == 3:
        x = random.uniform(117, 123)
        y = random.uniform(-48, 48)
    elif p == 4:
        x = random.uniform(107, 113)
        y = random.uniform(-48, 48)
    elif p == 5:
        x = random.uniform(42, 48)
        y = random.uniform(-48, 48)
    elif p == 6:
        x = random.uniform(-11, -17)
        y = random.uniform(-48, 48)
    else:
        x = random.uniform(-41, -47)
        y = random.uniform(-48, 48)
    return [int(x), int(y)]


def create_rover_point(rover_num):
    in_obstacle = True
    rover_size = 2
    center_box = []
    for i in range(rover_num):
        in_obstacle = True
        while in_obstacle:
            p = math.floor(random.uniform(0, 8))
            [x, y] = rand_rover(p)
            in_obstacle = False
            for i in range(len(center_box)):
                if abs(x - center_box[i][0]) > rover_size + 2 or abs(y - center_box[i][1]) > rover_size + 2:
                    in_obstacle = False
                else:
                    in_obstacle = True
                    break
        center_box.append([x, y])
    return center_box

map_num=random.randint(0,2)
content = open("base"+str(map_num)+".world", 'r')
with open("house.world", 'w') as f:
    count = 5
    change_flag = False
    count_loop=0
    lines = content.readlines()
    center_list, find_solution = create_point()
    while (not find_solution) and count_loop<10:
        center_list,find_solution = create_point()
        count_loop=count_loop+1
        print(' Try again!')
    name_list = ['house_1_146', 'house_3_156', 'house_3_157', 'house_3_158', 'gas_station_73', 'fast_food_93',
                 'house_1_66', 'house_1_67', 'house_1_146_clone', 'house_2_71', 'house_2_125', 'house_2_126',
                 'house_3_68']
    if count_loop==10:
        lines = content.readlines()
        f.writelines(lines)
        print('Base world is used!')
    else:
        for num, line in enumerate(lines):
            for i in range(len(name_list)):
                if count == 1 or count == 4:
                    line = line_1
                if change_flag and "<pose frame=''>" in line and "<link name='link'>" not in lines[num - 1]:
                    pose_ori = line.split()
                    line = "<pose frame=''>" + str(center_list[i][0]) + ' ' + str(center_list[i][1]) + ' ' + str(
                        pose_ori[-4]) + ' ' + str(pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                    change_flag = False
                if "<model name='" + name_list[i] + "'>\n" in line:
                    if "<static>1</static>\n" in lines[num + 1]:
                        change_flag = True
                    else:
                        count = 0
                        pose_ori = lines[num + 1].split()
                        line_1 = "<pose frame=''>" + str(center_list[i][0]) + ' ' + str(center_list[i][1]) + ' ' + str(
                            pose_ori[-4]) + ' ' + str(pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(
                            pose_ori[-1]) + '\n'

            count = count + 1
            f.write(line)
f.close()
print('Finish!')
## random_rover
rover_center_list = create_rover_point(rover_num)
rover_content = open("rover_static.world", 'r')
with open("rover.world", 'w') as f:
    lines = rover_content.readlines()
    for i in range(rover_num):
        count = 0
        line_i = "<model name='rover_static_clone_clone_10_clone_clone_" + str(i + 1) + "'>\n"
        for num, line in enumerate(lines):
            if count == 1 and num + 1 < len(lines):
                if "</model>" in lines[num + 1]:
                    pose_ori = line.split()
                    line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                        rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(pose_ori[-3]) + ' ' + str(
                        pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                    if count == 2 and num > 0:
                        if "<model name='rover_static" in line:
                            line = line_i
                        elif "<model name='rover_static" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                                rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='base_link'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                                rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='front_left_wheel'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] - 1.2293) + ' ' + str(
                                rover_center_list[i][1] - 0.6585) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='front_left_wheel_steering_block'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] - 1.2189) + ' ' + str(
                                rover_center_list[i][1] - 0.4431) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='front_right_wheel'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] - 1.1614) + ' ' + str(
                                rover_center_list[i][1] + 0.7712) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='front_right_wheel_steering_block'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] - 1.1715) + ' ' + str(
                                rover_center_list[i][1] + 0.5558) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='rear_left_wheel'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] + 0.964) + ' ' + str(
                                rover_center_list[i][1] - 0.7626) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='rear_right_wheel'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0] + 1.0319) + ' ' + str(
                                rover_center_list[i][1] + 0.667) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        elif "<link name='rover/imu_link'>" in lines[num - 1]:
                            pose_ori = line.split()
                            line = " <pose frame=''>" + str(rover_center_list[i][0]) + ' ' + str(
                                rover_center_list[i][1]) + ' ' + str(pose_ori[-4]) + ' ' + str(
                                pose_ori[-3]) + ' ' + str(pose_ori[-2]) + ' ' + str(pose_ori[-1]) + '\n'
                        else:
                            line = line
            if "<model name='rover_static_clone_clone_10_clone_clone" in line:
                line = line_i
                count = count + 1
            f.write(line)
f.close()
for i in range(len(rover_center_list)):
    center_list.append(rover_center_list[i])
black_box = obstacle_list(center_list)
with open("black_box.txt", 'w') as f:
    line = str(black_box)
    f.write(line)
f.close()
## random_actor_pose
print('Obstacle List generated!')
content = open("house.world", 'r')
rover_content = open("rover.world", 'r')
with open(output_path + "robocup.world", 'w') as f:
    lines = content.readlines()
    lines_rover = rover_content.readlines()
    count = 2
    for num, line in enumerate(lines):
        if count == 1:
            line = line_1
        if 'filename="libros_actor_cmd_pose_plugin.so">' in line:
            count = 0
            a, b = create_human_point(black_box)
            line_1 = "        <init_pose>" + str(a) + ' ' + str(b) + " 1.25 1.57 0 0</init_pose>"
        count = count + 1
        f.write(line)
        if num + 1 < len(lines):
            if "</world>\n" in lines[num + 1]:
                f.writelines(lines_rover)
f.close()
print('human initial pose generated!')

## obstacle.txt
result_list = change_list(black_box, 1.0)
length = len(result_list)
ARRS = []
f = open('obstacle.txt', 'w+')
for i in range(length):
    jointsFrame = result_list[i]
    ARRS.append(jointsFrame)
    for Ji in range(2):
        strNum = str(jointsFrame[Ji])
        f.write(strNum)
        f.write(' ')
    f.write('\n')
f.close()
