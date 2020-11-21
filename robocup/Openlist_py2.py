list_o = [[[-35.375, -34.825], [-4, -3]], [[-3.275, -2.725], [-4, -3]], [[20.725, 21.275], [-4, -3]], \
        [[57.725, 58.275], [-4, -3]], [[83.725, 84.275], [-4, -3]], [[-25.475, -24.925], [3, 4]], [[8.725, 9.275], [3, 4]], \
        [[26.725, 27.275], [3, 4]], [[70.725, 71.275], [3, 4]], [[90.225, 90.775], [3, 4]],[[52.2, 55.2],[-10.0, -5.72]], \
        [[78, 91], [-34.6, -27]],[[2, 10], [33, 37]], [[-36.0, -19.0], [16.0, 34.5]], [[3.0, 23.0], [10.0, 26.0]], [[54, 70.0], [14, 31.3]], \
        [[70.7, 84.5], [8.5, 19.5]], [[87.3, 101.7], [10.9, 17.2]], [[78, 95.7], [21.2, 35.5]], [[52, 71], [-36, -26]], [[-6.4, 6.5], [-20.4, -9.6]], \
        [[13.5, 40.5], [-23.5, -4.5]], [[-6.0, 6.0], [-35.0, -21.0]], [[-28.8, -22.5], [-28.5, -14]], [[-37, -23], [-36, -30]], \
        [[-36.5, -30.3], [-26.5, -12]], [[2.0,9.2],[33.3,36.7]]]

def change_list(list_o, d):
    list_result = []
    k = 1.0*d/2.0
    count = 0
    for item in list_o:
        count += 1
        y = item[1][0] + k
        while (y+k) <= item[1][1]:
            x = item[0][0]+k
            if count == 1:
                print y
            #list_result.append([round(xx, 4),round(yy, 4)])
            list_result.append([x, y])
            while (x+k) <= item[0][1]:
                x += d
                #list_result.append([round(x, 4),round(yy, 4)])
                list_result.append([x, y])
            y += d
    return list_result

# print change_list([[[15, 18], [-16, -12]]], 2)
result_list = change_list(list_o, 1.0)
length = len(result_list)

ARRS = []

f=open('obstacle.txt','w+')

for i in range(length):

    jointsFrame = result_list[i]

    ARRS.append(jointsFrame)

    for Ji in range(2):

        strNum = str(jointsFrame[Ji])

        f.write(strNum)

        f.write(' ')

    f.write('\n')

f.close()

