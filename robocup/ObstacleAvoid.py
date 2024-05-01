import numpy
import math
from geometry_msgs.msg import Point
import sys

class ObstacleAvoid:
    def __init__(self):
        self.indexOfObstInSp = []
        self.obstInUAVCoorSys = []
        self.obstlist = numpy.loadtxt('2024.txt')
        self.flag = True
        self.useOriginalCurPos = False
        self.subTarg = Point()
        self.side = 0
        self.indexOfFirstObst = 0
        self.minDisInSpline = 9000
        self.obstList = self.obstList2point()

    def obstList2point(self):
        length = len(self.obstlist)
        list = [Point() for i in range(length)]
        for i in range(length):
            list[i].x = self.obstlist[i][0]
            list[i].y = self.obstlist[i][1]
            list[i].z = 0.0
        return list

    def GetPointList(self, curPos, targetPos, safeDis):
        subTargList = []
        self.obstInUAVCoorSys = [Point() for i in range(len(self.obstList))]
        tempStartPt = curPos
        tempTargePos = targetPos
        self.flag = True
        self.useOriginalCurPos = False
        while self.flag:
            self.subTarg = Point()
            if self.GetSubTarget(tempStartPt, tempTargePos, safeDis):
                tempStartPt = Point()
                tempStartPt.x = self.subTarg.x
                tempStartPt.y = self.subTarg.y
                subTargList.append(tempStartPt)
            else:
                self.flag = False
        subTargList.append(tempTargePos)
        
        return subTargList

    def GetSubTarget(self, startPos, targetPos, safeDis):
        curposToTarget = self.VectDiff(targetPos, startPos)  # vector differential, point from the end of the second to that of the first.
        if curposToTarget.x == 0.0 and curposToTarget.y == 0.0:
            return False
        lengthCurToTarget = math.sqrt(pow(curposToTarget.x, 2) + pow(curposToTarget.y, 2))  # distance from the start point to the target point.
        ones = Point()
        ones.x = 0
        ones.y = 0
        ones.z = 1
        #convert coordinates of obstacles from the the global coordinate system to that of the uav
        for k in range(len(self.obstList)):  #698
            temp1 = self.VectDiff(self.obstList[k], startPos)
            self.obstInUAVCoorSys[k].x = self.VectDot(curposToTarget, temp1) / lengthCurToTarget   # touying vector shuxiangji
            self.obstInUAVCoorSys[k].y = self.VectCross(curposToTarget, temp1) / lengthCurToTarget # touying vector xiangliangji

        #is true when calculate subtarget from current poistion of uav
        if not self.useOriginalCurPos:
            self.indexOfObstInSp = []
            for j in range(len(self.obstInUAVCoorSys)):
                if ((self.obstInUAVCoorSys[j].x > 0) and (self.obstInUAVCoorSys[j].x < lengthCurToTarget) and (abs(self.obstInUAVCoorSys[j].y) + abs(self.obstInUAVCoorSys[j].x) < safeDis)):
                    self.indexOfObstInSp.append(j)
                    #print 'self.obstList'+str(j)+':', self.obstList[j]
                    #print 'self.obstInUAVCoorSys'+str(j)+':', self.obstInUAVCoorSys[j]
        if len(self.indexOfObstInSp) > 0:
            self.indexOfFirstObst = self.indexOfObstInSp[0]
            minDisInSpline = self.obstInUAVCoorSys[self.indexOfFirstObst].x
            for i in range(len(self.indexOfObstInSp)):
                if minDisInSpline > self.obstInUAVCoorSys[self.indexOfObstInSp[i]].x:
                    self.indexOfFirstObst = self.indexOfObstInSp[i]
                    #print 'self.indexOfFirstObst:', self.indexOfFirstObst
                    minDisInSpline = self.obstInUAVCoorSys[self.indexOfObstInSp[i]].x
                    #print 'self.obstList[self.indexOfObstInSp[i]]:', self.obstList[self.indexOfObstInSp[i]]
        # the assemblage of obstacles that need to be avoid when avoid the first obstacle
        # the content are the signals of obstacles in vector 'obstInUAVCoorSys'
            G = []
            numOfG = 0
            G.append(self.indexOfFirstObst)
            numOfG = numOfG + 1
            for i in range(120):            ###############cirletimes
                if i < numOfG:
                    for j in range(len(self.obstList)):
                        if j == G[i]:
                            continue
                        elif self.VectNorm(self.obstList[j], self.obstList[G[i]]) <= 2*safeDis:
                            if j not in G:
                                G.append(j)
                                numOfG = numOfG + 1
            maxPosEd = 0
            maxNegEd = 0
            #print "g0:", self.obstList[G[0]]
            for i in range(numOfG):
                if self.obstInUAVCoorSys[G[i]].y > 0:
                #posEd[posNum++] = obstInUAVCoorSys[G[i]].y
                    if self.obstInUAVCoorSys[G[i]].y > maxPosEd:
                        maxPosEd = self.obstInUAVCoorSys[G[i]].y
                elif self.obstInUAVCoorSys[G[i]].y < 0:
                #negEd[negNum++] = fabs(obstInUAVCoorSys[G[i]].y)
                    if abs(self.obstInUAVCoorSys[G[i]].y) > maxNegEd:
                        maxNegEd = abs(self.obstInUAVCoorSys[G[i]].y)
            if maxPosEd < maxNegEd:
            #leftB = 1
                sign_side = 1
            # the maximum ed in the left side is smaller than that in the right side
            else:
            #rigthB = 1
                sign_side = -1
            alpha = [0 for i in range(numOfG)]
        #numOfAlpha = 0
            maxAlpha = 0
            minAlpha = 10000
        #the two index marks the signal of obstacle that has the maximun or minimum angle in G
            maxAlphaIndex = 0
            minAlphaIndex = 0
            for i in range(numOfG):
                alpha[i] = math.atan2(self.obstInUAVCoorSys[G[i]].y, self.obstInUAVCoorSys[G[i]].x) + sign_side * (math.asin((safeDis+0.1)/self.VectNorm(self.obstList[G[i]], startPos))+0.1)  #0.1 for overshoot
                if alpha[i] > maxAlpha:
                    maxAlpha = alpha[i]
                    maxAlphaIndex = int(i)
                if alpha[i] < minAlpha:
                    minAlpha = alpha[i]
                    minAlphaIndex = int(i)

            if sign_side == 1:
                angle = maxAlpha
                obstIndex = maxAlphaIndex
            elif sign_side == -1:
                angle = minAlpha
                obstIndex = minAlphaIndex
            resu = self.VectNorm(self.obstList[G[obstIndex]], startPos) / lengthCurToTarget
            self.subTarg.y = startPos.y + (curposToTarget.y * math.cos(angle) + math.sin(angle) * curposToTarget.x) * resu
            self.subTarg.x = startPos.x + (curposToTarget.x * math.cos(angle) - math.sin(angle) * curposToTarget.y) * resu
            self.useOriginalCurPos = False
            #print 'self.subTarg:', self.subTarg
            return True
        else:
            return False

    def VectDiff(self, endPt, startPt):
        temp = Point()
        temp.x = endPt.x - startPt.x
        temp.y = endPt.y - startPt.y
        return temp

    def VectDot(self, Pt1, Pt2):
        dot = Pt1.x * Pt2.x + Pt1.y * Pt2.y
        return dot

    def VectDot3D(self, Pt1, Pt2):
        dot = Pt1.x * Pt2.x + Pt1.y * Pt2.y + Pt1.z * Pt2.z
        return dot

    def VectCross(self, vect1, vect2):
        temp = vect1.x * vect2.y - vect1.y * vect2.x
        return temp

    def VectSign(self, Pt):
        if abs(Pt.x) > 1e-5:
            Pt.x = Pt.x / abs(Pt.x)
        if abs(Pt.y) > 1e-5:
            Pt.y = Pt.y / abs(Pt.y)
        if abs(Pt.z) > 1e-5:
            Pt.z = Pt.z / abs(Pt.z)
        return Pt

    def VectNorm(self, Pt1, Pt2):
        norm = math.sqrt(pow(Pt1.x - Pt2.x, 2) + pow(Pt1.y - Pt2.y, 2))
        return norm

if __name__ == "__main__":
    curr = Point()
    curr.x = float(sys.argv[1])
    curr.y = float(sys.argv[2])
    print('curr',curr.x,curr.y)
    targ = Point()
    targ.x = float(sys.argv[3])
    targ.y = float(sys.argv[4])
    print('targ',targ.x,targ.y)

    avoid = ObstacleAvoid()
    tar = avoid.GetPointList(curr, targ, 1)


    










        
