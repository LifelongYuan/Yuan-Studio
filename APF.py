from math import sqrt
import numpy as np
from vision import Vision
from action import Action
from debug import Debugger
import time
from math import cos,sin,atan
class APF():
    # setx,y:start point's x,y
    # forx,y:end point's x,y
    # Ka:attraction's contant
    # da:attraction's Threshold
    # Kb:rejection's contant
    # db:rejection's Threshold

    # This Function is used to get new information from map
    def Refresh_robo(self):  # 重新读取小车实时位置
        self.px = self.vision.blue_robot[0].x
        self.py = self.vision.blue_robot[0].y
        self.vx = 0
        self.vy = 0
        self.orientation = self.vision.blue_robot[0].orientation

    def vdistance(self, id):
        if id == -1:
            return [self.px - self.ox, self.py - self.oy]
        return [self.px - self.vision.yellow_robot[id].x, self.py - self.vision.yellow_robot[id].y]

    def Euclidean_distance(self, vec):
        return sqrt(vec[0] ** 2 + vec[1] ** 2)

    def __init__(self, forx, fory, Ka, da, Kb, db, delta_t, distance):

        self.vision = Vision()


        self.debugger = Debugger()
        time.sleep(0.5)
        #print(self.vision.yellow_robot[0].x)
        self.Refresh_robo()
        self.ox = forx
        self.oy = fory
        self.Ka = Ka
        self.da = da
        self.Kb = Kb
        self.db = db
        self.delta_t = delta_t
        self.distance = distance
        self.final_list = [[self.px,self.py]]
        self.ax = 1
        self.ay = 1

    # TYPE = 0 is attraction while TYPE = 1 is rejection
    def Force(self, TYPE, id):
        if TYPE == 0:
            distance = self.vdistance(-1)
            s = np.array(distance)
            Edistance = self.Euclidean_distance(distance)
            if Edistance <= self.da:
                return list((-2) * self.Ka * s)
            if Edistance > self.da:
                return list((-2) * self.Ka * self.da / Edistance * s)
        elif TYPE == 1:
            distance = self.vdistance(id)
            s = np.array(distance)
            Edistance = self.Euclidean_distance(distance)
            if Edistance <= self.db:
                f = list(self.Kb * (1 / Edistance - 1 / self.db) * ((1 / Edistance) ** 3) * s)
                return f
            if Edistance > self.db:
                return [0, 0]

    def Total_Force(self):  # 计算当前x ,y 方向上的合力
        [self.ax, self.ay] = [0,0]
        #print(self.Force(0, 0))
        [self.ax, self.ay] = [self.ax+ self.Force(0, 0)[0], self.ay+self.Force(0, 0)[1]]
        for i in range(16):
             print(self.Force(1, i))
             [self.ax, self.ay] = [self.ax + self.Force(1, i)[0], self.ay+self.Force(1, i)[1]]
        print("adddddd")
        print(self.ax,self.ay)
    def Refresh_Position(self):
        # self.Refresh_robo()  路径规划时不执行
        self.Total_Force()
        # print("AA")
        # print(self.ax, self.ay)
        # self.vx += self.delta_t * self.ax
        # self.vy += self.delta_t * self.ay
        # self.px += self.vx * self.delta_t + 0.5 * self.ax * self.delta_t ** 2
        # self.py += self.vy * self.delta_t + 0.5 * self.ay * self.delta_t ** 2
        # print(self.px,self.py)
        # self.final_list.append([self.px, self.py])
        # self.debugger.draw_line(self.final_list[-1][0],self.final_list[-1][1],self.final_list[-2][0],self.final_list[-2][1],"FORWARD")
        # time.sleep(1)
        theta = atan(self.ay/self.ax)
        print("theta")
        print(theta)
        self.px+= self.delta_t*(self.ax/sqrt(self.ax**2+self.ay**2))
        self.py+= self.delta_t*(self.ay/sqrt(self.ax**2+self.ay**2))
        self.final_list.append([self.px, self.py])
        self.debugger.draw_line(self.final_list[-1][0],self.final_list[-1][1],self.final_list[-2][0],self.final_list[-2][1],"FORWARD")
        time.sleep(0.01)
        # Action()  发送具体的运动指令

    def checkDestination(self):
        s1 = np.array([self.ox, self.oy])
        s2 = np.array(self.final_list[-1])
        print("dis")

        dis = self.Euclidean_distance(list(s2-s1))
        print(dis)
        if dis <= self.distance:
            return True
        else:
            return False

    def Draw(self):
        self.debugger.draw_final_line(self.final_list, 0, 0)
