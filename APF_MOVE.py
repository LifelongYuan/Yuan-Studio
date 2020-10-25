from math import sqrt, pi
import numpy as np
from vision import Vision
from action import Action
from debug import Debugger
import time
from math import cos, sin, atan
from control import linearcotrol, phase_atan


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
        self.vx = self.vision.blue_robot[0].vel_x
        self.vy = self.vision.blue_robot[0].vel_y
        self.real_list.append([self.px, self.py])
        if self.vision.blue_robot[0].orientation < 0:
            self.orientation = self.vision.blue_robot[0].orientation + 2 * pi
        else:
            self.orientation = self.vision.blue_robot[0].orientation

    def vdistance(self, id):
        if id == -1:
            return [self.px - self.ox, self.py - self.oy]
        return [self.px - self.vision.yellow_robot[id].x, self.py - self.vision.yellow_robot[id].y]

    def Euclidean_distance(self, vec):
        return sqrt(vec[0] ** 2 + vec[1] ** 2)

    def __init__(self, forx, fory, Ka, da, Kb, db, delta_t, distance, distance_2, dy, Kt, distance_3,cv,vision,action,debugger):

        self.vision = vision
        self.action = action
        self.debugger = debugger
        time.sleep(0.5)
        # print(self.vision.yellow_robot[0].x)
        self.px = self.vision.blue_robot[0].x
        self.py = self.vision.blue_robot[0].y
        self.real_list = [[self.px, self.py]]  # 小车实际位置序列
        self.Refresh_robo()
        self.ox = forx
        self.oy = fory
        self.Ka = Ka
        self.da = da
        self.Kb = Kb
        self.db = db
        self.cv = cv
        self.delta_t = delta_t
        self.distance = distance
        self.final_list = [[self.px, self.py]]
        self.signal_list = [[self.px, self.py, self.orientation]]  # 风向标序列

        self.ax = 1
        self.ay = 1
        self.control = linearcotrol(Kp=10, Ka=1.1, Kb=0.7)
        self.dy = dy
        self.Kt = Kt
        self.distance_2 = distance_2

        self.distance_3 = distance_3

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
            if id in [3, 4, 6, 9, 12]:
                k = self.Doppler(id)

                if Edistance <= self.db + self.dy:
                    f = list(
                        k*(self.Kb + self.Kt) * (1 / Edistance - 1 / (self.db + self.dy)) * ((1 / Edistance) ** 3) * s)
                    return f
                if Edistance > self.db + self.dy:
                    return [0, 0]
            else:
                if Edistance <= self.db:
                    f = list(self.Kb * (1 / Edistance - 1 / self.db) * ((1 / Edistance) ** 3) * s)
                    return f
                if Edistance > self.db:
                    return [0, 0]

    def Total_Force(self):  # 计算当前x ,y 方向上的合力
        [self.ax, self.ay] = [0, 0]
        # print(self.Force(0, 0))
        [self.ax, self.ay] = [self.ax + self.Force(0, 0)[0], self.ay + self.Force(0, 0)[1]]
        for i in range(16):
            [self.ax, self.ay] = [self.ax + self.Force(1, i)[0], self.ay + self.Force(1, i)[1]]

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
        self.theta = phase_atan(self.ax, self.ay)
        self.px += self.delta_t * (self.ax / sqrt(self.ax ** 2 + self.ay ** 2))
        self.py += self.delta_t * (self.ay / sqrt(self.ax ** 2 + self.ay ** 2))
        self.final_list.append([self.px, self.py])
        self.debugger.draw_line(self.final_list[-1][0], self.final_list[-1][1], self.final_list[-2][0],
                                self.final_list[-2][1], "FORWARD")
        time.sleep(0.005)
        # Action()  发送具体的运动指令

    def Refresh_control(self, n_step):
        for i in range(n_step):
            self.Refresh_Position()
        self.signal_list.append(self.final_list[-1])
        (self.signal_list[-1]).append(self.theta)
        # print(self.signal_list)
        # print(self.final_list)

    def Control(self, n_step):
        self.Refresh_control(10)

        for i in range(n_step):
            self.Refresh_robo()
            self.control.getControl([self.px, self.py, self.orientation], self.signal_list[-1])
            self.action.sendCommand(vx=self.control.v, vw=self.control.w)

    def checkDestination(self):
        s1 = np.array([self.ox, self.oy])
        self.Refresh_robo()
        s2 = np.array([self.px, self.py])

        dis = self.Euclidean_distance(list(s2 - s1))
        if dis <= self.distance_2:
            v_slow = 0
            while not self.checkEnd() and v_slow==0:
                self.Refresh_robo()
                print("开始减速！开始减速！！")

                #self.control.getControl([self.px, self.py, self.orientation], [self.ox, self.oy, pi])
                if self.control.v > 60:
                    self.action.sendCommand(vx=self.control.v-60, vw=self.control.w)
                    self.control.v -= 60
                else:
                    self.action.sendCommand(vx=self.control.v, vw=self.control.w)
                time.sleep(0.05)
                print("v_real = {}".format(self.control.v))
                if self.control.v <= 60:
                    self.control.Kb = 0
                    self.control.Ka = 1
                    v_slow = 1
                    self.control.vmaxlimit = 60
            while not self.checkEnd():
                print("最终导引开始")
                self.Refresh_robo()
                self.control.getControl([self.px, self.py, self.orientation], [self.ox, self.oy, pi])
                self.action.sendCommand(vx=self.control.v, vw=self.control.w)
                self.debugger.draw_line(self.px,self.py,self.ox, self.oy, "FORWARD")
                time.sleep(0.1)
            self.action.sendCommand()


            return True

    def checkEnd(self):
        s1 = np.array([self.ox, self.oy])
        self.Refresh_robo()
        s2 = np.array([self.px, self.py])

        dis = self.Euclidean_distance(list(s2 - s1))
        if dis <= self.distance_3:
            return True
        else:
            return False

    def Draw(self):
        self.debugger.draw_final_line(self.real_list, 0, 0)

    def Doppler(self, id):
        vo = np.array([self.vx, self.vy])
        vs = np.array([self.vision.yellow_robot[id].vel_x, self.vision.yellow_robot[id].vel_x])
        delta = self.vdistance(id)
        delta_d = self.Euclidean_distance(delta)
        delta_a = (-1) * np.array(delta)
        o = np.dot(delta_a, vo) / delta
        s = np.dot(delta_a, vs) / delta
        Dop = (self.cv + o) / (self.cv + s)
        return Dop

    def __del__(self):
        del self.vision
        del self.debugger
        del self.action
