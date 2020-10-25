from math import sqrt, atan
from math import pi

class linearcotrol():
    # Kp, Ka, Kb为线性控制律参数
    # 变量顺序为x,y,theta
    def __init__(self, Kp, Ka, Kb):
        self.Kp = Kp
        self.Ka = Ka
        self.Kb = Kb
        self.v0 = 0
        self.w0 = 0
        self.vlimit = 60
        self.wlimit = pi/6
        self.vmaxlimit = 600
    def refresh(self, startpoint, endpoint):
        self.setpoint = startpoint
        self.obpoint = endpoint

    def deltachange(self):
        self.delta_x = self.obpoint[0] - self.setpoint[0]
        self.delta_y = self.obpoint[1] - self.setpoint[1]
        self.delta_theta = self.obpoint[2] - self.setpoint[2]

    def polechange(self):
        self.r = sqrt(self.delta_x ** 2 + self.delta_y ** 2)
        self.beta = phase_atan(self.delta_x,self.delta_y) - self.obpoint[2]
        if self.beta > pi:
            self.beta = -2*pi + self.beta
        if self.beta < -pi:
            self.beta = self.beta + 2*pi
       # print(self.beta )
        self.alpha = -self.setpoint[2] + phase_atan(self.delta_x,self.delta_y)
        if self.alpha < (-pi):
            self.alpha = self.alpha + 2*pi
        if self.alpha > pi:
            self.alpha = self.alpha - 2*pi


    def getControl(self, startpoint, endpoint):
        self.refresh(startpoint, endpoint)
        self.deltachange()
        self.polechange()
        if -2*pi/3 <= self.alpha <= 2*pi/3:
            self.v = self.Kp * self.r
            self.w = self.Ka * self.alpha + self.Kb * self.beta

        # elif -2*pi/3 > self.alpha:
        #     self.alpha = -2*pi/3
        #     self.v = (self.Kp * self.r)
        #     self.w = (self.Ka * self.alpha + self.Kb * self.beta)
        # elif 2 * pi / 3 < self.alpha:
        #     self.alpha = 2 * pi / 3
        #     self.v = (self.Kp * self.r)
        #     self.w = (self.Ka * self.alpha + self.Kb * self.beta)
        else:
            self.v = -self.Kp * self.r
            self.w = (self.Ka * self.alpha + self.Kb * self.beta)
        if abs(self.v - self.v0) >= self.vlimit:
            if self.v - self.v0 <= 0:
                self.v = self.v0 - self.vlimit
            else:
                self.v = self.v0 + self.vlimit
        if abs(self.v) >= self.vmaxlimit: # TODO
            if self.v > self.vmaxlimit:
                self.v = self.vmaxlimit
            if self.v < -self.vmaxlimit:
                self.v = -self.vmaxlimit
        self.v0 = self.v
        if abs(self.w - self.w0) >= self.wlimit:
            if self.w - self.w0 <= 0:
                self.w = self.w0 - self.wlimit
            else:
                self.w = self.w0 + self.wlimit
        self.w0 = self.w
        print("v_control = {}  ".format(self.v))
        print("vw_control = {}\n".format(self.w))

def phase_atan(x, y):
    if x > 0 and y > 0:
        return atan(y / x)
    elif x == 0 and y > 0:
        return pi / 2
    elif x < 0 and y > 0:
        return pi + atan(y / x)
    elif x < 0 and y == 0:
        return pi
    elif x < 0 and y < 0:
        return pi + atan(y / x)
    elif x == 0 and y < 0:
        return pi * 3 / 2
    elif x > 0 and y < 0:
        return 2 * pi + atan(y / x)
    elif x > 0 and y == 0:
        return 0