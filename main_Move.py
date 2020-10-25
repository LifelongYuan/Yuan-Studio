from APF_MOVE import APF
from action import Action
from vision import Vision
from debug import Debugger
import time
if __name__ == '__main__':
    x=[-2400,2400,-2400,2400,-2400,2400,-2400,2400,-2400,2400]
    y=[-1500,1500,-1500,1500,-1500,1500,-1500,1500,-1500,1500]
    vision = Vision()
    debugger = Debugger()
    action = Action()
    for i in range(10):
        APF_Plan = APF(x[i], y[i], Ka=17, Kb=2 * 10 ** 12, da=400, db=800, delta_t=10, distance=200, dy=150, ## TODO  db=725 Kb = 2*10**12  Ka = 15
                       Kt=10 * 10 ** 11,
                       distance_2=300, distance_3=100, cv=420,debugger=debugger,vision=vision,action=action) ## TODO distance3 = 400
        max_num = 10000
        for i in range(max_num):
            APF_Plan.Control(1)  # 得到风向标位置
            if APF_Plan.checkDestination():
                # APF_Plan.final_list.append([forx, fory])
                print("此次避障成功")
                APF_Plan.Draw()
                break
        else:
            APF_Plan.Draw()
            print("超过最大迭代次数！")
        time.sleep(1)
        del APF_Plan

