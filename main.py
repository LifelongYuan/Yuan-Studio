from APF import APF

if __name__ == '__main__':
    # vision = Vision()
    # action = Action()
    # debugger = Debugger()
    # x1 = vision.my_robot.x
    # y1 = vision.my_robot.y
    forx = 2000
    fory = 2000
    APF_Plan = APF(forx, fory, Ka=10, Kb=2*10**12, da=400, db=400, delta_t=10, distance=100)
    max_num = 10000
    for i in range(max_num):
        APF_Plan.Refresh_Position()
        if APF_Plan.checkDestination():
            #APF_Plan.final_list.append([forx, fory])
            print("成功")
            APF_Plan.Draw()
            break
    else:
        APF_Plan.Draw()
        print("结束")