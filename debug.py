import socket
import sys
import time

from zss_debug_pb2 import Debug_Msgs, Debug_Msg


class Debugger(object):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.debug_address = ('localhost', 20001)

    def draw_circle(self, x, y):
        package = Debug_Msgs()
        msg = package.msgs.add()
        msg.type = Debug_Msg.ARC
        msg.color = Debug_Msg.WHITE
        arc = msg.arc
        radius = 300
        arc.rectangle.point1.x = x - radius
        arc.rectangle.point1.y = y - radius
        arc.rectangle.point2.x = x + radius
        arc.rectangle.point2.y = y + radius
        arc.start = 0
        arc.end = 360
        arc.FILL = True
        self.sock.sendto(package.SerializeToString(), self.debug_address)

    def draw_robot(self, x, y, dir):
        package = Debug_Msgs()
        msg = package.msgs.add()
        msg.type = Debug_Msg.ROBOT
        msg.color = Debug_Msg.WHITE
        robo = msg.robot
        robo.pos.x = x
        robo.pos.y = y
        robo.dir = dir
        self.sock.sendto(package.SerializeToString(), self.debug_address)

    def draw_line(self, x1, y1, x2, y2, Ward):
        package = Debug_Msgs()
        msg = package.msgs.add()
        msg.type = Debug_Msg.LINE
        msg.color = Debug_Msg.WHITE
        line = msg.line
        line.start.x = x1
        line.start.y = y1
        line.end.x = x2
        line.end.y = y2
        if Ward == "FORWARD":
            line.FORWARD = True
            line.BACK = False
        else:
            line.FORWARD = False
            line.BACK = True
        self.sock.sendto(package.SerializeToString(), self.debug_address)


    def draw_final_line(self,final_list,T,startpoint):
        package = Debug_Msgs()
        for i in range(len(final_list)-1):
            msg = package.msgs.add()
            msg.type = Debug_Msg.LINE
            msg.color = Debug_Msg.GREEN
            line = msg.line
            line.start.x = final_list[i][0]
            line.start.y = final_list[i][1]
            line.end.x = final_list[i+1][0]
            line.end.y = final_list[i+1][1]
            line.FORWARD = True
            line.BACK = False
        #self.Draw_PreorderTraverse(T,startpoint)
        self.sock.sendto(package.SerializeToString(), self.debug_address)
    def Draw_PreorderTraverse(self, T,startpoint):
        if T != startpoint:
            self.draw_line((T.parent)[0].x,(T.parent)[0].y,T.x,T.y,"FORWARD")
        for item in range(len(T.child)):
            self.PreorderTraverse(T.child[item])
    def draw_point(self, x, y):
        package = Debug_Msgs()
        msg = package.msgs.add()
        msg.type = Debug_Msg.POINT
        msg.color = Debug_Msg.WHITE
        point = msg.point
        point.x = x
        point.y = y
        self.sock.sendto(package.SerializeToString(), self.debug_address)


if __name__ == '__main__':
    debugger = Debugger()
    while True:
        debugger.draw_circle(x=100, y=200)
        debugger.draw_robot(x=100, y=200, dir=30)
        debugger.draw_line(0, 0, 200, 400, "FORWARD")
        time.sleep(0.02)
