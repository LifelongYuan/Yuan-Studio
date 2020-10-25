import socket
import sys
import time
from vision import Vision
from zss_cmd_pb2 import Robots_Command, Robot_Command
import math

class Action(object):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.command_address = ('localhost', 50001)

    def sendCommand(self, vx=0, vy=0, vw=0):
        commands = Robots_Command()
        command = commands.command.add()
        command.robot_id = 0
        command.velocity_x = vx
        command.velocity_y = vy
        command.velocity_r = vw
        command.kick = False
        command.power = 0
        command.dribbler_spin = False
        self.sock.sendto(commands.SerializeToString(), self.command_address)


if __name__ == '__main__':
    action_module = Action()
    while True:
        action_module.sendCommand(vx=0, vy=500, vw=1)
        print(action_module.vision.blue_robot[0].orientation)
        # if math.pi/2-0.1<action_module.vision.blue_robot[0].orientation<=math.pi/2:
        #     action_module.sendCommand(vx=0, vy=500, vw=0)
        #     break
        time.sleep(0.02)