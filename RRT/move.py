import time

from action import Action
import math
from vision import Vision

def CalDistance(node1_x, node1_y, node2_x, node2_y):
    return (node1_x - node2_x)*(node1_x - node2_x) + (node1_y - node2_y)*(node1_y - node2_y)

def move(vision, x1, y1):
    action = Action()
    while True:
        now_x, now_y = vision.my_robot.x, vision.my_robot.y
        D_x = x1 - now_x
        D_y = y1 - now_y
        theta = vision.my_robot.orientation
        alpha = math.atan2(D_y, D_x)
        alpha_1 = alpha
        theta_1 = theta
        theta = 2 * math.pi + theta if theta < 0 else theta
        alpha = 2 * math.pi + alpha if alpha < 0 else alpha
        error = abs(theta - alpha)
        error = 2 * math.pi + error if error < 0 else error
        error_1 = theta_1 - alpha_1
        print(error_1)

        if error_1 >= math.pi*10/180 and error_1 <= math.pi:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=0, vw=-1)
        # else:
        #     action.sendCommand(vx=500, vw=1)


        if error_1 >= 0 and error_1 <= math.pi*10/180:
            action.sendCommand(vx=0, vw=-1)
        if error_1 >= math.pi and error_1 <= math.pi*350/180:
            action.sendCommand(vx=0, vw=-3)
        if error_1 >= math.pi*350/180 and error_1 <= math.pi*2:
            action.sendCommand(vx=0, vw=1)
        if error_1 >= -math.pi and error_1 <= -math.pi * 10 / 180:
            action.sendCommand(vx=0, vw=3)

        if error_1 >= -math.pi * 10 / 180 and error_1 <= 0:
            action.sendCommand(vx=0, vw=1)
        if error_1 >= -math.pi*350/180 and error_1 <= -math.pi:
            action.sendCommand(vx=0, vw=-3)
        if error_1 >= -math.pi*2 and error_1 <= -math.pi*350/180:
            action.sendCommand(vx=0, vw=-1)

        # if 0 <= error_1 <= math.pi * 15 / 180:
        #     action.sendCommand(vx=10, vw=1)
        # if math.pi * 15 / 180 <= error_1 <= math.pi/2:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
        #     action.sendCommand(vx=0, vw=3)
        # if -math.pi * 15 / 180 <= error_1 < 0:
        #     action.sendCommand(vx=10, vw=-1)
        # if -math.pi/2 <= error_1 < -math.pi * 15 / 180:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
        #     action.sendCommand(vx=0, vw=-3)
        # if math.pi * 345 / 180 <= error_1 <= math.pi * 2:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
        #     action.sendCommand(vx=10, vw=-1)
        # if -math.pi <= error_1 <= -math.pi * 15 / 180:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
        #     action.sendCommand(vx=0, vw=-3)
        # if -math.pi * 15 / 180 <= error_1 <= 0:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
        #     action.sendCommand(vx=10, vw=-1)
        # if -math.pi * 345 / 180 <= error_1 <= -math.pi:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
        #     action.sendCommand(vx=0, vw=3)
        # if -math.pi * 2 <= error_1 <= -math.pi * 345 / 180:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
        #     action.sendCommand(vx=10, vw=x1)
        while error > (3 / 180) * math.pi:  # 给定一个停止旋转范围
            theta = vision.my_robot.orientation
            theta = 2 * math.pi + theta if theta < 0 else theta
            alpha = math.atan2(D_y, D_x)
            alpha = 2 * math.pi + alpha if alpha < 0 else alpha
            error = abs(theta - alpha)
            error = 2 * math.pi - error if error > math.pi else error
        v = CalDistance(now_x, now_y, x1, y1)**0.5 / 0.2
        action.sendCommand(vx=v, vw=0)  # 非旋转时刻前进
        time.sleep(0.2)
        if abs(now_x - x1) <= 30 and abs(now_y - y1) <= 30:
            action.sendCommand(vx=0, vw=0)
            break
    print('arrive')  # 阶段测试
'''
def move_to_position(vision, path_x, path_y):
    action = Action()
    while True:
        now_x, now_y = vision.my_robot.x, vision.my_robot.y
        D_x = path_x - now_x
        D_y = path_y - now_y
        theta = vision.my_robot.orientation
        alpha = math.atan2(D_y, D_x)
        alpha_1 = alpha
        theta_1 = theta
        theta = 2 * math.pi + theta if theta < 0 else theta
        alpha = 2 * math.pi + alpha if alpha < 0 else alpha
        error = abs(theta - alpha)
        error = 2 * math.pi + error if error < 0 else error

'''