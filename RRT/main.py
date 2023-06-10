import math
from vision import Vision
from action import Action
from debug import Debugger
import time


def calculate_distance(x1, y1, x2, y2):
    return abs((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))**0.5


if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    time.sleep(0.1)
    now_x = vision.my_robot.x
    now_y = vision.my_robot.y
    goal_x = -2400
    goal_y = -1500
    K_a = 2.5
    K_r = 100000000000
    d_a = 500
    p_0 = 500
    while True:
        #action.controlObs(vision)
        obstacle = []
        now_x = vision.my_robot.x
        now_y = vision.my_robot.y
        if calculate_distance(now_x, now_y, goal_x, goal_y) < 30:
            action.sendCommand(vx=50, vw=2)
            if goal_x == -2400:
                goal_x = 2400
                goal_y = 1500
            else:
                goal_x = -2400
                goal_y = -1500

        for i in range(0, len(vision.yellow_robot)):
            if vision.yellow_robot[i].x != -999999:
                obstacle.append(vision.yellow_robot[i])
        for i in range(1, len(vision.blue_robot)):
            if vision.yellow_robot[i].x != -999999:
                obstacle.append(vision.blue_robot[i])
        distance = calculate_distance(now_x, now_y, goal_x, goal_y)
        if distance <= d_a:
            F_ob_x = -2 * K_a * (now_x - goal_x)
            F_ob_y = -2 * K_a * (now_y - goal_y)
        else:
            F_ob_x = -2 * K_a * d_a * (now_x - goal_x) / distance
            F_ob_y = -2 * K_a * d_a * (now_y - goal_y) / distance
        F_x = F_ob_x
        F_y = F_ob_y
        for obstacle_object in obstacle:
            #print(obstacle_object.x,obstacle_object.y)
            p = calculate_distance(now_x, now_y, obstacle_object.x, obstacle_object.y)
            #print(p)
            if p < p_0:
                F_ad_x = K_r * (1 / p - 1 / p_0) * (1 / p ** 3) * (now_x - obstacle_object.x)
                F_ad_y = K_r * (1 / p - 1 / p_0) * (1 / p ** 3) * (now_y - obstacle_object.y)
            else:
                F_ad_x = 0
                F_ad_y = 0
            F_x = F_x + F_ad_x
            F_y = F_y + F_ad_y
        print(F_x, F_y)

        point_robot = vision.my_robot.orientation
        point_robot = 2 * math.pi + point_robot if point_robot < 0 else point_robot
        point_robot_save = point_robot  # 保留信息
        point_F = math.atan2(F_y, F_x)
        point_F = 2 * math.pi + point_F if point_F < 0 else point_F
        point_F_save = point_F
        error = abs(point_robot - point_F)
        error_save = point_robot_save - point_F_save  # 求车头方向和指向目标方向的夹角（非绝对值）
        error = 2 * math.pi - error if error > math.pi else error
        if error_save >= 0:  # 判断夹角大小，大于零逆时针；小于零，顺时针转
            action.sendCommand(vx=0, vw=-2)
        else:
            action.sendCommand(vx=0, vw=2)
        while error > (15 / 180) * math.pi:  # 给定一个停止旋转范围
            point_robot = vision.my_robot.orientation
            point_robot = 2 * math.pi + point_robot if point_robot < 0 else point_robot
            point_F = math.atan2(F_y, F_x)
            point_F = 2 * math.pi + point_F if point_F < 0 else point_F
            error = abs(point_robot - point_F)
            error = 2 * math.pi - error if error > math.pi else error

        action.sendCommand(vx=1000, vw=0)  # 非旋转时刻前进

        time.sleep(0.002)
        now_x = vision.my_robot.x
        now_y = vision.my_robot.y
        if calculate_distance(now_x, now_y, goal_x, goal_y) < 100:
            action.sendCommand(vx=50,vw=2)
            if goal_x == -2400:
                goal_x = 2400
                goal_y = 1500
            else:
                goal_x = -2400
                goal_y = -1500

