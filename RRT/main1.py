from vision import Vision
from action import Action
import time
import debug
import RRT
import numpy as np
import feedback

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = debug.Debugger()
    time.sleep(0.05)
    # while True:
    # action.controlObs(vision)
    # action.sendCommand(vx=500, vy=0, vw=0)
    # my_robot = vision.my_robot
    # print("my robot ", my_robot.x, my_robot.y)
    # print(vision.my_robot.x)
    goal_x = 2400
    goal_y = 1500
    #path = RRT.Create_RRT_accuate(vision, goal_x, goal_y)

    #feedback.move_to_pose(vision=vision, path_x=np.array(path)[:, 0], path_y=np.array(path)[0, :])
    #path_x, path_y = RRT.Smooth_path(vision, np.array(path)[:, 0], np.array(path)[0, :])
    # print(1)
    # path = RRT.Find_Path(mR)
    # for n1, n2 in zip(path[0:len(path)-1], path[1:len(path)]):
    #     debugger.draw_circle(package, x=n1.x, y=n1.y, radius=50)
    #     debugger.draw_circle(package, x=n2.x, y=n2.y, radius=50)
    #     debugger.draw_line(package, x1=n1.x, y1=n1.y, x2=n2.x, y2=n2.y)
    # # 画多条线
    # debugger.draw_lines(package, x1=[0,0], y1=[0,2000], x2=[2000,2000], y2=[0,2000])
    # # 画一个点
    # path = [[1000, 1000], [500, 1500]]

    while True:
        # package = debug.Debug_Msgs()
        # while True:
        # for p in path:
        #         debugger.draw_circle(int(p[0]), int(p[1]))
        if ((vision.my_robot.x - goal_x)**2 + (vision.my_robot.y - goal_y)**2)**0.5 < 100:
            goal_x, goal_y = -goal_x, -goal_y
        path = RRT.Create_RRT_accuate(vision, goal_x, goal_y)
        #path = RRT.Create_RRT_crude(vision, goal_x, goal_y)
        # path_x, path_y = RRT.Smooth_path(vision, np.array(path)[:, 0], np.array(path)[0, :])
        # debugger.draw_all(path_x, path_y)
        # move_to_pose(vision,np.array(path)[:, 0], np.array(path)[0, :])
        path_x1 = []
        path_y1 = []
        for i in range(0, len(np.array(path)[:, 0])):
            path_x1.append(np.array(path)[-1 - i, 0])
            path_y1.append(np.array(path)[-1 - i, 1])
        time.sleep(0.02)
        if ((vision.my_robot.x - goal_x)**2 + (vision.my_robot.y - goal_y)**2)**0.5 < 300:
            path_x1=[path_x1[-1],goal_x]
            path_y1=[path_y1[-1],goal_y]
        time.sleep(0.05)
        debugger.draw_all(np.array(path)[:, 0], np.array(path)[:, 1])
        feedback.move_to_pose(vision=vision, path_x=path_x1, path_y=path_y1)

        # # 画多个点
        # debugger.draw_points(package, x=[1000, 2000], y=[3000, 3000])
        # 发送调试信息
        # debugger.send(package)

        # time.sleep(0.01)
        # action.controlObs(vision)

