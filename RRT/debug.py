import socket
import sys
import time

from zss_debug_pb2 import Debug_Msgs, Debug_Msg, Debug_Arc

class Debugger(object):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.debug_address = ('localhost', 20001)

    def draw_circle(self, x, y, radius=300):
        package = Debug_Msgs()
        msg = package.msgs.add()
        msg.type = Debug_Msg.ARC
        msg.color = Debug_Msg.WHITE
        arc = msg.arc
        arc.rectangle.point1.x = x - radius
        arc.rectangle.point1.y = y - radius
        arc.rectangle.point2.x = x + radius
        arc.rectangle.point2.y = y + radius
        arc.start = 0
        arc.end = 360
        arc.FILL = True
        self.sock.sendto(package.SerializeToString(), self.debug_address)

    def draw_line(self, x1, y1, x2, y2):
        package = Debug_Msgs()
        msg = package.msgs.add()
        msg.type = Debug_Msg.LINE
        msg.color = Debug_Msg.WHITE
        line = msg.line
        line.start.x = x1
        line.start.y = y1
        line.end.x = x2
        line.end.y = y2
        line.FORWARD = True
        line.BACK = True
        self.sock.sendto(package.SerializeToString(), self.debug_address)

    def draw_point(self, x, y):
        package = Debug_Msgs()
        msg = package.msgs.add()
        # line 1
        msg.type = Debug_Msg.LINE
        msg.color = Debug_Msg.WHITE
        line = msg.line
        line.start.x = x + 50
        line.start.y = y + 50
        line.end.x = x - 50
        line.end.y = y - 50
        line.FORWARD = True
        line.BACK = True
        # line 2
        msg = package.msgs.add()
        msg.type = Debug_Msg.LINE
        msg.color = Debug_Msg.WHITE
        line = msg.line
        line.start.x = x - 50
        line.start.y = y + 50
        line.end.x = x + 50
        line.end.y = y - 50
        line.FORWARD = True
        line.BACK = True
        self.sock.sendto(package.SerializeToString(), self.debug_address)

    def draw_points(self, package, x, y):
        for i in range(len(x)):
            # line 1
            msg = package.msgs.add()
            msg.type = Debug_Msg.LINE
            msg.color = Debug_Msg.WHITE
            line = msg.line
            line.start.x = x[i] + 50
            line.start.y = y[i] + 50
            line.end.x = x[i] - 50
            line.end.y = y[i] - 50
            line.FORWARD = True
            line.BACK = True
            # line 2
            msg = package.msgs.add()
            msg.type = Debug_Msg.LINE
            msg.color = Debug_Msg.WHITE
            line = msg.line
            line.start.x = x[i] - 50
            line.start.y = y[i] + 50
            line.end.x = x[i] + 50
            line.end.y = y[i] - 50
            line.FORWARD = True
            line.BACK = True

    def draw_roadmap(self, package, sample_x, sample_y, road_map):
        for (i, edges) in zip(range(len(road_map)), road_map):
            # print(edges)
            for edge in edges:
                msg = package.msgs.add()
                msg.type = Debug_Msg.LINE
                msg.color = Debug_Msg.WHITE
                line = msg.line
                line.start.x = sample_x[i]
                line.start.y = sample_y[i]
                line.end.x = sample_x[edge]
                line.end.y = sample_y[edge]
                # print(sample_x[i], sample_y[i], sample_x[edge], sample_y[edge])
                line.FORWARD = True
                line.BACK = True

    def draw_finalpath(self, package, x, y):
        for i in range(len(x)-1):
            msg = package.msgs.add()
            msg.type = Debug_Msg.LINE
            msg.color = Debug_Msg.GREEN
            line = msg.line
            line.start.x = x[i]
            line.start.y = y[i]
            line.end.x = x[i+1]
            line.end.y = y[i+1]
            line.FORWARD = True
            line.BACK = True

    def draw_all(self, path_x, path_y):
        package = Debug_Msgs()
        self.draw_finalpath(package, path_x, path_y)
        self.sock.sendto(package.SerializeToString(), self.debug_address)


if __name__ == '__main__':
    debugger = Debugger()
    while True:
        # debugger.draw_circle(x=100, y=200)
        # debugger.draw_line(x1=100, y1=100, x2=600, y2=600)
        # debugger.draw_point(x=300, y=300)
        debugger.draw_points(x=[1000, 2000], y=[3000, 3000])
        time.sleep(0.02)
