"""
A* grid based planning

author: Atsushi Sakai(@Atsushi_twi)
"""

import matplotlib.pyplot as plt
import math
import sys
import numpy as np

show_animation = True


class Node:

    def __init__(self, x, y, cost, pind):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.pind)


def calc_fianl_path(ngoal, closedset, reso):
    # generate final course
    rx, ry = [ngoal.x * reso], [ngoal.y * reso]
    pind = ngoal.pind
    while pind != -1:
        n = closedset[pind]
        rx.append(n.x * reso)
        ry.append(n.y * reso)
        pind = n.pind

    return rx, ry


def a_star_planning(sx, sy, gx, gy, ox, oy, reso, rr):
    """
    gx: goal x position [m]
    gx: goal x position [m]
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    reso: grid resolution [m]
    rr: robot radius[m]
    """

    nstart = Node(round(sx / reso), round(sy / reso), 0.0, -1)
    ngoal = Node(round(gx / reso), round(gy / reso), 0.0, -1)
    ox = [iox / reso for iox in ox]
    oy = [ioy / reso for ioy in oy]

    obmap, minx, miny, maxx, maxy, xw, yw = calc_obstacle_map(ox, oy, reso, rr)

    motion = get_motion_model()

    openset, closedset = dict(), dict()
    openset[calc_index(nstart, xw, minx, miny)] = nstart

    while 1:
        c_id = min(
            openset, key=lambda o: openset[o].cost + calc_h(ngoal, openset[o].x, openset[o].y))
        current = openset[c_id]
        #  print("current", current)

        # show graph
        if show_animation:
            plt.figure(1)
            plt.plot(current.x * reso, current.y * reso, "xc")
            if len(closedset.keys()) % 10 == 0:
                plt.pause(0.001)

        if current.x == ngoal.x and current.y == ngoal.y:
            print("Find goal")
            ngoal.pind = current.pind
            ngoal.cost = current.cost
            break

        # Remove the item from the open set
        del openset[c_id]
        # Add it to the closed set
        closedset[c_id] = current

        # expand search grid based on motion model
        for i in range(len(motion)):
            node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], c_id)
            n_id = calc_index(node, xw, minx, miny)

            if not verify_node(node, obmap, minx, miny, maxx, maxy):
                continue

            if n_id in closedset:
                continue
            # Otherwise if it is already in the open set
            if n_id in openset:
                if openset[n_id].cost > node.cost:
                    openset[n_id].cost = node.cost
                    openset[n_id].pind = c_id
            else:
                openset[n_id] = node

    rx, ry = calc_fianl_path(ngoal, closedset, reso)

    return rx, ry


def calc_h(ngoal, x, y):
    w = 10.0  # weight of heuristic
    d = w * math.sqrt((ngoal.x - x)**2 + (ngoal.y - y)**2)
    return d


def verify_node(node, obmap, minx, miny, maxx, maxy):

    if node.x < minx:
        return False
    elif node.y < miny:
        return False
    elif node.x >= maxx:
        return False
    elif node.y >= maxy:
        return False

    if obmap[node.x][node.y]:
        return False

    return True


def calc_obstacle_map(ox, oy, reso, vr):

    minx = round(min(ox))
    miny = round(min(oy))
    maxx = round(max(ox))
    maxy = round(max(oy))
    #  print("minx:", minx)
    #  print("miny:", miny)
    #  print("maxx:", maxx)
    #  print("maxy:", maxy)

    xwidth = round(maxx - minx)
    ywidth = round(maxy - miny)
    #  print("xwidth:", xwidth)
    #  print("ywidth:", ywidth)

    # obstacle map generation
    obmap = [[False for i in range(xwidth)] for i in range(ywidth)]
    for ix in range(xwidth):
        x = ix + minx
        for iy in range(ywidth):
            y = iy + miny
            #  print(x, y)
            for iox, ioy in zip(ox, oy):
                d = math.sqrt((iox - x)**2 + (ioy - y)**2)
                if d <= vr / reso:
                    obmap[ix][iy] = True
                    break

    return obmap, minx, miny, maxx, maxy, xwidth, ywidth


def calc_index(node, xwidth, xmin, ymin):
    return (node.y - ymin) * xwidth + (node.x - xmin)


def get_motion_model():
    # dx, dy, cost
    motion = [[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, math.sqrt(2)],
              [-1, 1, math.sqrt(2)],
              [1, -1, math.sqrt(2)],
              [1, 1, math.sqrt(2)]]

    return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    scale2world = 0.06

    ## for load the gate positions
    # open file
    # gate_position = []
    # for line in line:
    #     temp = []
    #     temp = [line[]/60-30.0,line[]/60-30.0]
    #     gate_position = gate_position.append(temp)
    #gate_position = [[30,10],[10,20],[20,20],[30,20],[40,30],[40,40],[30,40],[20,40]]
    gate_position = []
    file = open("gates.dat", "r")
    data = file.read()
    gates = data.split("\n")
    gate = [0,0,0,0]
    for i in range(4):
        temp = gates[i].split(",")
        for j in range(4):
            gate[j] = float(temp[j])
        gate_position.append([gate[0],gate[1]])
        gate_position.append([gate[2],gate[3]])

    gate_position[:] = [[x[0]/scale2world + 30, x[1]/scale2world + 30] for x in gate_position]
    print(gate_position)

    rx , ry = [], []
    Way_position = []
    Way_position.append([34,26])
    #########
    #Way_position = [[34,26],[45,35],[35,35],[25,45],[25,35],[15,15],[35,15]]

    #########
    for i in range(4):
        left_gate = gate_position[2*i]
        right_gate = gate_position[2*i+1]
        print('gateNo.',i,left_gate,right_gate)
        if np.absolute(left_gate[0] - right_gate[0]) < 2:
            if left_gate[1] < right_gate[1]:
                way_pos_start = [left_gate[0]+5,(left_gate[1]+right_gate[1])/2]
                way_pos_goal = [left_gate[0]-5,(left_gate[1]+right_gate[1])/2]
            else:
                way_pos_start = [left_gate[0]-5,(left_gate[1]+right_gate[1])/2]
                way_pos_goal = [left_gate[0]+5,(left_gate[1]+right_gate[1])/2]
        else:
            if left_gate[0] < right_gate[0]:
                way_pos_start = [(left_gate[0]+right_gate[0])/2,left_gate[1]-5]
                way_pos_goal = [(left_gate[0]+right_gate[0])/2,left_gate[1]+5]
            else:
                way_pos_start = [(left_gate[0]+right_gate[0])/2,left_gate[1]+5]
                way_pos_goal = [(left_gate[0]+right_gate[0])/2,left_gate[1]-5]
        Way_position.append(way_pos_start)
        Way_position.append(way_pos_goal)
    print(Way_position)

    for i in range(len(Way_position)-1):
        sx = Way_position[i][0]
        sy = Way_position[i][1]
        gx = Way_position[i+1][0]
        gy = Way_position[i+1][1]

        grid_size = 1.0  # [m]
        robot_size = 4 # [m]
        gate_size = int(1)
        wall_width = int(1)
        wall_length = int(8)
        ox, oy = [], []

    # Tony: Double for loop for blocks  #####
        # for wall 1
        # for i in range(wall_width):
        #     for j in range(2*wall_length):
        #         ox.append(30+i)
        #         oy.append(20+j)
        # # for wall 2
        for i in range(wall_length):
            for j in range(wall_width):
                ox.append(32+i)
                oy.append(30+j)
        # # for wall 3
        # for i in range(wall_length):
        #     for j in range(wall_width):
        #         ox.append(20+i)
        #         oy.append(20+j)
        # for gates
        for gate_pos in gate_position:
            for i in range(0,gate_size):
                for j in range(0,gate_size):
                    ox.append(int(gate_pos[0])+i)
                    oy.append(int(gate_pos[1])+j)

        for i in range(60):
            ox.append(i)
            oy.append(0.0)
        for i in range(60):
            ox.append(60.0)
            oy.append(i)
        for i in range(61):
            ox.append(i)
            oy.append(60.0)
        for i in range(61):
            ox.append(0.0)
            oy.append(i)


        if show_animation:
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "xr")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")

        rx_temp, ry_temp = a_star_planning(sx, sy, gx, gy, ox, oy, grid_size, robot_size)
        rx_temp.reverse()
        ry_temp.reverse()
        rx = rx + rx_temp
        ry = ry + ry_temp
        #print('setpoints',rx,ry)

    rx[:] = [(x-30.0)*scale2world for x in rx]
    ry[:] = [(y-30.0)*scale2world for y in ry]
    rx_new, ry_new = [], []

    #randomly pick setpoints
    # for i in range(0,95,3):
    #     rx_new.append(rx[i])
    #     ry_new.append(ry[i])
    #pick good setpoints
    current_rx = rx[0]
    current_ry = ry[0]
    for i in range(len(rx)-1):
        # if rx[i] != rx[i+1] and ry[i] != ry[i+1]:
        #     if math.tan(math.atan2((ry[i+1]-ry[i]),(rx[i+1]-rx[i]))) !=-1 and math.tan(math.atan2((ry[i+1]-ry[i]),(rx[i+1]-rx[i]))) != 1:
        #         rx_new.append(rx[i])
        #         ry_new.append(ry[i])
        if rx[i] != current_rx and ry[i] != current_ry:
             if np.absolute(np.absolute(math.tan(math.atan2((ry[i+1]-ry[i]),(rx[i+1]-rx[i])))) - 1.0) > 0.01 :
                #print(math.tan(math.atan2((ry[i+1]-ry[i]),(rx[i+1]-rx[i]))))
                rx_new.append(rx[i])
                ry_new.append(ry[i])
                current_rx = rx[i]
                current_ry = ry[i]


    n = len(rx_new)
    data = str(n) +  " 0.2 0.02 \n"
    for i in range(n):
        data = data + str(rx_new[i]) + " " + str(ry_new[i]) + " 0.0 \n"


    file = open("setpoints.cfg","w")
    file.write(data)
    file.close()

    if show_animation:
        plt.figure(2)
        plt.subplot(311)
        plt.plot(rx, ry)
        plt.subplot(312)
        plt.plot(rx_new,ry_new,'*')
        plt.show()



if __name__ == '__main__':
    main()
