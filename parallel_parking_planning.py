#!/usr/bin/python
# coding=utf-8

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import numpy as np
from copy import copy
from time import sleep


def calculate_forward_point(start_point_, length_, start_heading_):
    end_point = np.array(start_point_) + calculate_vector(length_, start_heading_)

    return end_point

def calculate_vector(length_, heading_):
    return length_ * np.array([math.cos(heading_), math.sin(heading_)])


# 计算点距离向量的横向距离，线的左侧为正数， 右侧为负数

def calculate_dis_point_to_line(one, two, three):

    line = two - one
    line = line / np.linalg.norm(line)

    line1 = three - one

    line = np.append(line, 0.0)
    line1 = np.append(line1, 0.0)
    result = np.cross(line, line1)

    return result[2]

# 计算侧方泊车位的四个角点位置，第一个点M， N， E， F

#   M |                      | F
#     |                      |
#     |                      |
#   N |----------------------| E


def calculate_four_parking_points(M_, parking_heading_, parking_spot_length_, parking_spot_width_):
    heading_MN = parking_heading_ - math.pi / 2.0
    heading_NE = parking_heading_
    heading_EF = parking_heading_ + math.pi / 2.0
    M = np.array(M_)
    N = calculate_forward_point(M, parking_spot_width_, heading_MN)
    E = calculate_forward_point(N, parking_spot_length_, heading_NE)
    F = calculate_forward_point(E, parking_spot_width_, heading_EF)

    return np.array([M, N, E, F])

# 计算理想情况第一次的车辆后轴中心的坐标 x y heading  (m m rad)

def calculate_fisrt_O(M_, parking_heading_, car_width_, distance_to_rear_):
    heading_MN = parking_heading_ - math.pi / 2.0
    O = calculate_forward_point(M_, car_width_ / 2.0, heading_MN) + calculate_vector(distance_to_rear_, parking_heading_)
    O = np.append(O, parking_heading_)

    return O


# 通过车辆后轴中心和车辆的空间信息计算车辆的四个点的坐标

# A |————————————————————| D
#   |                    |
#   |   O-------->       |
#   |                    |
# B |————————————————————| C

def calculate_car_five_points(O_, car_width_, distance_to_rear_, distance_to_front_):
    heading_AB = O_[2] - math.pi / 2.0
    heading_BC = O_[2]
    heading_CD = O_[2] + math.pi / 2.0
    heading_DA = O_[2] + math.pi


    A = calculate_forward_point(O_[0:2], distance_to_rear_, heading_DA) + calculate_vector(car_width_ / 2.0, heading_CD)
    B = calculate_forward_point(O_[0:2], distance_to_rear_, heading_DA) + calculate_vector(car_width_ / 2.0, heading_AB)
    C = calculate_forward_point(O_[0:2], distance_to_front_, heading_BC) + calculate_vector(car_width_ / 2.0, heading_AB)
    D = calculate_forward_point(O_[0:2], distance_to_front_, heading_BC) + calculate_vector(car_width_ / 2.0, heading_CD)
    return np.array([A, B, C, D, A])





if __name__ == '__main__':

    fig, ax = plt.subplots()
    ax.set_aspect('equal', adjustable='box')

    # 侧方泊车位的信息, 角度单位degree
    parking_M = np.array([0.0, 0.0])
    parking_heading = 0.0
    parking_spot_length = 5.6
    parking_spot_width = 2.28

    # 车辆的空间信息
    wheel_base = 2.85
    max_angle = 30
    distance_to_rear = 1.2
    distance_to_front = 3.6
    car_width = 1.9

    # 路径采样的长度 

    ds = 0.05


    # 计算侧方泊车位的四个点

    four_parking_points = calculate_four_parking_points(parking_M, parking_heading / 180.0 * math.pi, parking_spot_length, parking_spot_width)


    first_O = calculate_fisrt_O(parking_M, parking_heading / 180.0 * math.pi, car_width, distance_to_rear)

    car_points = calculate_car_five_points(first_O, car_width, distance_to_rear, distance_to_front)

    # print(first_O)

    
    car_O = copy(first_O)
    

    min_r = wheel_base / math.tan(max_angle / 180.0 * math.pi)


    car_path = []


    ax.set_xlim([-parking_spot_length, parking_spot_length * 3.0])
    ax.set_ylim([-parking_spot_width - 2.0, 2 * parking_spot_width + 2.0])

    parking_xs = four_parking_points[:,0]
    parking_ys = four_parking_points[:,1]


    ax.plot(parking_xs, parking_ys)

# 大致的侧方位泊车的几何示意图，包括关键点信息的显示和字母标注
#
#               P_O |
#                   |
#                   |
#                   |
#                   |                                
#    M |   A |-------------------| D     | F
#      |     |      |            |       |
#      |     | car_O|----------->|       |
#      |     |                   |       |
#      |   B |-------------------| C     |
#      |                                 |
#    N |---------------------------------| E
    

    while True:
        P_O = car_O[0:2] + calculate_vector(min_r, car_O[2] + math.pi / 2.0)

        dis_P_O__C = np.linalg.norm(P_O - car_points[2])

        dis_P_O__F = np.linalg.norm(P_O - four_parking_points[3])

        d_thelta = ds / min_r

        thelta = car_O[2] - math.pi / 2.0

        if dis_P_O__F > (dis_P_O__C):
            print("一次出库！")
            
            # 左打满向前，直到车右边的距离距离库角F大于0.2m
            while True:
                car_O = np.append(P_O + calculate_vector(min_r, thelta), thelta + math.pi / 2.0)
                car_points = calculate_car_five_points(car_O, car_width, distance_to_rear, distance_to_front)
                result = calculate_dis_point_to_line(car_points[1], car_points[2], four_parking_points[3])

                if result > -0.2:
                    car_path.append(car_O)
                    car_xs = car_points[:, 0]
                    car_ys = car_points[:, 1]
                    ax.plot(car_xs, car_ys, color='green')

                    fig.canvas.draw_idle()
                    plt.pause(0.2)

                    thelta = thelta + d_thelta
                else:
                    break

                
            
            
            # 直线向前行驶，直到后轮过库角F
            # car_O = car_path[-1]
            car_points = calculate_car_five_points(car_O, car_width, distance_to_rear, distance_to_front)
            result = calculate_dis_point_to_line(car_points[0], car_points[1], four_parking_points[3])

            while result > distance_to_rear:
                car_path.append(car_O)
                car_xs = car_points[:, 0]
                car_ys = car_points[:, 1]
                ax.plot(car_xs, car_ys, color='green')

                fig.canvas.draw_idle()
                plt.pause(0.2)

                car_O = np.append(car_O[0:2] + calculate_vector(ds, car_O[2]), car_O[2])

                car_points = calculate_car_five_points(car_O, car_width, distance_to_rear, distance_to_front)
                result = calculate_dis_point_to_line(car_points[0], car_points[1], four_parking_points[3])


            
            break
        else:
            print("多次调整出库")

            # 左打满向前，直到车子右前方的角C碰到车库前面的边界线EF

            while True:
                car_O = np.append(P_O + calculate_vector(min_r, thelta), thelta + math.pi / 2.0)

                car_points = calculate_car_five_points(car_O, car_width, distance_to_rear, distance_to_front)

                result = calculate_dis_point_to_line(four_parking_points[2], four_parking_points[3], car_points[2])

                if result > 0.05:
                    car_path.append(car_O)
                    car_xs = car_points[:, 0]
                    car_ys = car_points[:, 1]
                    ax.plot(car_xs, car_ys, color='green')

                    fig.canvas.draw_idle()
                    plt.pause(0.2)

                    thelta = thelta + d_thelta

                else:
                    break
            





            P_O = car_O[0:2] + calculate_vector(min_r, car_O[2] - math.pi / 2.0)

            thelta = thelta + math.pi
            # 右打满向后，直到车子后方的角A碰到车库后面的边界线MN或者角B碰到车库后面的边界线NE

            while True:

                
                car_O = np.append(P_O + calculate_vector(min_r, thelta), thelta - math.pi / 2.0)

                car_points = calculate_car_five_points(car_O, car_width, distance_to_rear, distance_to_front)

                result = calculate_dis_point_to_line(four_parking_points[0], four_parking_points[1], car_points[0])

                result1 = calculate_dis_point_to_line(four_parking_points[1], four_parking_points[2], car_points[1])

                if result > 0.05 and result1 > 0.05:
                    car_path.append(car_O)                
                    car_xs = car_points[:, 0]
                    car_ys = car_points[:, 1]
                    ax.plot(car_xs, car_ys, color='green')
                    
                    fig.canvas.draw_idle()
                    plt.pause(0.2)

                    thelta = thelta + d_thelta
                else:
                    break



    car_path = np.array(car_path)
    ax.scatter(car_path[:,0], car_path[:,1], s = 2)

    
    

    plt.show()




