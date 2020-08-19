# coding=utf-8
import os
import random
from math import sqrt

import numpy as np
import bpy
import blensor


"""
提取路网数据并保存到points
"""
points = []
for obj in bpy.data.objects:
    if 'road' in obj.name:
        points.append((obj.location[0], obj.location[1]))
print("len(points) = ", len(points))

with open('points.txt', 'w') as outfile:
    for pt in points:
        outfile.write(str(pt[0]) + " " + str(pt[1]) + "\n")

min_d = 10  # mininum distance between points


def findLongestPath(points, cur_point, path):
    x = cur_point[0]
    y = cur_point[1]
    candidate_points = []
    if (x+min_d, y) in points and (x+min_d, y) not in path:
        candidate_points.append((x+min_d, y))
    if (x-min_d, y) in points and (x-min_d, y) not in path:
        candidate_points.append((x-min_d, y))
    if (x, y+min_d) in points and (x, y+min_d) not in path:
        candidate_points.append((x, y+min_d))
    if (x, y-min_d) in points and (x, y-min_d) not in path:
        candidate_points.append((x, y-min_d))

    while len(candidate_points) != 0:
        cur_point = candidate_points[random.randint(
            0, len(candidate_points)-1)]  # 随机选一个候选点
        path += [cur_point]

        x = cur_point[0]
        y = cur_point[1]
        candidate_points = []
        if (x+min_d, y) in points and (x+min_d, y) not in path:
            candidate_points.append((x+min_d, y))
        if (x-min_d, y) in points and (x-min_d, y) not in path:
            candidate_points.append((x-min_d, y))
        if (x, y+min_d) in points and (x, y+min_d) not in path:
            candidate_points.append((x, y+min_d))
        if (x, y-min_d) in points and (x, y-min_d) not in path:
            candidate_points.append((x, y-min_d))

    return path
    # if len(candidate_points) == 0:  # 到达搜索终点
    #     print("return")
    #     return path
    # else:
    #     cur_point = candidate_points[random.randint(
    #         0, len(candidate_points)-1)]  # 随机选一个候选点
    #     path += [cur_point]
    # max_len = 0
    # for i in range(len(candidate_points)):
    #     print("i/n = ", i+1, "/", len(candidate_points))
    #     tmp_path = findLongestPath(
    #         points, candidate_points[i], list(path)+[candidate_points[i]])
    #     if len(tmp_path) > max_len:
    #         max_path = tmp_path
    #         max_len = len(tmp_path)
    # return max_path


endpoints = []
for pt in points:
    count = 0
    if (pt[0]+min_d, pt[1]) in points:
        count += 1
    if (pt[0]-min_d, pt[1]) in points:
        count += 1
    if (pt[0], pt[1]+min_d) in points:
        count += 1
    if (pt[0], pt[1]-min_d) in points:
        count += 1
    if count == 1:
        endpoints.append(pt)


max_len = 0
for i in range(len(endpoints)):
    cur_point = endpoints[i]
    path = [cur_point]
    long_path = findLongestPath(points, cur_point, path)
    if len(long_path) > max_len:
        longest_path = long_path
        max_len = len(long_path)
if len(endpoints) == 0:  # 没有端点
    cur_point = points[random.randint(0, len(points)-1)]
    path = [cur_point]
    longest_path = findLongestPath(points, cur_point, path)
print("len(longest_path) = ", len(longest_path))

with open('path.txt', 'w') as outfile:
    for pt in longest_path:
        outfile.write(str(pt[0]) + " " + str(pt[1]) + "\n")

# fig = plt.figure(figsize=(10, 10))
# plt.scatter(np.array(points)[:, 0], np.array(points)[:, 1])
# plt.plot(np.array(longest_path)[:, 0], np.array(longest_path)[:, 1])
# plt.show()


"""If the scanner is the default camera it can be accessed for example by bpy.data.objects["Camera"]"""
scanner = bpy.data.objects["Camera"]

"""Sensor settings"""
scanner.scan_type = 'velodyne'
scanner.velodyne_model = 'vlp16'
scanner.local_coordinates = False


# 加上高度
height = 5
path3d = []
for i in range(len(longest_path)):
    path3d.append((longest_path[i][0], longest_path[i][1], height))
print(path3d)
print(len(path3d))

print("start to generate data")

# 写出的点都是转角点，只需要指定转角点
# path1 = [[-65, -85, 5], [-65, -55, 5], [-15, -55, 5], [-15, 25, 5],
#          [45, 25, 5], [45, -25, 5], [-15, -25, 5], [-15, -55, 5], [-65, -55, 5]]

# scanner移动速度设置，1m/s = 3.6km/hr
velocity_mps = 5
# scanner采集频率设置
freq_hz = 10

# 判断为到达下一个目标点的可允许误差
eps = velocity_mps / freq_hz / 2

# 仿真数据存放路径
idx = []
all_sets = os.listdir("/home/hjx/Documents/blender_ws/sim_data/")
for s in all_sets:
    idx.append(int(s.lstrip('set_')))
if len(idx) != 0:
    i = max(idx)+1
else:
    i = 1
# 注意folder末尾需要/,这样生成的pcd才会放在set文件夹下
folder = "/home/hjx/Documents/blender_ws/sim_data/set_{}/".format(i)


def scan_path(given_path, folder):
    # 传感器初始位置
    scanner.location = given_path[0]

    # 毫秒为单位
    timestamp = 12345

    # 确保文件夹路径存在
    if not os.path.exists(folder):
        os.mkdir(folder)

    # 目的点对应id
    target = 1
    while target <= len(given_path):
        filename = folder + "{:0>5d}".format(timestamp) + ".pcd"
        print(filename)

        # 采集当前帧的数据
        blensor.blendodyne.scan_advanced(
            scanner, evd_file=filename, add_noisy_blender_mesh=False, world_transformation=scanner.matrix_world)

        # scanner当前位置的向量
        vec_scanner = np.array(
            [scanner.location[0], scanner.location[1], scanner.location[2]])
        vec_target = np.array(given_path[target])

        # 下一个目标点的距离
        distance = np.linalg.norm(vec_target - vec_scanner)
        # 到达最近的目标点，更新下一个目标点
        if distance < eps and target == len(given_path) - 1:
            break
        elif distance < eps:
            target += 1
            vec_target = np.array(given_path[target])
            distance = np.linalg.norm(vec_target - vec_scanner)

        # 计算下一帧可以运行的距离
        step = min(velocity_mps / freq_hz, distance)

        # 方向向量
        orientation = (vec_target - vec_scanner) / distance

        # 计算下一帧的位置
        offset = step * orientation
        scanner.location[0] += offset[0]
        scanner.location[1] += offset[1]
        scanner.location[2] += offset[2]

        # 毫秒为单位，每次增加一帧对应的毫秒数
        timestamp += int(1000 / freq_hz)


scan_path(path3d, folder)
