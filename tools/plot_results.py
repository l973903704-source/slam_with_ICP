#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function

import csv
import os
import sys

try:
    import matplotlib.pyplot as plt
except Exception as exc:
    print('matplotlib 导入失败: %s' % str(exc))
    print('如需画图，请提前安装 python-matplotlib 或 python3-matplotlib')
    sys.exit(1)


PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
RESULT_DIR = os.path.join(PROJECT_ROOT, 'catkin_ws', 'src', 'slamware_loop_slam', 'result')


def read_csv_points(path):
    if not os.path.exists(path):
        return []
    pts = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append((float(row['x']), float(row['y'])))
    return pts


def read_loops(path):
    if not os.path.exists(path):
        return []
    items = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            items.append({
                'from_id': int(row['from_id']),
                'to_id': int(row['to_id']),
                'dx': float(row['dx']),
                'dy': float(row['dy']),
                'dtheta_rad': float(row['dtheta_rad']),
                'icp_error': float(row['icp_error']),
            })
    return items


def read_keyframes(path):
    if not os.path.exists(path):
        return {}
    items = {}
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            items[int(row['id'])] = (
                float(row['optimized_x']),
                float(row['optimized_y']),
            )
    return items


def plot_no_loop(path_csv, out_png):
    pts = read_csv_points(path_csv)
    if not pts:
        print('没有找到无回环轨迹: %s' % path_csv)
        return False
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    plt.figure(figsize=(8, 6))
    plt.plot(xs, ys)
    plt.xlabel('x / m')
    plt.ylabel('y / m')
    plt.title('No-loop ICP trajectory')
    plt.axis('equal')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_png, dpi=150)
    plt.close()
    print('已生成: %s' % out_png)
    return True


def plot_pose_graph(local_csv, opt_csv, loops_csv, keyframes_csv, compare_png, loops_png):
    local_pts = read_csv_points(local_csv)
    opt_pts = read_csv_points(opt_csv)
    loops = read_loops(loops_csv)
    keyframes = read_keyframes(keyframes_csv)

    if not local_pts and not opt_pts:
        print('没有找到回环版轨迹结果')
        return False

    plt.figure(figsize=(8, 6))
    if local_pts:
        plt.plot([p[0] for p in local_pts], [p[1] for p in local_pts], label='local path')
    if opt_pts:
        plt.plot([p[0] for p in opt_pts], [p[1] for p in opt_pts], label='optimized path')
    plt.xlabel('x / m')
    plt.ylabel('y / m')
    plt.title('Pose graph SLAM: local vs optimized')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(compare_png, dpi=150)
    plt.close()
    print('已生成: %s' % compare_png)

    if opt_pts:
        plt.figure(figsize=(8, 6))
        xs = [p[0] for p in opt_pts]
        ys = [p[1] for p in opt_pts]
        plt.plot(xs, ys, label='optimized path')
        for item in loops:
            i = item['from_id']
            j = item['to_id']
            if i in keyframes and j in keyframes:
                plt.plot([keyframes[i][0], keyframes[j][0]], [keyframes[i][1], keyframes[j][1]])
            elif i < len(opt_pts) and j < len(opt_pts):
                plt.plot([opt_pts[i][0], opt_pts[j][0]], [opt_pts[i][1], opt_pts[j][1]])
        plt.xlabel('x / m')
        plt.ylabel('y / m')
        plt.title('Pose graph loops')
        plt.axis('equal')
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(loops_png, dpi=150)
        plt.close()
        print('已生成: %s' % loops_png)
    return True


def main():
    no_loop_csv = os.path.join(RESULT_DIR, 'no_loop_path.csv')
    local_csv = os.path.join(RESULT_DIR, 'pose_graph_local.csv')
    opt_csv = os.path.join(RESULT_DIR, 'pose_graph_optimized.csv')
    loops_csv = os.path.join(RESULT_DIR, 'pose_graph_loops.csv')
    keyframes_csv = os.path.join(RESULT_DIR, 'pose_graph_keyframes.csv')

    plot_no_loop(no_loop_csv, os.path.join(RESULT_DIR, 'no_loop_path.png'))
    plot_pose_graph(
        local_csv,
        opt_csv,
        loops_csv,
        keyframes_csv,
        os.path.join(RESULT_DIR, 'pose_graph_compare.png'),
        os.path.join(RESULT_DIR, 'pose_graph_loops.png')
    )


if __name__ == '__main__':
    main()
