import open3d
import numpy as np
import pcl
import os
def triangle_pcd():
    '''
    定义三角形的点云
    :return:
    '''
    from open3d.open3d.utility import Vector3dVector
    from open3d.open3d.utility import Vector2iVector
    triangle_points = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32)
    lines = [[0, 1], [1, 2], [2, 0]]  # Right leg
    colors = [[0, 0, 1] for i in range(len(lines))]  # Default blue
    # 定义三角形的三个角点
    point_pcd = open3d.geometry.PointCloud()  # 定义点云
    point_pcd.points = Vector3dVector(triangle_points)

    # 定义三角形三条连接线

    line_pcd = open3d.open3d.geometry.LineSet()

    line_pcd.lines = Vector2iVector(lines)
    line_pcd.colors = Vector3dVector(colors)
    line_pcd.points = Vector3dVector(triangle_points)

    return line_pcd, point_pcd

def visualize(pointcloud):
    from open3d.open3d.geometry import PointCloud
    from open3d.open3d.utility import Vector3dVector
    from open3d.open3d.visualization import draw_geometries

    # from open3d_study import *


    # points = np.random.rand(10000, 3)
    point_cloud = PointCloud()
    point_cloud.points = Vector3dVector(pointcloud[:,0:3].reshape(-1,3))
    # draw_geometries([point_cloud])
    # 方法1（非阻塞显示）

    line_pcd, point_pcd = triangle_pcd()
    vis = open3d.open3d.visualization.Visualizer()
    vis.create_window(window_name="Open3D")
    vis.add_geometry(point_cloud)
    vis.add_geometry(line_pcd)
    vis.add_geometry(point_pcd)
    vis.run()


if __name__ == '__main__':

    plane_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603/data/lidar_plane/planes'

    dirfiles = os.listdir(plane_path)

    for dirfile in dirfiles:
        if dirfile[-3:] != 'pcd':
            continue
        pointcloud = pcl.load(plane_path + '/' + dirfile)
        with open(plane_path + '/' + dirfile[0:-4] + '_ransac_result.txt','r') as f:
            lines = f.readlines()
        print(lines)
        print("Pointcloud Num = ",pointcloud.size)
        pointcloud_np = np.array(pointcloud)
        visualize(pointcloud_np)
        break