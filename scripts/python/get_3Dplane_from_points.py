from io_tools import *
import copy
import numpy as np
import pcl


def xy2uv(Points,Info):
    r = Info[2]
    max_row = Info[0]
    max_col = Info[1]
    RowCol = [max_row, max_col] - Points[:, 0:2] / r
    RowCol = RowCol.astype('int')
    UV = copy.deepcopy(RowCol)
    UV[:,0] = RowCol[:,1]
    UV[:,1] = RowCol[:,0]
    return UV

def uv2xy(Points,Info):
    r = Info[2]
    max_row = Info[0]
    max_col = Info[1]
    YX = r * ([max_col,max_row] - Points[:, 0:2])
    XY = copy.deepcopy(YX)
    XY[:,0] = YX[:,1]
    XY[:,1] = YX[:,0]
    return XY

def get_png_info(txt_path):
    f = open(txt_path)
    lines = f.readlines()
    resolution, max_global_ROW_index, max_global_COL_index = 0.0, 0, 0
    for line in lines:
        if line[0:14] == "resolution(m):":
            resolution = float(line[14:])
        elif line[0:14] == "max_global_ROW":
            max_global_ROW_index = int(line[21:])
        elif line[0:14] == "max_global_COL":
            max_global_COL_index = int(line[21:])
    f.close()

    print("resolution : ", resolution)
    print("max_global_ROW_index :", max_global_ROW_index)
    print("max_global_COL_index :", max_global_COL_index)

    png_Info = [max_global_ROW_index, max_global_COL_index, resolution]
    return png_Info

def get_png_points(points_path):
    lines = []
    with open(points_path,'r') as f:
        points = f.readlines()
        for line in points:
            p = line.split()
            p1 = [float(p[0]),float(p[1])]
            p2 = [float(p[2]),float(p[3])]
            lines.append([p1,p2])
    lines = np.array(lines)
    return lines

if __name__ == '__main__':

    txt_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/raw_data/2020-06-01-19-49-55_cut.bag_xray_xy_all.txt'
    png_Info = get_png_info(txt_path)

    points_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/lidar_plane/png_points.txt'
    pcl_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/raw_data/2020-06-01-19-49-55_cut.bag_pointcloud.pcd'
    pcd_output_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/lidar_plane'


    pointcloud_raw = pcl.load(pcl_path)
    pointcloud_np = np.array(pointcloud_raw)
    print('point num = ',pointcloud_raw.size)
    #获得平面信息点对
    pointcloud_select = np.array([])
    lines_uv = get_png_points(points_path)
    pointcloud_list =[]
    mkdir(pcd_output_path+'/planes')
    lines_xy = []

    for line_uv in lines_uv:
        line_xy = uv2xy(line_uv,png_Info)
        lines_xy.append(line_xy)
        # AB = [0] - [1] * 1
        AB_vec = line_xy[0] - line_xy[1]

        '''
        第一遍筛选，保留投影在线段内的点
        '''
        # PA = P - [0] * n
        PA_vec = pointcloud_np[:,0:2] - line_xy[0]
        # cos 1 = dot(AB,PA)/|AB|*|PA|
        PA_dis = np.sqrt(PA_vec[:, 0] * PA_vec[:, 0] + PA_vec[:, 1] * PA_vec[:, 1])
        cos_1 = (AB_vec[0] * PA_vec[:, 0] + AB_vec[1] * PA_vec[:, 1]) / (np.linalg.norm(AB_vec) * (PA_dis))
        # PB = P - [1] * n
        PB_vec = pointcloud_np[:,0:2] - line_xy[1]
        # cos 2 = dot(AB,PB)/|AB|*|PB|
        PB_dis = np.sqrt(PB_vec[:, 0] * PB_vec[:, 0] + PB_vec[:, 1] * PB_vec[:, 1])
        cos_2 = (AB_vec[0] * PB_vec[:, 0] + AB_vec[1] * PB_vec[:, 1]) / (np.linalg.norm(AB_vec) * (PB_dis))

        pointcloud_sub1 = pointcloud_np[np.where(cos_1 * cos_2 < 0)[0]]

        '''
        第二遍筛选，保留距离小于阈值的点
        '''
        # PA = P - [0] * n
        PA_dis_sub = PA_dis[np.where(cos_1 * cos_2 < 0)[0]]
        cos_1_sub = cos_1[np.where(cos_1 * cos_2 < 0)[0]]
        P_line_dis = np.sqrt(1-cos_1_sub*cos_1_sub)*PA_dis_sub
        pointcloud_sub = pointcloud_sub1[np.where(P_line_dis < 0.05)[0]]
        pointcloud = pcl.PointCloud()
        pointcloud.from_array(pointcloud_sub)
        pcl.save(pointcloud,pcd_output_path+'/planes/plane' + str(len(pointcloud_list)) + '.pcd')
        pointcloud_list.append(pointcloud_sub)
        print('pointcloud_sub num = ', pointcloud_sub.shape)
        '''
        合并点云
        '''
        if pointcloud_select.size == 0:
            pointcloud_select = copy.deepcopy(pointcloud_sub)
        else:
            pointcloud_select = np.concatenate((pointcloud_select, pointcloud_sub), axis=0)

    with open(pcd_output_path+'/planes/plane_info.txt','w') as f:
        for i in range(len(lines_xy)):
            f.write("plane" + str(i) + ' ' + str(lines_xy[i][0][0]) +
                                       ' ' + str(lines_xy[i][0][1]) +
                                       ' ' + str(lines_xy[i][1][0]) +
                                       ' ' + str(lines_xy[i][1][1]) + ' \n')
    print('pointcloud_select num = ',pointcloud_select.shape)
    pointcloud = pcl.PointCloud()
    pointcloud.from_array(pointcloud_select)
    # pcl.save(pointcloud,pcl_path[:-4]+'_plane.pcd')
    # pcl.save(pointcloud,'/home/alex/Desktop/a.pcd')
    pcl.save(pointcloud,pcd_output_path+'/all.pcd')

