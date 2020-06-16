from io_tools import *
import numpy as np
import cv2
import csv
import lie_algebra as la

def tum_str_to_transform(tum):
    # str 2 float
    x = float(tum[1])
    y = float(tum[2])
    z = float(tum[3])
    qx = float(tum[4])
    qy = float(tum[5])
    qz = float(tum[6])
    qw = float(tum[7])
    # transform inverse
    quat = np.array([qw, qx, qy, qz])
    transform = la.tr.quaternion_matrix(quat)
    transform[0, 3] = x
    transform[1, 3] = y
    transform[2, 3] = z
    # t0 = np.array([[ 0.82810756, -0.56048199 , 0.00989007 ,-0.17192686],
    #                  [-0.00943854, -0.03158144 ,-0.99945662 , 0.03901846],
    #                  [ 0.56048977 , 0.82756423 ,-0.03144296 ,-0.02323644],
    #                  [ 0.0     ,     0.0    ,      0.0      ,    1.0   ]])
    # print(np.dot(np.linalg.inv(t0),np.linalg.inv(transform)))
    return transform

def tum_data(line):
    line = line.strip()
    strs = line.split(' ')
    return strs

def read_tum_txt(file_path):
    tum_list = []
    with open(file_path) as fp:
        line = fp.readline()
        tum_list.append(tum_data(line))
        while line:
            line = fp.readline()
            if(len(line) ==0):
                break
            tum_list.append(tum_data(line))
    return tum_list

def is_big_move(tum_pre, tum_cur, max_time_seconds, max_distance_meters, max_angle_radians):
    '''
    传进来的tum是str形式的一行tum数据
    '''
    # tum格式的timestamp是s为单位
    delta_time = float(tum_cur[0]) - float(tum_pre[0])
    # print("time = ",float(tum_cur[0]))
    tr_pre = tum_str_to_transform(tum_pre)
    tr_cur = tum_str_to_transform(tum_cur)
    delta_tr = np.matmul(la.tr.inverse_matrix(tr_pre), tr_cur)
    delta_translation = np.array([delta_tr[0, 3], delta_tr[1, 3], delta_tr[2, 3]])
    delta_distance = np.linalg.norm(delta_translation)
    delta_angle, _, _ = la.tr.rotation_from_matrix(delta_tr)
    # 注意: 这里和cartographer不同, 按照位移,角度,时间的重要程度进行判断
    # print("delta_time = ",delta_time)
    # print("delta_distance = ",delta_distance)
    # print("delta_angle = ",delta_angle)
    return delta_time > max_time_seconds or delta_distance > max_distance_meters or delta_angle > max_angle_radians

if __name__ == '__main__':
    cam_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/raw_data/2020-06-01-19-49-55_cut'
    tum_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/raw_data/2020-06-01-19-49-55_cut.bag_tum.txt'
    out_path = "/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try"


    # 打开data.csv文件
    f_cam0 = open(cam_path + "/mav0/cam0/data.csv", "r")
    cam0_csv = csv.reader(f_cam0)
    cam0_list = []
    for row in cam0_csv:
        cam0_list.append(row)
    f_cam1 = open(cam_path + "/mav0/cam1/data.csv", "r")
    cam1_csv = csv.reader(f_cam1)
    cam1_list = []
    for row in cam1_csv:
        cam1_list.append(row)
    f_cam0.close()
    f_cam1.close()

    # 保留双目同步的图像时间戳
    stereo_image_timestamp_num = 0
    stereo_image_timestamp = []
    for i in range(len(cam0_list)):
        for j in range(len(cam1_list)):
            if cam0_list[i][0] == cam1_list[j][0]:
                stereo_image_timestamp_num = stereo_image_timestamp_num + 1
                stereo_image_timestamp.append(cam0_list[i])



    # 降采样轨迹
    tum_list = read_tum_txt(tum_path)
    good_tum_num = 0
    image_timestamp_traj = []
    traj_last = []
    down_num = 0
    for i in range(stereo_image_timestamp_num):
        time_str = stereo_image_timestamp[i][0]
        time_image = float(time_str) / 1000000000
        min_time = 10000000.0
        min_tum_index = 0
        for j in range(len(tum_list)):
            time_tum = float(tum_list[j][0])
            delta_time = abs(time_tum - time_image)
            if delta_time < min_time:
                min_time = delta_time
                min_tum_index = j
        if min_time < 0.05:
            good_tum_num = good_tum_num + 1
            if traj_last == []:
                traj_last = tum_list[min_tum_index]
            # elif is_big_move(traj_last, tum_list[min_tum_index], 1.5, 0.8, 0.3):
            elif is_big_move(traj_last, tum_list[min_tum_index], 2, 1.2, 0.5):
                traj_last = tum_list[min_tum_index]
                down_num = down_num + 1
                image_timestamp_traj.append([stereo_image_timestamp[i], tum_list[min_tum_index]])

    with open(out_path + "/downsample_tum.txt", 'w') as f_out:
        for i in range(len(image_timestamp_traj)):
            f_str = image_timestamp_traj[i][0][0] + ' ' + \
                    image_timestamp_traj[i][1][1] + ' ' + \
                    image_timestamp_traj[i][1][2] + ' ' + \
                    image_timestamp_traj[i][1][3] + ' ' + \
                    image_timestamp_traj[i][1][4] + ' ' + \
                    image_timestamp_traj[i][1][5] + ' ' + \
                    image_timestamp_traj[i][1][6] + ' ' + \
                    image_timestamp_traj[i][1][7]
            f_out.write(f_str + '\n')

    with open(out_path + "/geos.txt", 'w') as f_out:
        for i in range(len(image_timestamp_traj)):
            f_str = image_timestamp_traj[i][0][1]
            f_out.write('cam0/' + f_str + ' ' + image_timestamp_traj[i][1][1] + ' ' + \
                    image_timestamp_traj[i][1][2] + ' ' + \
                    image_timestamp_traj[i][1][3] + '\n')
    for i in range(len(image_timestamp_traj)):
        im = cv2.imread(cam_path + "/mav0/cam0/data/" + image_timestamp_traj[i][0][1])
        # cv2.imshow("im", im)
        # cv2.waitKey(1)
    print("down num = ", down_num)
    print("good num = ", good_tum_num)
    print("stereo image num = ", stereo_image_timestamp_num)

    f = open(out_path + "/downsample_tum.txt", 'r')
    image_path = cam_path
    image_save_path = out_path + '/images_downsample'
    mkdir(image_save_path)
    mkdir(image_save_path+'/cam0')
    mkdir(image_save_path+'/cam1'
          )
    lines = f.readlines()
    f.close()

    for i in range(len(lines)):
        time = lines[i].split()[0]
        # print(lines[i].split()[0])
        image1 = cv2.imread(image_path + '/mav0/cam0/data/' + time + '.png')
        image2 = cv2.imread(image_path + '/mav0/cam1/data/' + time + '.png')
        if image1.shape[0] == 0 or image2.shape[0] == 0:
            print('Lost Frame ',time ,'.png')
            continue

        cv2.imwrite(image_save_path + '/cam0/' + time + '.png', image1)
        cv2.imwrite(image_save_path + '/cam1/' + time + '.png', image2)

    print("Finish!")