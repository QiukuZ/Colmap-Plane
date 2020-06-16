import numpy as np
from read_write_model import *
from tempfile import mkdtemp
import os
from io_tools import *
def get_plane_info(plane_info_path):
    with open(plane_info_path+'/plane_info.txt','r') as f:
        plane_boundarys_str = f.readlines()
    plane_boundarys = []
    plane_coeffs = []
    plane_num = len(plane_boundarys_str)
    for i in range(plane_num):
        plane_boundarys.append([float(plane_boundarys_str[i].split()[1]),
                                float(plane_boundarys_str[i].split()[2]),
                                float(plane_boundarys_str[i].split()[3]),
                                float(plane_boundarys_str[i].split()[4])])
        # print(plane_info_path+'/plane'+str(i)+'_ransac_result.txt')
        with open(plane_info_path+'/plane'+str(i)+'_ransac_result.txt','r') as f:
            plane_coeffs_str = f.readlines()
            # print(plane_coeffs_str[12])
            plane_coeffs.append([float(plane_coeffs_str[12].split()[1]),
                                    float(plane_coeffs_str[12].split()[2]),
                                    float(plane_coeffs_str[12].split()[3]),
                                    float(plane_coeffs_str[12].split()[4])])
    return plane_coeffs,plane_boundarys

def main():
    path_to_model_bin_folder = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/sparse_aligned'
    path_to_plane_folder = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/lidar_plane/planes'
    output_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data'
    cameras, images, points3D = read_model(path_to_model_bin_folder, ext=".bin")

    plane_coeffs, plane_bouyndarys = get_plane_info(path_to_plane_folder)
    plane_point3D_ids = []
    for i in range(len(plane_coeffs)):

        print(plane_bouyndarys)
        points3D_new = {}
        points3D_ids = []
        a,b,c,d = plane_coeffs[i]
        A = np.array([plane_bouyndarys[i][0], plane_bouyndarys[i][1]])
        B = np.array([plane_bouyndarys[i][2], plane_bouyndarys[i][3]])
        AB = A - B
        AB_l = np.linalg.norm(AB)
        print("Plane Coeffs : ",a,b,c,d)
        print("Bouyndarys A : ",A)
        print("Bouyndarys B : ",B)
        print('Point3D Num = ',len(points3D))
        for key in points3D:
            p = points3D[key].xyz
            P = np.array(p[0:2])
            PA = P - A
            PB = P - B

            cos1 = (PA*AB).sum()/(AB_l*np.linalg.norm(PA))
            cos2 = (PB*AB).sum()/(AB_l*np.linalg.norm(PB))

            distance = abs(a*p[0] + b*p[1] + c*p[2] + d)
            # print(distance)
            if distance < 0.2 and cos1*cos2 < 0:
                points3D_ids.append(key)
                points3D_new[key] = Point3D(
                    id=key, xyz=p, rgb=points3D[key].rgb,
                    error=points3D[key].error, image_ids=points3D[key].image_ids,
                    point2D_idxs=points3D[key].point2D_idxs)
        print("Plane ", i ," points3D New Num = ",len(points3D_new))
        plane_point3D_ids.append(points3D_ids)
        output_model_path = output_path + '/sparse_planes/' + str(i)
        mkdir(output_model_path)
        write_model(cameras,images,points3D_new,output_model_path,ext=".bin")
    with open( output_path + '/planes_constraint.txt','w') as f:
        f.write('PlaneNum ' + str(len(plane_coeffs)) + ' \n')
        for i in range(len(plane_coeffs)):
            f.write('Plane ' + str(plane_coeffs[i][0]) + ' ' +
                               str(plane_coeffs[i][1]) + ' ' +
                               str(plane_coeffs[i][2]) + ' ' +
                               str(plane_coeffs[i][3]) + ' \n')
            f.write('PointsNum ' + str(len(plane_point3D_ids[i])) + ' \n')
            f.write('PointIds')
            for j in range(len(plane_point3D_ids[i])):
                f.write(' ' + str(plane_point3D_ids[i][j]))
            f.write(' \n')
if __name__ == '__main__':
    main()
