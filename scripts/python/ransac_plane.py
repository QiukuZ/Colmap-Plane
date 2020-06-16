import os

if __name__ == '__main__':
    plane_files_path = '/media/qk/RichMan/test/0603_long/lidar_plane/planes'
    dir_file_lists = os.listdir(plane_files_path)

    for dirfile in dir_file_lists:
        if dirfile[-3:] != 'pcd':
            continue
        print(dirfile)
        cmd = "./../../plane_ransac/build/plane_ransac "
        cmd = cmd + plane_files_path + '/' + dirfile + ' '
        cmd = cmd + plane_files_path + '/' + dirfile[0:-4] + '_ransac_result.txt'
        print(cmd)
        os.system(cmd)
