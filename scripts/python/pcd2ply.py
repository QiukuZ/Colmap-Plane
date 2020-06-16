import pcl
if __name__ == '__main__':
    pcd_path = '/home/alex/Desktop/Lidar_Visual_Plane/0603/data/raw_data/2020-06-01-19-49-55_cut.bag_pointcloud.pcd'
    p = pcl.load(pcd_path)
    pcl.save(p,pcd_path[0:-3]+'.ply')