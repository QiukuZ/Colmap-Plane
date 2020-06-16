#include <iostream>
#include <string>
// #include <pcl/io/pcd_io.h>
// #include <pcl/console/parse.h>
// #include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_plane.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <eigen3/Eigen/Core>
#include <vector>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/statistical_outlier_removal.h>
struct Result{
  Eigen::VectorXf coeff;
  double p;
  double inlier_num;
};
int main(int argc, char **argv) {

  // 此函数用来对pcd文件平面拟合
  // 每个pcd文件对应一个平面
  // arg 1 = pcd_path
  // arg 2 = dis_th
  if (argc < 3)
  {
    std::cout << "Usage : ./plane_ransac pcd_path result_txt_path" << std::endl;
    return 0;
  }
  
  std::string pcd_path = argv[1];
  std::string pcd_txt_path = argv[2];
  std::vector<double> dis_th = {0.005,0.01,0.02,0.03,0.05,0.1};
  std::cout << dis_th.size() << std::endl;
  // 读取点云文件
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(pcd_path,*cloud);
  std::cout << "Process pcd file : " << pcd_path << std::endl;
  // std::cout << "Distance Threshold :" << dis_th << std::endl;
  std::cout << "Loaded "<< cloud->width * cloud->height << " data points !"<< std::endl;
  std::cout << "***************************************************" << std::endl;

  std::vector<Result> ransac_result;
  ransac_result.clear();

  for (size_t i = 0; i < dis_th.size() ; i++)
  {
    std::vector<int> inliers; // 用于存放合群点
    
    // RANSAC平面拟合
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));        //针对平面模型的对象
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
    ransac.setDistanceThreshold(dis_th[i]);
    ransac.computeModel();
    ransac.getInliers(inliers);
    double p = ransac.getProbability();
    Eigen::VectorXf coeff;
    ransac.getModelCoefficients(coeff);
    std::cout << "Plane  Coeff = "<<coeff[0]<<" "<<coeff[1]<<" "<<coeff[2]<<" "<<coeff[3]<<std::endl;
    std::cout << "Inliers Num = " << inliers.size() << '/' << cloud->width * cloud->height << std::endl;
    std::cout << "P = " << p << std::endl;
    Result result;
    result.coeff = coeff;
    result.p = p;
    result.inlier_num = inliers.size();
    ransac_result.push_back(result);
    if (dis_th[i]==0.01)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
      for (size_t j = 0; j < final->points.size(); j++)
      {
        final->points[j].x = final->points[j].x + 2;
        final->points[j].y = final->points[j].y - 0.2;
      }
      
      pcl::io::savePCDFile("/home/qk/Documents/Lidar_Visual_Plane/0603/data/lidar_plane/planes/test.pcd",*final);
    }
    
  }

  std::ofstream fout;
  fout.open(pcd_txt_path);
  fout.clear();
  fout << "PointsNum " << cloud->width * cloud->height << std::endl;
  for (size_t i = 0; i < ransac_result.size(); i++)
  {
    fout << "DistanTh " << std::to_string(dis_th[i]) << " " << std::endl;
    fout << "Coeffs " << std::to_string(ransac_result[i].coeff[0]) << " "
                      << std::to_string(ransac_result[i].coeff[1]) << " "
                      << std::to_string(ransac_result[i].coeff[2]) << " "
                      << std::to_string(ransac_result[i].coeff[3]) << " " << std::endl;
    fout << "InlierNum " << std::to_string(ransac_result[i].inlier_num) << " " << std::endl;
    fout << "P " << std::to_string(ransac_result[i].p) << " " << std::endl;
    fout << std::endl;
  }
  std::cout << "Result write to " << pcd_txt_path << " " << std::endl;
  std::cout << "***************************************************" << std::endl;
  fout.close();
}
