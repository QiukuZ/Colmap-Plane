// # Changed by:
// # ezxr-sx-zhangqunkang
// # 

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "base/similarity_transform.h"
#include "controllers/automatic_reconstruction.h"
#include "controllers/bundle_adjustment.h"
#include "controllers/hierarchical_mapper.h"
#include "estimators/coordinate_frame.h"
#include "feature/extraction.h"
#include "feature/matching.h"
#include "feature/utils.h"
#include "mvs/meshing.h"
#include "mvs/patch_match.h"
#include "retrieval/visual_index.h"
#include "ui/main_window.h"
#include "util/opengl_utils.h"
#include "util/version.h"

using namespace colmap;
using namespace std;

void PrintUsage(); 

int main(int argc, char** argv) {
  InitializeGlog(argv);
  
  // if (argc < 7)
  // {
  //   PrintUsage();
  //   return 0;
  // }
  
  string input_path = "/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/sparse_aligned";
  string output_path = "/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/sparse_test";
  string plane_constraint_path = "/home/alex/Desktop/Lidar_Visual_Plane/0603_long_try/data/planes_constraint.txt";

  // string input_path = argv[2];
  // string output_path = argv[4];
  // string plane_constraint_path = argv[6];

  string pose_geos_path = "/home/alex/Desktop/Lidar_Visual_Plane/0603/geos.txt";
  string tum_result_path = "/home/alex/Desktop/Lidar_Visual_Plane/0603/tum_result.txt";

  cout << "Optim test Start!" << endl;
  OptionManager options;
  options.AddBundleAdjustmentOptions();

  // cout << "BA refine_extra_params : " << options.bundle_adjustment->refine_extra_params << endl;
  
  Reconstruction reconstruction;
  reconstruction.Read(input_path);

  cout << "Reconstruction read success!" << endl;

  // BA controller 来控制 BA问题构建
  BundleAdjustmentController_ezxr ba_controller(options, &reconstruction);
  ba_controller.AddPlaneConstraint(plane_constraint_path);
  ba_controller.AddPoseConstraint(pose_geos_path,100.0);
  ba_controller.SaveImageTraj(tum_result_path);
  ba_controller.Start();
  ba_controller.Wait();

  cout << "Optim Finished!" << endl;

  reconstruction.Write(output_path); 

  cout << "Result Write Success!" << endl; 
  return 1;
}


void PrintUsage()
{
  cout << " " << endl;
  cout << "Usage : " << endl;
  cout << "./optim_ezxr " << endl;
  cout << "-input_path /xxx/xxx/xxx " << endl;
  cout << "-output_path /xxx/xxx/xxx " << endl;
  cout << "-plane_constraint_path /xxx/xxx/planes_constraint.txt" << endl;
  cout << " " << endl;
}