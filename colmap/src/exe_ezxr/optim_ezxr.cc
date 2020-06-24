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

#include "cJSON.h"

using namespace colmap;
using namespace std;

void PrintUsage(); 
JsonConfig GetConfigInfo(std::string config_json_path);

struct JsonConfig {
    std::string input_model_path;
    std::string output_model_path;
    std::string plane_constraint_path;
    std::string pose_geos_path;
    std::string tum_result_path;
    int plane_num;
    std::vector<double> plane_weight;
    double pose_weight;
    bool plane_constraint_enable;
    bool pose_constraint_enable;
};


int main(int argc, char** argv) {
  InitializeGlog(argv);
  
  if (argc < 2)
  {
    std::cout << "Usage : /optim_ezxr /path_to_config.json" << std::endl;
    return -1;
  }
  
  JsonConfig config = GetConfigInfo(argv[1]);

  cout << "Optim test Start!" << endl;
  OptionManager options;
  options.AddBundleAdjustmentOptions();

  
  Reconstruction reconstruction;
  reconstruction.Read(config.input_model_path);

  cout << "Reconstruction read success!" << endl;

  // BA controller 来控制 BA问题构建
  BundleAdjustmentController_ezxr ba_controller(options, &reconstruction);
  // ba_controller.AddPlaneConstraint(config.plane_constraint_path, config.plane_weight);
  // ba_controller.AddPoseConstraint(config.pose_geos_path, config.pose_weight);
  // ba_controller.SaveImageTraj(config.tum_result_path);
  ba_controller.SetConfig(config);
  ba_controller.Start();
  ba_controller.Wait();

  cout << "Optim Finished!" << endl;

  reconstruction.Write(config.output_model_path); 

  cout << "Result Write Success!" << endl; 
  return 1;
}


JsonConfig GetConfigInfo(std::string config_json_path){
    
    cJSON *json;
    std::ifstream infile;
    infile.open(config_json_path);
    infile.seekg(0, std::ios::end);    // go to the end  
    int length = infile.tellg();           // report location (this is the length)  
    infile.seekg(0, std::ios::beg);    // go back to the beginning  
    char* buffer = new char[length];    // allocate memory for a buffer of appropriate dimension  
    infile.read(buffer, length);       // read the whole file into the buffer  
    infile.close();     

    char* text = buffer;

    JsonConfig config;

    json = cJSON_Parse(text);
    if(NULL == json)
    {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
        exit(0);
    }

    cJSON *json_info;

    // get input_model_path 
    json_info = cJSON_GetObjectItem(json, "input_model_path");
    if(json_info->type == cJSON_String)
        config.input_model_path = json_info->valuestring;
    
    // get output_model_path 
    json_info = cJSON_GetObjectItem(json, "output_model_path");
    if(json_info->type == cJSON_String)
        config.output_model_path = json_info->valuestring;

    // get plane_constraint_path
    json_info = cJSON_GetObjectItem(json, "plane_constraint_path");
    if(json_info->type == cJSON_String)
        config.plane_constraint_path = json_info->valuestring;

    // get pose_geos_path 
    json_info = cJSON_GetObjectItem(json, "pose_geos_path");
    if(json_info->type == cJSON_String)
        config.pose_geos_path = json_info->valuestring;

    // get tum_result_path
    json_info = cJSON_GetObjectItem(json, "tum_result_path");
    if(json_info->type == cJSON_String)
        config.tum_result_path = json_info->valuestring;

    // get plane_num 
    json_info = cJSON_GetObjectItem(json, "plane_num");
    if (json_info->type == cJSON_Number)
        config.plane_num = json_info->valueint;

    //get plane_weight 
    json_info = cJSON_GetObjectItem(json, "plane_weight");
    cJSON *weight = NULL;
    cJSON_ArrayForEach(weight, json_info){
        if (weight->type == cJSON_Number)
            config.plane_weight.push_back(weight->valuedouble);
    }
    
    // get pose_weight 
    json_info = cJSON_GetObjectItem(json, "pose_weight");
    if (json_info->type == cJSON_Number)
        config.pose_weight = json_info->valuedouble;
    
    // get plane_constraint_enable
    json_info = cJSON_GetObjectItem(json , "plane_constraint_enable");
    if ( cJSON_IsBool(json_info))
    {
        if(json_info->valueint)
            config.plane_constraint_enable = true;
        else
            config.plane_constraint_enable = false;
    }
    
    // get pose_constraint_enable
    json_info = cJSON_GetObjectItem(json , "pose_constraint_enable");
    if ( cJSON_IsBool(json_info))
    {
        if(json_info->valueint)
            config.pose_constraint_enable = true;
        else
            config.pose_constraint_enable = false;
    }

    cJSON_Delete(json);

    // Print config 
    std::cout << "===> input_model_path : " << config.input_model_path << std::endl;
    std::cout << "===> output_model_path : " << config.output_model_path << std::endl;
    std::cout << "===> plane_constraint_path : " << config.plane_constraint_path << std::endl;
    std::cout << "===> pose_geos_path : " << config.pose_geos_path << std::endl;
    std::cout << "===> tum_result_path : " << config.tum_result_path << std::endl;
    std::cout << "===> plane_num : " << config.plane_num << std::endl;
    std::cout << "===> plane_weight : [";
    for (size_t i = 0; i < config.plane_weight.size(); i++)
    {
        if (i < config.plane_weight.size()-1)
            std::cout << config.plane_weight[i] << ",";
        else
            std::cout << config.plane_weight[i] << "]" << std::endl;
    }
    std::cout << "===> pose_weight : " << config.pose_weight << std::endl;
    std::cout << "===> plane_constraint_enable : " << config.plane_constraint_enable << std::endl;
    std::cout << "===> pose_constraint_enable : " << config.pose_constraint_enable << std::endl;
    return config;

}