// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef COLMAP_SRC_CONTROLLERS_BUNDLE_ADJUSTMENT_H_
#define COLMAP_SRC_CONTROLLERS_BUNDLE_ADJUSTMENT_H_

#include "base/reconstruction.h"
#include "util/option_manager.h"
#include "util/threading.h"
#include "vector"
namespace colmap {

// Class that controls the global bundle adjustment procedure.
class BundleAdjustmentController : public Thread {
 public:
  BundleAdjustmentController(const OptionManager& options,
                             Reconstruction* reconstruction);

 private:
  void Run();

  const OptionManager options_;
  Reconstruction* reconstruction_;
};


// Added by ezxr-sx-zhangqunkang
// Add Start

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
    bool save_image_traj_enable;
};

// Class that controls the global bundle adjustment procedure.
class BundleAdjustmentController_ezxr : public Thread {
 public:
  BundleAdjustmentController_ezxr(const OptionManager& options,
                             Reconstruction* reconstruction);
  void AddPlaneConstraint(std::string plane_constraint_path, const std::vector<double> plane_weight);
  void AddPoseConstraint(std::string pose_geos_path, const double weight);
  void SaveImageTraj(std::string tum_path);
  void SetConfig(JsonConfig config);
 private:
  void Run();

  const OptionManager options_;
  Reconstruction* reconstruction_;
  std::string plane_constraint_path_;
  std::string pose_geos_path_;
  std::string tum_path_;
  double pose_weight_;
  std::vector<double> plane_weight_;
  bool plane_constraint_;
  bool pose_geos_constraint_;
  bool save_image_traj_;
};
// Add End

}  // namespace colmap

#endif  // COLMAP_SRC_CONTROLLERS_BUNDLE_ADJUSTMENT_H_
