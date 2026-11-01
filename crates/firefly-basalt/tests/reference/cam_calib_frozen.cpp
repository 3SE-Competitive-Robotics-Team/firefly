// SPDX-License-Identifier: BSD-3-Clause
// Replay unmodified official controller from fixed native corner/pose caches.
#include <pangolin/pangolin.h>
#include <pangolin/display/image_view.h>
#include <basalt/calibration/calibration_helper.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/optimization/poses_optimize.h>
#include <fstream>
#include <nlohmann/json.hpp>
#define private public
#include <basalt/calibration/cam_calib.h>
#undef private
int main(int argc, char** argv) {
  if (argc != 5) return 1;
  basalt::CamCalib controller(argv[1], "bag", argv[2], argv[3], "calib-cam", 1, {"ds", "ds"}, false);
  controller.loadDataset();
  controller.cache_path = std::string(argv[4]) + "/";
  controller.initCamIntrinsics();
  controller.initCamExtrinsics();
  controller.initOptimization();
  { std::ofstream file(std::string(argv[4])+"/initial_calibration.json"); cereal::JSONOutputArchive archive(file); archive(*controller.calib_opt->calib); }
  nlohmann::json projections = nlohmann::json::array();
  for (const auto& [id, data] : controller.reprojected_vignette) {
    nlohmann::json points = nlohmann::json::array();
    for (const auto& p : data.corners_proj) points.push_back({p.x(), p.y()});
    projections.push_back({{"time", id.frame_id}, {"camera", id.cam_id}, {"points", points}, {"valid", data.corners_proj_success}});
  }
  std::ofstream(std::string(argv[4])+"/initial_vignette_projections.json") << projections.dump();
  while (!controller.optimizeWithParam(false)) {}
  controller.computeVign();
  controller.saveCalib();
}
