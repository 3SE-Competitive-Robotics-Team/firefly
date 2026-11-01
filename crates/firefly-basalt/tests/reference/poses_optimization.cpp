// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// Independent original Basalt calibration-linearization reference.
#include <basalt/optimization/poses_linearize.h>
#include <nlohmann/json.hpp>
#include <fstream>
int main(int argc, char** argv) {
  if(argc != 2) return 2;
  basalt::Calibration<double> calibration;
  for(const auto& name : {"pinhole", "ds"}) {
    auto camera=basalt::GenericCamera<double>::fromString(name);
    camera.setFromInit(Eigen::Vector4d(200,205,160,120));
    calibration.intrinsics.push_back(camera);
  }
  calibration.T_i_c={Sophus::SE3d(),Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(0.12,0,0))};
  Eigen::aligned_unordered_map<int64_t,Sophus::SE3d> poses;
  poses[10]=Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(0.01,0.02,0));
  poses[20]=Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(0.15,-0.02,0.03));
  Eigen::aligned_vector<Eigen::Vector4d> points;
  for(int i=0;i<12;i++) points.emplace_back((i%4-1.5)*0.3,(i/4-1)*0.25,3+0.02*i,1);
  std::unordered_map<int64_t,size_t> pose_offsets={{10,0},{20,6}};
  std::vector<size_t> extrinsics={12,18,24}, intrinsics={24,28,34};
  Eigen::aligned_vector<basalt::AprilgridCornersData> observations;
  for(int time:{10,20}) for(int camera=0;camera<2;camera++) {
    basalt::AprilgridCornersData obs;
    obs.timestamp_ns=time; obs.cam_id=camera;
    auto transform=(poses.at(time)*calibration.T_i_c[camera]).inverse().matrix();
    for(int i=0;i<12;i++) {
      Eigen::Vector2d pixel;
      std::visit([&](const auto& cam){ cam.project((transform*points[i]).eval(),pixel); },calibration.intrinsics[camera].variant);
      pixel += Eigen::Vector2d(i==0?4:0.15, -0.1);
      obs.corner_pos.push_back(pixel); obs.corner_id.push_back(i);
    }
    observations.push_back(obs);
  }
  nlohmann::json result=nlohmann::json::array();
  for(bool optimize:{false,true}) {
    basalt::LinearizeBase<double>::CalibCommonData common{};
    common.calibration=&calibration; common.aprilgrid_corner_pos_3d=&points;
    common.offset_T_i_c=&extrinsics;common.offset_intrinsics=&intrinsics;
    common.offset_poses=&pose_offsets;common.opt_intrinsics=optimize;common.huber_thresh=1;
    basalt::LinearizePosesOpt<double,basalt::DenseAccumulator<double>> lin(34,poses,common);
    lin(tbb::blocked_range<decltype(observations)::const_iterator>(observations.begin(),observations.end()));
    nlohmann::json h=nlohmann::json::array(),b=nlohmann::json::array();
    for(int r=0;r<34;r++){ for(int c=0;c<34;c++)h.push_back(lin.accum.getH()(r,c));b.push_back(lin.accum.getB()[r]); }
    result.push_back({{"opt_intrinsics",optimize},{"h",h},{"b",b},{"error",lin.error},{"reprojection_error",lin.reprojection_error},{"num_points",lin.num_points}});
  }
  std::ofstream(argv[1])<<result.dump(2)<<'\n';
}
