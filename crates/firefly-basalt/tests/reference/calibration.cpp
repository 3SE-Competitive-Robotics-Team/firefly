// SPDX-License-Identifier: BSD-3-Clause
// Original Basalt AprilGrid and VignetteEstimator oracle.
#include <basalt/calibration/vignette.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cstdio>
using Json=nlohmann::json;
struct Dataset : basalt::VioDataset {
  std::vector<int64_t> times;
  Eigen::aligned_vector<basalt::AccelData> accelerations;
  Eigen::aligned_vector<basalt::GyroData> gyros;
  Eigen::aligned_vector<Sophus::SE3d> poses;
  size_t get_num_cams() const override {return 2;}
  std::vector<int64_t>& get_image_timestamps() override {return times;}
  const Eigen::aligned_vector<basalt::AccelData>& get_accel_data() const override{return accelerations;}
  const Eigen::aligned_vector<basalt::GyroData>& get_gyro_data() const override{return gyros;}
  const std::vector<int64_t>& get_gt_timestamps() const override{return times;}
  const Eigen::aligned_vector<Sophus::SE3d>& get_gt_pose_data() const override{return poses;}
  int64_t get_mocap_to_imu_offset_ns() const override{return 0;}
  std::vector<basalt::ImageData> get_image_data(int64_t) override{return {};}
};
template<class T> Json vector(const T& x){Json out=Json::array();for(int i=0;i<x.size();++i)out.push_back(x[i]);return out;}
int main(int argc,char**argv) {
  if(argc!=3)return 1;
  { std::ofstream stream(argv[2]);stream<<R"({"tagCols":2,"tagRows":1,"tagSize":0.04,"tagSpacing":0.25})"; }
  basalt::AprilGrid grid(argv[2]); std::remove(argv[2]);
  Json output;
  for(auto&x:grid.aprilgrid_corner_pos_3d)output["corners"].push_back(vector(x));
  for(auto&x:grid.aprilgrid_vignette_pos_3d)output["vignette"].push_back(vector(x));
  Eigen::aligned_vector<Eigen::Vector2d> centers{{20,15},{20,15}};
  Eigen::aligned_vector<Eigen::Vector2i> resolutions{{40,30},{40,30}};
  std::map<basalt::TimeCamId,Eigen::aligned_vector<Eigen::Vector3d>> data;
  for(int frame=0;frame<3;++frame)for(int camera=0;camera<2;++camera){
    auto& p=data[{frame,camera}];
    for(int i=0;i<grid.aprilgrid_vignette_pos_3d.size();++i) {
      double radius=2+(i+frame*3+camera)%17;
      p.emplace_back(20+radius*.6,15+radius*.8,(i%9==0)?-1:100+i*2-frame*3-camera);
    }
  }
  auto dataset=std::make_shared<Dataset>();
  basalt::VignetteEstimator estimator(dataset,centers,resolutions,data,grid);
  for(int stage=0;stage<4;++stage){
    if(stage==1)estimator.opt_irradience();
    if(stage==2)estimator.opt_vign();
    if(stage==3)estimator.optimize();
    Json state;
    for(auto&s:estimator.get_vign_param()) {Json knots=Json::array();for(auto&v:s.getKnots())knots.push_back(v[0]);state["knots"].push_back(knots);}
    std::map<basalt::TimeCamId,std::vector<double>> errors;estimator.compute_error(&errors);
    for(auto&[key,values]:errors)state["errors"].push_back({key.frame_id,key.cam_id,values});
    std::vector<std::vector<float>> log;estimator.compute_data_log(log);state["data_log"]=log;
    output["stages"].push_back(state);
  }
  std::ofstream(argv[1])<<output.dump(2)<<'\n';
}
