// SPDX-License-Identifier: BSD-3-Clause
#define main original_mapper_main
#include "/home/zhongjin/Projects/basalt/src/mapper.cpp"
#undef main
#include <nlohmann/json.hpp>
using Json = nlohmann::json;
Json snapshot() {
  Json poses=Json::object(), points=Json::object();
  for(const auto& [time,state]:nrf_mapper->getFramePoses()) {
    Json matrix=Json::array(); const auto pose=state.getPose().matrix();
    for(int row=0;row<4;++row)for(int col=0;col<4;++col)matrix.push_back(pose(row,col));
    poses[std::to_string(time)]=matrix;
  }
  for(const auto& [id,point]:nrf_mapper->lmdb.getLandmarks())
    points[std::to_string(id)]={{"parameters",{point.direction.x(),point.direction.y(),point.inv_dist}},
      {"host",{point.host_kf_id.frame_id,point.host_kf_id.cam_id}},{"observations",point.obs.size()}};
  return {{"poses",poses},{"points",points}};
}
int frozen(const char* calibration,const char* cache,const char* config,const char* output,const char* input=nullptr) {
  vio_config.load(config);load_data(calibration,cache);
  for(auto& [time,data]:marg_data)nrf_mapper->addMargData(data);
  Json result;result["loaded"]=snapshot();
  if(input) {
    std::ifstream file(input);Json source;file>>source;
    for(const auto& frame:source["corners"]) {
      basalt::TimeCamId image{frame["image"][0].get<int64_t>(),frame["image"][1].get<size_t>()};
      auto& data=nrf_mapper->feature_corners[image];
      for(const auto& pixel:frame["pixels"])data.corners.emplace_back(pixel[0].get<double>(),pixel[1].get<double>());
    }
    for(const auto& entry:source["tracks"]) {
      auto& track=nrf_mapper->feature_tracks[entry["id"].get<int64_t>()];
      for(const auto& obs:entry["observations"])
        track[{obs[0].get<int64_t>(),obs[1].get<size_t>()}]=obs[2].get<int>();
    }
  } else {detect();match();nrf_mapper->build_tracks();}
  result["corners"]=Json::array();result["tracks"]=Json::array();
  for(const auto& [image,data]:nrf_mapper->feature_corners) {
    Json pixels=Json::array();for(const auto& pixel:data.corners)pixels.push_back({pixel.x(),pixel.y()});
    result["corners"].push_back({{"image",{image.frame_id,image.cam_id}},{"pixels",pixels}});
  }
  for(const auto& [id,track]:nrf_mapper->feature_tracks) {
    Json observations=Json::array();
    for(const auto& [image,feature]:track)observations.push_back({image.frame_id,image.cam_id,feature});
    result["tracks"].push_back({{"id",id},{"observations",observations}});
  }
  nrf_mapper->setup_opt();result["setup"]=snapshot();
  optimize();result["optimized"]=snapshot();
  filter();result["filtered"]=snapshot();
  optimize();result["final"]=snapshot();
  std::ofstream file(output);file<<result.dump(2)<<'\n';return 0;
}
int main(int argc, char** argv) {
  if(argc==7 && std::string(argv[1])=="--replay")
    return frozen(argv[2],argv[3],argv[4],argv[6],argv[5]);
  if(argc==6 && std::string(argv[1])=="--freeze")
    return frozen(argv[2],argv[3],argv[4],argv[5]);
  if (argc > 1) {
    // Run the unmodified original headless controller, then expose the same
    // trajectory-export callback used by its UI. No optimizer code is replaced.
    int result = original_mapper_main(argc, argv);
    if (result == 0) {
      tum_rgbd_fmt = true;
      saveTrajectoryButton();
    }
    return result;
  }
  nrf_mapper = std::make_shared<basalt::NfrMapper>(calib, vio_config);
  for (int i = 0; i < 3; ++i) {
    int64_t time = 1234567890123456LL + i * 123456789;
    Sophus::SE3d pose(Sophus::SO3d::exp(Eigen::Vector3d(.1*i,-.03*i,.04*i)), Eigen::Vector3d(.25*i, -.125*i, 1.5));
    nrf_mapper->getFramePoses()[time] = basalt::PoseStateWithLin<double>(time, pose, false);
    gt_frame_t_ns.push_back(time);
    gt_frame_t_w_i.push_back(pose.translation());
  }
  {
    std::ofstream output("gt.cereal", std::ios::binary);
    cereal::BinaryOutputArchive archive(output);
    archive(gt_frame_t_ns, gt_frame_t_w_i);
  }
  std::ostringstream discarded;
  auto old = std::cout.rdbuf(discarded.rdbuf());
  tum_rgbd_fmt = true; saveTrajectoryButton();
  tum_rgbd_fmt = false; saveTrajectoryButton();
  std::cout.rdbuf(old);
  auto read = [](const char* path) { std::ifstream file(path); return std::string(std::istreambuf_iterator<char>(file), {}); };
  std::ifstream binary("gt.cereal", std::ios::binary);
  std::vector<uint8_t> bytes(std::istreambuf_iterator<char>(binary), {});
  std::cout << nlohmann::json{{"tum",read("keyframeTrajectory.txt")},{"euroc",read("keyframeTrajectory.csv")},{"gt_binary",bytes}}.dump() << '\n';
}
