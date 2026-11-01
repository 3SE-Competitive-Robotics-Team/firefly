// Original Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc.
#include <basalt/vi_estimator/landmark_database.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <map>

nlohmann::json snapshot(const basalt::LandmarkDatabase<double>& db) {
  nlohmann::json result={{"num_observations",db.numObservations()}};
  std::map<size_t,nlohmann::json> landmarks;
  for(const auto& [id,landmark]:db.getLandmarks()) {
    nlohmann::json observations=nlohmann::json::array();
    for(const auto& [target,pixel]:landmark.obs)
      observations.push_back({target.frame_id,target.cam_id,pixel.x(),pixel.y()});
    landmarks[id]={{"id",id},{"host",{landmark.host_kf_id.frame_id,landmark.host_kf_id.cam_id}},
      {"direction",{landmark.direction.x(),landmark.direction.y()}},{"inv_dist",landmark.inv_dist},{"observations",observations}};
  }
  result["landmarks"]=nlohmann::json::array();
  for(const auto& [id,landmark]:landmarks) result["landmarks"].push_back(landmark);
  auto hosts=db.getHostKfs();std::sort(hosts.begin(),hosts.end());
  result["hosts"]=nlohmann::json::array();
  for(const auto& host:hosts) result["hosts"].push_back({host.frame_id,host.cam_id,db.getLandmarksForHost(host).size()});
  return result;
}
int main() {
  basalt::LandmarkDatabase<double> db;
  nlohmann::json result=nlohmann::json::array();
  for(int id=1;id<=3;++id) {
    basalt::Keypoint<double> point;
    point.direction=Eigen::Vector2d(id*0.25,-id*0.5);
    point.inv_dist=id*0.125;
    point.host_kf_id=basalt::TimeCamId(id==3?2:1,id==2?1:0);
    db.addLandmark(id,point);
    for(int frame=1;frame<=4;++frame) db.addObservation(basalt::TimeCamId(frame,0),{id,Eigen::Vector2d(id*10+frame,id*20-frame)});
  }
  result.push_back(snapshot(db));
  db.backup(); db.getLandmark(1).direction=Eigen::Vector2d(100,200);db.getLandmark(1).inv_dist=99;
  result.push_back(snapshot(db));db.restore();result.push_back(snapshot(db));
  db.addObservation(basalt::TimeCamId(1,0),{2,Eigen::Vector2d(7,8)});
  result.push_back(snapshot(db));
  db.removeFrame(4);result.push_back(snapshot(db));
  db.removeObservations(1,{basalt::TimeCamId(3,0)});result.push_back(snapshot(db));
  db.removeKeyframes({2},{},{});result.push_back(snapshot(db));
  db.removeLandmark(2);result.push_back(snapshot(db));
  std::cout<<result.dump(2)<<'\n';
}
