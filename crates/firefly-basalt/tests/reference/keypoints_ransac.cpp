// SPDX-License-Identifier: BSD-3-Clause
// Runs the complete official Basalt keypoints implementation without editing it.
#include <src/utils/keypoints.cpp>
#include <nlohmann/json.hpp>
#include <iostream>

int main() {
  basalt::KeypointsData a,b;
  basalt::MatchData matches;
  const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(0.12,Eigen::Vector3d::UnitZ())*
    Eigen::AngleAxisd(-0.08,Eigen::Vector3d::UnitY())*
    Eigen::AngleAxisd(0.04,Eigen::Vector3d::UnitX())).matrix();
  const Eigen::Vector3d translation(0.3,-0.2,0.1);
  for(int i=0;i<48;++i) {
    Eigen::Vector3d p((i%6)*0.6-1.4,(i/6)*0.4-1.2,4.0+(i%7)*0.35);
    Eigen::Vector4d x,y;
    x << p.normalized(),0;
    y << (rotation.transpose()*(p-translation)).normalized(),0;
    a.corners_3d.push_back(x);b.corners_3d.push_back(y);matches.matches.emplace_back(i,i);
  }
  basalt::findInliersRansac(a,b,1e-8,30,matches);
  auto m=matches.T_i_j.matrix();
  nlohmann::json matrix=nlohmann::json::array();
  for(int r=0;r<4;++r)for(int c=0;c<4;++c)matrix.push_back(m(r,c));
  std::cout<<nlohmann::json({{"matrix",matrix},{"inliers",matches.inliers}}).dump(2)<<"\n";
}
