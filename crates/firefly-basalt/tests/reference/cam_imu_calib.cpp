// Copyright (c) 2019 Vladyslav Usenko and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// Angular-velocity least squares and AngleAxis projection from cam_imu_calib.cpp.
#include <Eigen/Dense>
#include <sophus/so3.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
int main() {
  nlohmann::json cases = nlohmann::json::array();
  for (int scenario = 0; scenario < 4; ++scenario) {
    const Sophus::SO3d initial = Sophus::SO3d::exp(Eigen::Vector3d(.2,-.1,.05));
    const Sophus::SO3d expected = Sophus::SO3d::exp(Eigen::Vector3d(.3+scenario,-.2,.1));
    Eigen::Matrix<double,3,12> visual, imu;
    for (int i=0; i<12; ++i) {
      Eigen::Vector3d velocity(.3+std::sin(i*.7), -.2+std::cos(i*.3), .1+i*.09);
      const auto a = Sophus::SO3d::exp(Eigen::Vector3d(.01*i,-.02*i,.005*i));
      const auto b = a * Sophus::SO3d::exp(velocity*.01);
      visual.col(i)=initial*(a.inverse()*b).log()/.01;
      imu.col(i)=expected*visual.col(i)+Eigen::Vector3d(.001*i,-.002*i,.003*i);
    }
    Eigen::Matrix3d raw=imu*visual.transpose()*(visual*visual.transpose()).inverse();
    Eigen::AngleAxisd aa(raw);
    const Eigen::Matrix3d rotation=aa.toRotationMatrix();
    nlohmann::json item; item["visual"]=nlohmann::json::array(); item["imu"]=nlohmann::json::array(); item["rotation"]=nlohmann::json::array();
    for(int i=0;i<12;++i) { for(int j=0;j<3;++j) {item["visual"].push_back(visual(j,i));item["imu"].push_back(imu(j,i));} }
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)item["rotation"].push_back(rotation(r,c));
    cases.push_back(item);
  }
  std::cout << cases.dump() << '\n';
}
