// Copyright (c) 2019 Vladyslav Usenko, Michael Loipführer and Nikolaus Demmel.
// SPDX-License-Identifier: BSD-3-Clause
// Hardware-free extraction of arithmetic from official device/rs_t265.cpp
// 0f3b2b52. This does NOT exercise librealsense callbacks or device lifetime.
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
int main(int argc,char**argv){if(argc!=2)return 1;nlohmann::json out;
for(double t:{10.,10.25,11.5,12.}){
double w0=(12.-t)/(12.-10.),w1=(t-10.)/(12.-10.);
Eigen::Vector3d value=w0*Eigen::Vector3d(1.,-2.,3.)+w1*Eigen::Vector3d(5.,6.,-1.);
out["interpolation"].push_back({{"t_ns",int64_t(t*1e6)},{"accel",{value.x(),value.y(),value.z()}}});}
for(auto input:{std::array<float,4>{.1f,-.2f,.3f,.9f},std::array<float,4>{2.f,3.f,4.f,5.f}}){
Eigen::Quaterniond q(input[3],input[0],input[1],input[2]);q.normalize();Sophus::SE3d pose(q,Eigen::Vector3d(1.,2.,3.));
out["quaternions"].push_back({pose.unit_quaternion().x(),pose.unit_quaternion().y(),pose.unit_quaternion().z(),pose.unit_quaternion().w()});}
float rotation[9]={0.f,1.f,0.f,-1.f,0.f,0.f,0.f,0.f,1.f};
Eigen::Matrix3f r=Eigen::Map<Eigen::Matrix3f>(rotation);Eigen::Quaterniond q(r.cast<double>());Sophus::SE3d pose(q,Eigen::Vector3d(.125,-.25,.5));
out["extrinsic_quaternion"]={pose.unit_quaternion().x(),pose.unit_quaternion().y(),pose.unit_quaternion().z(),pose.unit_quaternion().w()};
std::ofstream(argv[1])<<out.dump(2)<<'\n';}
