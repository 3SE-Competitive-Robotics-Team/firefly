// SPDX-License-Identifier: BSD-3-Clause
#include <basalt/serialization/headers_serialization.h>
#include <cereal/archives/json.hpp>
#include <iostream>
int main(){basalt::MocapCalibration<double>calibration;calibration.T_moc_w=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.125,-.25,.375)),Eigen::Vector3d(1,2,3));calibration.T_i_mark=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(-.25,.5,-.125)),Eigen::Vector3d(-1,.5,.25));calibration.mocap_time_offset_ns=-1234567890123456789LL;calibration.mocap_to_imu_offset_ns=1234567890123456789LL;cereal::JSONOutputArchive archive(std::cout);archive(calibration);}
