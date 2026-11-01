// SPDX-License-Identifier: BSD-3-Clause
#include <basalt/calibration/calibration.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class Derived>Json values(const Eigen::MatrixBase<Derived>&matrix){
  Json out=Json::array();for(int r=0;r<matrix.rows();++r)for(int c=0;c<matrix.cols();++c)out.push_back(matrix(r,c));return out;
}
template<class S>Json run(){
  basalt::Calibration<S> calibration;
  for(int i=0;i<9;++i)calibration.calib_accel_bias.getParam()[i]=S(0.001*(i-4));
  for(int i=0;i<12;++i)calibration.calib_gyro_bias.getParam()[i]=S(0.002*(i-5));
  Eigen::Matrix<S,3,1> raw(S(1.2),S(-.3),S(9.81)),ab,gb;
  Eigen::Matrix<S,3,3> as,gs;
  calibration.calib_accel_bias.getBiasAndScale(ab,as);
  calibration.calib_gyro_bias.getBiasAndScale(gb,gs);
  auto accel=calibration.calib_accel_bias.getCalibrated(raw);
  auto gyro=calibration.calib_gyro_bias.getCalibrated(raw);
  Json result={{"accel_bias",values(ab)},{"accel_scale",values(as)},{"gyro_bias",values(gb)},{"gyro_scale",values(gs)},
    {"accel",values(accel)},{"gyro",values(gyro)},
    {"gyro_discrete",values(calibration.dicrete_time_gyro_noise_std())},{"accel_discrete",values(calibration.dicrete_time_accel_noise_std())}};
  if constexpr(std::is_same_v<S,double>){
    result["inverse_accel"]=values(calibration.calib_accel_bias.invertCalibration(accel));
    result["inverse_gyro"]=values(calibration.calib_gyro_bias.invertCalibration(gyro));
    calibration.T_i_c.emplace_back(Sophus::SO3d::exp(Eigen::Vector3d(.2,-.1,.3)),Eigen::Vector3d(.1,.2,.3));
    auto cast=calibration.template cast<float>();
    result["cast_pose"]=values(cast.T_i_c[0].matrix());
    result["cast_accel_bias"]=values(cast.calib_accel_bias.getParam());
  }
  return result;
}
int main(){std::cout<<Json({{"f64",run<double>()},{"f32",run<float>()}}).dump(2)<<"\n";}
