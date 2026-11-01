// SPDX-License-Identifier: BSD-3-Clause
// Official headers aa441ba3, Sophus d0b7315a0d90fc6143defa54596a3a95d9fa10ec.
#include <basalt/imu/imu_types.h>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class M>Json values(const M&m){Json out=Json::array();for(int i=0;i<m.size();++i)out.push_back(m[i]);return out;}
template<class S>Json run(){
  using V3=Eigen::Matrix<S,3,1>;
  Sophus::SO3<S> initial(Eigen::Quaternion<S>(S(.9),S(.2),S(-.3),S(.4)));
  Sophus::SO3<S> step=Sophus::SO3<S>::exp(V3(S(.001),S(.002),S(-.0005)));
  auto rotation=initial;
  basalt::PoseState<S> state(42,Sophus::SE3<S>(initial,V3::Zero()));
  Eigen::Matrix<S,6,1> increment=Eigen::Matrix<S,6,1>::Zero();increment[3]=S(.001);
  Json out={{"initial",values(initial.unit_quaternion().coeffs())},{"step",values(step.unit_quaternion().coeffs())},{"increment",values(increment)}};
  for(int i=1;i<=10000;++i){
    rotation=rotation*step;state.applyInc(increment);
    if(i==1||i==100||i==1000||i==10000)out["samples"].push_back({{"steps",i},{"product",values(rotation.unit_quaternion().coeffs())},{"state",values(state.T_w_i.so3().unit_quaternion().coeffs())},{"inverse",values(rotation.inverse().unit_quaternion().coeffs())}});
  }
  out["cast_f32"]=values(initial.template cast<float>().unit_quaternion().coeffs());
  out["cast_f64"]=values(initial.template cast<double>().unit_quaternion().coeffs());
  out["same_cast"]=values(initial.template cast<S>().unit_quaternion().coeffs());
  return out;
}
int main(){std::cout<<Json({{"f32",run<float>()},{"f64",run<double>()}}).dump(2)<<'\n';}
