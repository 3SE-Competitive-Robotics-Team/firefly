// SPDX-License-Identifier: BSD-3-Clause
// Official Sophus d0b7315a, Eigen 5.0.1; scalar-specific raw quaternion inputs.
#include <sophus/se3.hpp>
#include <basalt/utils/sophus_utils.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class S> Json run() {
  Json result=Json::array();
  for(double angle:{0.,1e-8,0.000125,0.00025,0.5,3.141592653589793}) {
    Sophus::SE3<S> pose(Sophus::SO3<S>::exp(Eigen::Matrix<S,3,1>(S(angle),S(-2*angle),S(3*angle))),Eigen::Matrix<S,3,1>(S(.125),S(-.25),S(.375)));
    auto quaternion=pose.unit_quaternion().coeffs();auto matrix=pose.matrix();
    Json q=Json::array(),m=Json::array();
    for(int i=0;i<4;++i)q.push_back(quaternion[i]);
    for(int r=0;r<4;++r)for(int c=0;c<4;++c)m.push_back(matrix(r,c));
    Eigen::Matrix<S,6,1> tangent;tangent<<S(.125),S(-.25),S(.375),S(angle/4),S(-angle/2),S(angle/3);
    Eigen::Matrix<S,6,6> jacobian,inverse;
    Sophus::rightJacobianSE3Decoupled(tangent,jacobian);
    Sophus::rightJacobianInvSE3Decoupled(tangent,inverse);
    Json phi=Json::array(),j=Json::array(),ji=Json::array();
    for(int i=0;i<6;++i)phi.push_back(tangent[i]);
    for(int r=0;r<6;++r)for(int c=0;c<6;++c){j.push_back(jacobian(r,c));ji.push_back(inverse(r,c));}
    result.push_back({{"quaternion",q},{"matrix",m},{"tangent",phi},{"jacobian",j},{"inverse_jacobian",ji}});
  }
  return result;
}
int main(){std::cout<<Json{{"f32",run<float>()},{"f64",run<double>()}}.dump()<<'\n';}
