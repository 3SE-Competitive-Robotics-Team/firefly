// SPDX-License-Identifier: BSD-3-Clause
// Calls unmodified original Basalt 0f3b2b52 utils/ba_utils.h.
#include <basalt/utils/ba_utils.h>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class M>Json matrix(const M&m){Json a=Json::array();for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c)a.push_back(m(r,c));return {{"rows",m.rows()},{"cols",m.cols()},{"values",a}};}
template<class S>Json run(){using V3=Eigen::Matrix<S,3,1>;using M6=Eigen::Matrix<S,6,6>;Json out;std::vector<Sophus::SE3<S>> poses;for(int i=0;i<4;++i){S v=S(i+1);poses.emplace_back(Sophus::SO3<S>::exp(V3(S(.03)*v,S(-.02)*v,S(.01)*v)),V3(S(.2)*v,S(-.125)*v,S(.05)*v));}
  M6 dh,dt;auto relative=basalt::computeRelPose(poses[0],poses[1],poses[2],poses[3],&dh,&dt);out["transform"]=matrix(relative.matrix());out["host_jacobian"]=matrix(dh);out["target_jacobian"]=matrix(dt);
  for(auto&pose:poses)out["poses"].push_back({{"quaternion",matrix(pose.unit_quaternion().coeffs())},{"translation",matrix(pose.translation())}});
  for(std::string name:{"pinhole","ds","kb4","ucm","eucm","pinhole-radtan8"}){auto camera=basalt::GenericCamera<S>::fromString(name);camera.setFromInit((Eigen::Matrix<S,4,1>()<<S(320),S(315),S(320),S(240)).finished());for(int mode=0;mode<3;++mode){basalt::Keypoint<S>point;point.direction<<S(.15),S(-.2);point.inv_dist=mode==1?S(0):S(.4);if(mode==2)point.direction<<S(4),S(3);Eigen::Matrix<S,2,1>obs(S(300),S(200)),res=Eigen::Matrix<S,2,1>::Constant(S(6));Eigen::Matrix<S,2,6>jx=Eigen::Matrix<S,2,6>::Constant(S(7));Eigen::Matrix<S,2,3>jp=Eigen::Matrix<S,2,3>::Constant(S(8));Eigen::Matrix<S,4,1>projection=Eigen::Matrix<S,4,1>::Constant(S(9));Eigen::Matrix<S,4,4>transform=relative.matrix();bool valid=std::visit([&](const auto&cam){return basalt::linearizePoint(obs,point,transform,cam,res,&jx,&jp,&projection);},camera.variant);out["points"].push_back({{"camera",name},{"direction",matrix(point.direction)},{"inverse_distance",point.inv_dist},{"observation",matrix(obs)},{"valid",valid},{"residual",matrix(res)},{"pose_jacobian",matrix(jx)},{"point_jacobian",matrix(jp)},{"projection",matrix(projection)}});}}
  return out;
}
int main(int argc,char**argv){if(argc!=2)return 2;std::string scalar=argv[1];if(scalar=="f32")std::cout<<run<float>().dump(2);else if(scalar=="f64")std::cout<<run<double>().dump(2);else return 2;std::cout<<'\n';}
