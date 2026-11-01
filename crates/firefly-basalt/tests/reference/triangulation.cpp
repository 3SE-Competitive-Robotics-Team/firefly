// SPDX-License-Identifier: BSD-3-Clause
// Executes official BaBase::triangulate and Eigen 5 JacobiSVD.
#include <basalt/vi_estimator/ba_base.h>
#include <nlohmann/json.hpp>
#include <fstream>
template<class V> nlohmann::json values(const V& v) {
  nlohmann::json out=nlohmann::json::array();
  for(int i=0;i<v.size();++i){if(std::isnan(v[i]))out.push_back("NaN");else if(std::isinf(v[i]))out.push_back(v[i]>0?"+Inf":"-Inf");else out.push_back(v[i]);}return out;
}
template<class S> nlohmann::json run() {
  using Vec=Eigen::Matrix<S,3,1>;nlohmann::json out=nlohmann::json::array();
  for(int i=0;i<10;++i){
    Vec first(S(.4),S(-.3),S(4)),second(S(.2),S(-.3),S(4));
    Sophus::SE3<S> pose(Sophus::SO3<S>(),Vec(S(.2),0,0));
    if(i==1){pose=Sophus::SE3<S>(Sophus::SO3<S>::exp(Vec(S(.1),S(-.2),S(.3))),Vec(S(.3),S(-.1),S(.2)));second=pose.inverse()*first;}
    if(i==2){second+=Vec(S(.001),S(-.002),0);}
    if(i==3){pose=Sophus::SE3<S>();second=first;}
    if(i==4){pose.translation().x()=S(1e-8);second=first-pose.translation();}
    if(i==5){first=-first;second=-second;}
    if(i==6){first=Vec(1,0,0);second=first;}
    if(i==7){first.setZero();second.setZero();}
    if(i==8){first=Vec(0,0,1);second=first;}
    if(i==9){pose.translation()=Vec(S(1e5),S(-2e5),S(3e5));first=Vec(S(2e5),S(3e5),S(4e5));second=first-pose.translation();}
    if(first.norm()>0)first.normalize();if(second.norm()>0)second.normalize();
    auto result=basalt::BundleAdjustmentBase<S>::triangulate(first,second,pose);
    out.push_back({{"first",values(first)},{"second",values(second)},{"translation",values(pose.translation())},{"quaternion",values(pose.unit_quaternion().coeffs())},{"result",values(result)}});
  }return out;
}
int main(int argc,char**argv){if(argc!=2)return 1;std::ofstream out(argv[1]);out<<nlohmann::json{{"f32",run<float>()},{"f64",run<double>()}}.dump(2)<<'\n';}
