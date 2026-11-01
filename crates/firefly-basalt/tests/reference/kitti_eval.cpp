// SPDX-License-Identifier: BSD-3-Clause
#define main original_kitti_main
#include "/home/zhongjin/Projects/basalt/src/kitti_eval.cpp"
#undef main
#include <nlohmann/json.hpp>
int main(){
    Eigen::aligned_vector<Sophus::SE3d> gt,estimated;
    for(int i=0;i<101;++i){double x=i*1.25;gt.emplace_back(Sophus::SO3d::exp(Eigen::Vector3d(0,0,.001*i)),Eigen::Vector3d(x,.125*std::sin(.1*i),0));estimated.emplace_back(Sophus::SO3d::exp(Eigen::Vector3d(.0001*i,0,.0012*i)),Eigen::Vector3d(x*1.02,.125*std::sin(.1*i)+.001*i,.0001*i));}
    std::map<std::string,std::map<std::string,double>> result;
    std::ostringstream discarded;auto* old=std::cout.rdbuf(discarded.rdbuf());eval_kitti({5.,10.,25.,50.,200.},gt,estimated,result);std::cout.rdbuf(old);
    std::cout<<nlohmann::json(result).dump()<<'\n';
}
