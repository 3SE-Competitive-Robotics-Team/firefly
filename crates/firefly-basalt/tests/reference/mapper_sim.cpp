// SPDX-License-Identifier: BSD-3-Clause
// Executes the original translation unit's standalone callbacks.
#define main original_mapper_sim_main
#include "/home/zhongjin/Projects/basalt/src/mapper_sim.cpp"
#undef main
#include <nlohmann/json.hpp>
using Json=nlohmann::json;
template<class M>Json rows(const M& m){Json result=Json::array();for(int r=0;r<m.rows();++r){Json row=Json::array();for(int c=0;c<m.cols();++c)row.push_back(m(r,c));result.push_back(row);}return result;}
int main(int argc,char**argv){if(argc!=2)return 1;Json out=Json::array();
 for(int test=0;test<3;++test){basalt::MargData data;int n=test==0?12:test==1?21:36;Eigen::MatrixXd a(n,n);for(int r=0;r<n;++r)for(int c=0;c<n;++c)a(r,c)=std::sin(double(1+r*3+c*7))*.1;data.abs_H=a.transpose()*a+Eigen::MatrixXd::Identity(n,n)*2;data.abs_b=Eigen::VectorXd::LinSpaced(n,-.2,.3);data.aom.total_size=n;
 for(int i=0;i<(test==2?3:2);++i){int64_t time=10+i*10;Sophus::Vector6d tangent;tangent<<.2*i,-.1*i,.3,.03*i,-.02,.04;auto pose=Sophus::se3_expd(tangent);int size=i==0||test==0?6:15;int offset=i==0?0:i==1?6:21;data.aom.abs_order_map[time]={offset,size};if(size==6)data.frame_poses.emplace(time,basalt::PoseStateWithLin<double>(time,pose));else{basalt::PoseVelBiasState<double> state(time,pose,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());data.frame_states.emplace(time,basalt::PoseVelBiasStateWithLin<double>(state));}}
 data.kfs_all={10,20};data.kfs_to_marg={10};Json sample={{"h",rows(data.abs_H)},{"b",rows(data.abs_b)}};if(test>0)processMargData(data);sample["reduced_h"]=rows(data.abs_H);sample["reduced_b"]=rows(data.abs_b);roll_pitch_factors.clear();rel_pose_factors.clear();extractNonlinearFactors(data);sample["roll_information"]=rows(roll_pitch_factors[0].cov_inv);sample["relative_information"]=rows(rel_pose_factors[0].cov_inv);sample["relative_pose"]=rows(rel_pose_factors[0].T_i_j.matrix());out.push_back(sample);
 }std::srand(1);nrf_mapper=std::make_shared<basalt::NfrMapper>(calib,basalt::VioConfig());nrf_mapper->getFramePoses().emplace(0,basalt::PoseStateWithLin<double>(0,Sophus::SE3d()));randomInc();randomYawInc();out[0]["random_pose"]=rows(nrf_mapper->getFramePoses().at(0).getPose().matrix());std::ofstream(argv[1])<<out.dump(2)<<'\n';}
