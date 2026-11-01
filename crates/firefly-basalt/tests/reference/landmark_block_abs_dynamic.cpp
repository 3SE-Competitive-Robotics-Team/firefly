// SPDX-License-Identifier: BSD-3-Clause
// Original Basalt 0f3b2b52; access modifiers changed only to inspect numeric storage.
#include <basalt/utils/ba_utils.h>
#include <basalt/linearization/landmark_block.hpp>
#include <fstream>
#include <mutex>
#define private public
#include <basalt/linearization/landmark_block_abs_dynamic.hpp>
#undef private
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class M>Json matrix(const M&m){Json a=Json::array();for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c){auto v=m(r,c);if(std::isnan(v))a.push_back("NaN");else if(std::isinf(v))a.push_back(v>0?"Infinity":"-Infinity");else a.push_back(v);}return {{"rows",m.rows()},{"cols",m.cols()},{"values",a}};}
template<class S>Json run(){using M4=Eigen::Matrix<S,4,4>;using V=Eigen::Matrix<S,Eigen::Dynamic,1>;Json out;for(bool householder:{true,false})for(std::string mode:{"normal","huber","infinity","degenerate","dropped"}){
  basalt::Calibration<S>calibration;auto cam=basalt::GenericCamera<S>::fromString("pinhole");cam.setFromInit((Eigen::Matrix<S,4,1>()<<S(320),S(315),S(320),S(240)).finished());calibration.intrinsics={cam,cam};basalt::Keypoint<S>point;point.direction<<S(.15),S(-.2);point.inv_dist=mode=="infinity"?S(0):S(.4);point.host_kf_id=basalt::TimeCamId(10,0);
  basalt::AbsOrderMap order;order.abs_order_map[10]={0,6};order.abs_order_map[20]={6,6};order.abs_order_map[30]={12,6};order.total_size=18;order.items=3;
  Eigen::aligned_unordered_map<std::pair<basalt::TimeCamId,basalt::TimeCamId>,basalt::RelPoseLin<S>> relative;
  Json observations=Json::array();for(int i=0;i<4;++i){basalt::TimeCamId target(i==3?40:(i==0?10:i==1?20:30),i%2);if(i==3&&mode!="dropped")continue;Eigen::Matrix<S,2,1>pixel(S(380+i*2),S(120-i*3));point.obs[target]=pixel;basalt::RelPoseLin<S>rel;rel.T_t_h=M4::Identity();if(mode!="degenerate"){rel.T_t_h(0,3)=S(.125)*S(i);rel.T_t_h(1,3)=S(-.0625)*S(i);}rel.d_rel_d_h.setIdentity();rel.d_rel_d_t=-Eigen::Matrix<S,6,6>::Identity();relative[{point.host_kf_id,target}]=rel;observations.push_back({{"frame",target.frame_id},{"camera",target.cam_id},{"pixel",matrix(pixel)},{"transform",matrix(rel.T_t_h)}});}
  typename basalt::LandmarkBlock<S>::Options options;options.use_householder=householder;options.huber_parameter=mode=="huber"?S(2):S(0);options.obs_std_dev=S(1.5);basalt::LandmarkBlockAbsDynamic<S,6>block;block.allocateLandmark(point,relative,calibration,order,options);
  Json item={{"householder",householder},{"mode",mode},{"observations",observations},{"inverse_distance",point.inv_dist}};item["error"]=block.linearizeLandmark();item["linearized"]=matrix(block.storage);V diagonal=V::Zero(18);if(mode!="dropped"){block.addJp_diag2(diagonal);item["pose_diagonal"]=matrix(diagonal);}block.scaleJl_cols();item["scaled"]=matrix(block.storage);item["landmark_scaling"]=matrix(block.Jl_col_scale);block.performQR();item["qr"]=matrix(block.storage);V scaling=V::Ones(18);for(int i=0;i<18;++i)scaling[i]=S(1)+S(i)*S(.015625);block.scaleJp_cols(scaling);item["pose_scaled"]=matrix(block.storage);
  for(S damping:{S(.25),S(2),S(0),S(.125)}){block.setLandmarkDamping(damping);item["damped"].push_back(matrix(block.storage));}
  Eigen::Matrix<S,Eigen::Dynamic,Eigen::Dynamic>h=Eigen::Matrix<S,Eigen::Dynamic,Eigen::Dynamic>::Zero(18,18);V b=V::Zero(18);static_cast<basalt::LandmarkBlock<S>&>(block).add_dense_H_b(h,b);item["h"]=matrix(h);item["b"]=matrix(b);V increment(18);for(int i=0;i<18;++i)increment[i]=S(i-8)*S(.000125);S decrease=S(.125);block.backSubstitute(increment,decrease);item["model_decrease"]=decrease;item["direction"]=matrix(point.direction);item["updated_inverse_distance"]=point.inv_dist;item["undamped"]=matrix(block.storage);out.push_back(item);
}return out;}
int main(int argc,char**argv){if(argc!=2)return 2;std::string scalar=argv[1];if(scalar=="f32")std::cout<<run<float>().dump(2);else if(scalar=="f64")std::cout<<run<double>().dump(2);else return 2;std::cout<<'\n';}
