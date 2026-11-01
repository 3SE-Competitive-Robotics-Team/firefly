// SPDX-License-Identifier: BSD-3-Clause
#define main original_time_alignment_main
#include "/home/zhongjin/Projects/basalt/src/time_alignment.cpp"
#undef main
#include <nlohmann/json.hpp>
using Json=nlohmann::json;
int main(){
    std::vector<int64_t> gt,mt;Eigen::aligned_vector<Eigen::Vector3d> gv,mv;
    for(int i=0;i<100;++i){gt.push_back(i*1000000);gv.emplace_back(std::sin(.17*i),std::cos(.07*i),.01*i);}
    for(int i=5;i<95;++i){mt.push_back(i*1000000-1234567);mv.push_back(gv[i]);}
    Json errors=Json::array();double best=std::numeric_limits<double>::max();int best_index=-1,best_offset=0;std::vector<double> values;
    for(int64_t offset=-4000000;offset<=4000000;offset+=100000){double e=compute_error(offset,gt,gv,mt,mv);errors.push_back(e);values.push_back(e);if(e<best){best=e;best_index=values.size()-1;best_offset=offset;}}
    Eigen::MatrixXd pol(21,3);Eigen::VectorXd rhs(21);for(int i=0;i<21;++i){int idx=i-10;pol.row(i)<<idx*idx,idx,1;rhs[i]=values[best_index+idx];}
    Eigen::Vector3d coeff=pol.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(rhs);int refined=best_offset;if(coeff[0]>1e-9)refined-=100000*coeff[1]/(2*coeff[0]);
    Json boundary=Json::array();for(int64_t offset:{-100000000LL,0LL,100000000LL})boundary.push_back(compute_error(offset,gt,gv,mt,mv));
    std::cout<<Json{{"errors",errors},{"best_index",best_index},{"best_offset",best_offset},{"refined",refined},{"coefficients",{coeff[0],coeff[1],coeff[2]}},{"boundary",boundary}}.dump()<<'\n';
}
