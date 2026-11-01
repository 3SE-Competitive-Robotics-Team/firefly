// SPDX-License-Identifier: BSD-3-Clause
// Independent original Eigen/Sophus Cereal serialization oracle.
#include <basalt/serialization/eigen_io.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
using Json=nlohmann::json;
template<class T>Json archive(T value){std::ostringstream binary,text;{cereal::BinaryOutputArchive out(binary);out(value);}{cereal::JSONOutputArchive out(text);out(value);}std::ostringstream hex;for(unsigned char c:binary.str())hex<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c);Json result={{"binary",hex.str()},{"json",Json::parse(text.str())["value0"]}};try{T copy;std::istringstream input(text.str());cereal::JSONInputArchive in(input);in(copy);result["json_load_ok"]=true;}catch(const std::exception&e){result["json_load_ok"]=false;result["json_load_error"]=e.what();}return result;}
int main(int argc,char**argv){if(argc!=2)return 1;Json output;
Eigen::Matrix<double,2,3> fixed;fixed<<1.,-2.5,3.25,4.,5.5,-6.;output["fixed_f64"]=archive(fixed);
Eigen::Matrix<float,2,3> single=fixed.cast<float>();output["fixed_f32"]=archive(single);
Eigen::Matrix<int32_t,2,2> integers;integers<<-1,2,0,2147483647;output["fixed_i32"]=archive(integers);
Eigen::Matrix<double,Eigen::Dynamic,3> rows=fixed;output["dynamic_rows"]=archive(rows);
Eigen::Matrix<double,2,Eigen::Dynamic> cols=fixed;output["dynamic_cols"]=archive(cols);
Eigen::MatrixXd dynamic=fixed;output["dynamic"]=archive(dynamic);
Eigen::MatrixXd empty(0,3);output["empty_dynamic"]=archive(empty);
Sophus::SE3d pose(Eigen::Quaterniond(.5,.5,.5,.5),Eigen::Vector3d(1.5,-2.,.125));output["se3"]=archive(pose);
Sophus::Sim3d similarity(Sophus::RxSO3d(4.,Eigen::Quaterniond(.5,.5,.5,.5)),Eigen::Vector3d(-1.,2.,3.));output["sim3"]=archive(similarity);
pose.so3().data()[0]=2.;output["raw_se3"]=archive(pose);
similarity.rxso3().data()[0]=0.;similarity.rxso3().data()[1]=0.;similarity.rxso3().data()[2]=0.;similarity.rxso3().data()[3]=0.;output["raw_sim3"]=archive(similarity);
std::ofstream(argv[1])<<output.dump(2)<<'\n';}
