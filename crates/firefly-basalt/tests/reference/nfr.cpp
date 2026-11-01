// SPDX-License-Identifier: BSD-3-Clause
// Independent oracle: unmodified Basalt 0f3b2b52 and its Eigen 5.0.1 dependency.
#include <basalt/utils/nfr.h>
#include <Eigen/QR>
#include <nlohmann/json.hpp>
#include <fstream>
using Json = nlohmann::json;
template<class T> Json rows(const T& value) {
  Json result = Json::array();
  for (int r=0;r<value.rows();++r) {
    Json row=Json::array();
    for (int c=0;c<value.cols();++c) row.push_back(value(r,c));
    result.push_back(row);
  }
  return result;
}
int main(int argc, char** argv) {
  if(argc!=2) return 1;
  Sophus::Vector6d va,vb,noise;
  va<<.2,-.7,.4,.3,-.2,.4; vb<<-.5,.1,.9,-.4,.2,-.1; noise.setConstant(.003);
  auto a=Sophus::se3_expd(va), b=Sophus::se3_expd(vb);
  auto measured=Sophus::se3_expd(noise)*a.inverse()*b;
  Sophus::Matrix6d ja,jb;
  auto residual=basalt::relPoseError(measured,a,b,&ja,&jb);
  Json out={{"residual",rows(residual)},{"ja",rows(ja)},{"jb",rows(jb)}};
  Eigen::Matrix<double,3,6> jp; Eigen::Matrix<double,1,6> jy; Eigen::Matrix<double,2,6> jr;
  out["position"]=rows(basalt::absPositionError(a,b.translation(),&jp));
  out["yaw"]=basalt::yawError(a,Eigen::Vector3d::UnitX(),&jy);
  out["roll_pitch"]=rows(basalt::rollPitchError(a,b.so3(),&jr));
  out["position_j"]=rows(jp); out["yaw_j"]=rows(jy); out["roll_pitch_j"]=rows(jr);
  out["qr"]=Json::array();
  for(int test=0;test<8;++test) {
    Eigen::MatrixXd m(5,5);
    for(int c=0;c<5;++c)for(int r=0;r<5;++r)m(r,c)=std::sin(double(r*5+c+test));
    if(test==0)m.setZero();
    else if(test==1)m.setIdentity();
    else if(test==2)m.col(4)=m.col(0)+m.col(2);
    else m.diagonal().array()+=std::pow(10.,-test*2);
    Eigen::FullPivHouseholderQR<Eigen::MatrixXd> qr(m);
    Eigen::MatrixXd rhs=m*Eigen::MatrixXd::Ones(5,2);
    Eigen::MatrixXd solved=qr.solve(rhs);
    out["qr"].push_back({{"matrix",rows(m)},{"rank",qr.rank()},{"rhs",rows(rhs)},{"solution",rows(solved)}});
  }
  std::ofstream(argv[1])<<out.dump(2)<<'\n';
}
