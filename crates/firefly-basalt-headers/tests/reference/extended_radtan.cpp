// SPDX-License-Identifier: BSD-3-Clause
// Official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/camera/extended_camera.hpp>
#include <basalt/camera/pinhole_radtan8_camera.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <cmath>
using Json=nlohmann::json;
template<class M> Json values(const M& m) {
  Json result=Json::array();
  for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c){
    double v=m(r,c);
    if(std::isnan(v))result.push_back("NaN");
    else if(std::isinf(v))result.push_back(v>0?"+Inf":"-Inf");
    else result.push_back(v);
  }
  return result;
}
template<class Camera> Json run(){
  using S=typename Camera::Scalar;
  constexpr int N=Camera::N;
  auto cameras=Camera::getTestProjections();
  if constexpr(N==6){
    for(S alpha:{S(0),S(.5),S(.75),S(1)}){
      typename Camera::VecN p;p<<S(200),S(220),S(100),S(90),alpha,S(1.2);cameras.emplace_back(p);
    }
  }else{
    auto p=cameras[0].getParam();cameras.emplace_back(p,S(.5));
    p.setZero();p[0]=S(200);p[1]=S(220);p[2]=S(100);p[3]=S(90);cameras.emplace_back(p);
  }
  Json out=Json::array();
  for(auto camera:cameras){
    Json record={{"param",values(camera.getParam())}};
    if constexpr(N==12){record["rpmax"]=camera.getRpmax();record["computed_rpmax"]=camera.computeRpmax();}
    for(Eigen::Matrix<S,4,1> p:{Eigen::Matrix<S,4,1>(S(.25),S(-.75),S(2),S(1)),{S(.25),S(-.75),S(-1),S(.7)},
        {S(0),S(0),S(1),S(0)},{S(0),S(0),S(-1),S(0)},{S(0),S(0),S(0),S(0)}}){
      Eigen::Matrix<S,2,1> pixel;Eigen::Matrix<S,2,4> jp;Eigen::Matrix<S,2,N> ji;
      bool valid=camera.project(p,pixel,&jp,&ji);
      record["project"].push_back({{"point",values(p)},{"pixel",values(pixel)},{"valid",valid},{"j_point",values(jp)},{"j_param",values(ji)}});
    }
    for(Eigen::Matrix<S,2,1> pixel:{Eigen::Matrix<S,2,1>(camera.getParam()[2],camera.getParam()[3]),{S(150),S(120)},{S(-50),S(300)},{S(2000),S(1000)}}){
      Eigen::Matrix<S,4,1> ray;Eigen::Matrix<S,4,2> jp;Eigen::Matrix<S,4,N> ji;
      bool valid;
      if constexpr(N==6)valid=camera.unproject(pixel,ray,&jp,&ji);
      else valid=camera.unproject(pixel,ray);
      Json row={{"pixel",values(pixel)},{"ray",values(ray)},{"valid",valid}};
      if constexpr(N==6){row["j_pixel"]=values(jp);row["j_param"]=values(ji);}
      record["unproject"].push_back(row);
    }
    if constexpr(N==12){
      for(Eigen::Matrix<S,2,1> xy:{Eigen::Matrix<S,2,1>(S(.2),S(-.4)),{S(0),S(0)},{S(3),S(5)}}){
        Eigen::Matrix<S,2,1> dist;Eigen::Matrix<S,2,2> j;camera.distort(xy,dist,&j);
        record["distort"].push_back({{"xy",values(xy)},{"dist",values(dist)},{"jacobian",values(j)}});
      }
    }
    Camera initialized;initialized.setFromInit(Eigen::Matrix<S,4,1>(S(100),S(120),S(50),S(60)));
    record["init"]=values(initialized.getParam());
    typename Camera::VecN inc=Camera::VecN::Zero();inc[4]=S(1.5);inc[5]=S(-2);
    initialized+=inc;record["increment"]=values(initialized.getParam());
    if constexpr(N==12)record["increment_rpmax"]=initialized.getRpmax();
    out.push_back(record);
  }
  return out;
}
int main(int argc,char**argv){
  if(argc!=3)return 2;
  std::string model=argv[1],scalar=argv[2];Json result;
  if(model=="eucm"&&scalar=="f32")result=run<basalt::ExtendedUnifiedCamera<float>>();
  else if(model=="eucm"&&scalar=="f64")result=run<basalt::ExtendedUnifiedCamera<double>>();
  else if(model=="radtan8"&&scalar=="f32")result=run<basalt::PinholeRadtan8Camera<float>>();
  else if(model=="radtan8"&&scalar=="f64")result=run<basalt::PinholeRadtan8Camera<double>>();
  else return 2;
  std::cout<<result.dump(2)<<'\n';
}
