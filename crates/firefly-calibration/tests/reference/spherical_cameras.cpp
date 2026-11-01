// SPDX-License-Identifier: BSD-3-Clause
// Official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/camera/double_sphere_camera.hpp>
#include <basalt/camera/unified_camera.hpp>
#include <basalt/camera/kannala_brandt_camera4.hpp>
#include <basalt/camera/fov_camera.hpp>
#include <basalt/camera/bal_camera.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <cmath>
using Json=nlohmann::json;
template<class M> Json values(const M& m) {
  Json out=Json::array();
  for(int r=0;r<m.rows();++r) for(int c=0;c<m.cols();++c) {
    double v=m(r,c);
    if(std::isnan(v))out.push_back("NaN");
    else if(std::isinf(v))out.push_back(v>0?"+Inf":"-Inf");
    else out.push_back(v);
  }
  return out;
}
template<class Camera> Json run() {
  using S=typename Camera::Scalar;
  constexpr int N=Camera::N;
  auto cameras=Camera::getTestProjections();
  if constexpr(N>=4) for(S alpha:{S(0),S(.5),S(.75),S(1)}) {
    typename Camera::VecN p;
    p.setZero();p[0]=S(200);p[1]=S(220);p[2]=S(100);p[3]=S(90);p[N-1]=alpha;
    if constexpr(N==6)p[4]=S(.2);
    cameras.emplace_back(p);
  }
  Json out=Json::array();
  for(const auto& camera:cameras) {
    Json record={{"param",values(camera.getParam())}};
    for(Eigen::Matrix<S,4,1> p: {Eigen::Matrix<S,4,1>(S(.25),S(-.75),S(2),S(1)),
        {S(.25),S(-.75),S(-1),S(.7)},{S(0),S(0),S(1),S(0)},
        {S(0),S(0),S(-1),S(0)},{S(0),S(0),S(0),S(0)}}) {
      Eigen::Matrix<S,2,1> pixel;
      Eigen::Matrix<S,2,4> jp;
      Eigen::Matrix<S,2,N> ji;
      bool valid=camera.project(p,pixel,&jp,&ji);
      record["project"].push_back({{"point",values(p)},{"pixel",values(pixel)},
        {"valid",valid},{"j_point",values(jp)},{"j_param",values(ji)}});
    }
    S cx=S(0),cy=S(0);
    if constexpr(N>=4){cx=camera.getParam()[2];cy=camera.getParam()[3];}
    for(Eigen::Matrix<S,2,1> pixel: {Eigen::Matrix<S,2,1>(cx,cy),
        {S(150),S(120)},{S(-50),S(300)},{S(2000),S(1000)}}) {
      Eigen::Matrix<S,4,1> ray;
      Eigen::Matrix<S,4,2> jp;
      Eigen::Matrix<S,4,N> ji;
      if constexpr(N==3) {
        bool valid=camera.unproject(pixel,ray);
        record["unproject"].push_back({{"pixel",values(pixel)},{"ray",values(ray)},{"valid",valid}});
      } else {
        bool valid=camera.unproject(pixel,ray,&jp,&ji);
        record["unproject"].push_back({{"pixel",values(pixel)},{"ray",values(ray)},
          {"valid",valid},{"j_pixel",values(jp)},{"j_param",values(ji)}});
      }
    }
    Camera initialized;
    initialized.setFromInit(Eigen::Matrix<S,4,1>(S(1),S(2),S(3),S(4)));
    record["init"]=values(initialized.getParam());
    if constexpr(N==8) {
      for(S radius:{S(.1),S(1),S(5)}) {
        S derivative=S(42);S theta=camera.template solveTheta<0>(radius,derivative);
        record["solve_theta"].push_back({{"iterations",0},{"radius",radius},{"theta",theta},{"derivative",derivative}});
        theta=camera.template solveTheta<1>(radius,derivative);
        record["solve_theta"].push_back({{"iterations",1},{"radius",radius},{"theta",theta},{"derivative",derivative}});
        theta=camera.template solveTheta<3>(radius,derivative);
        record["solve_theta"].push_back({{"iterations",3},{"radius",radius},{"theta",theta},{"derivative",derivative}});
      }
    }
    typename Camera::VecN inc=Camera::VecN::Constant(S(2));
    initialized+=inc;record["increment_high"]=values(initialized.getParam());
    inc.setConstant(S(-4));initialized+=inc;record["increment_low"]=values(initialized.getParam());
    out.push_back(record);
  }
  return out;
}
int main(int argc,char**argv) {
  if(argc!=3)return 2;
  const std::string model=argv[1],scalar=argv[2];Json out;
  if(model=="ds" && scalar=="f32")out=run<basalt::DoubleSphereCamera<float>>();
  else if(model=="ds" && scalar=="f64")out=run<basalt::DoubleSphereCamera<double>>();
  else if(model=="ucm" && scalar=="f32")out=run<basalt::UnifiedCamera<float>>();
  else if(model=="ucm" && scalar=="f64")out=run<basalt::UnifiedCamera<double>>();
  else if(model=="kb4" && scalar=="f32")out=run<basalt::KannalaBrandtCamera4<float>>();
  else if(model=="kb4" && scalar=="f64")out=run<basalt::KannalaBrandtCamera4<double>>();
  else if(model=="fov" && scalar=="f32")out=run<basalt::FovCamera<float>>();
  else if(model=="fov" && scalar=="f64")out=run<basalt::FovCamera<double>>();
  else if(model=="bal" && scalar=="f32")out=run<basalt::BalCamera<float>>();
  else if(model=="bal" && scalar=="f64")out=run<basalt::BalCamera<double>>();
  else return 2;
  std::cout<<out.dump(2)<<'\n';
}
