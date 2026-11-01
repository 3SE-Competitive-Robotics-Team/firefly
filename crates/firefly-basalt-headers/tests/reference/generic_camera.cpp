// SPDX-License-Identifier: BSD-3-Clause
// Execute official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/camera/generic_camera.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class M> Json values(const M& m){Json out=Json::array();for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c){double v=m(r,c);if(std::isnan(v))out.push_back("NaN");else if(std::isinf(v))out.push_back(v>0?"+Inf":"-Inf");else out.push_back(v);}return out;}
template<class Points>Json rows(const Points& points){Json out=Json::array();for(const auto&p:points)out.push_back(values(p));return out;}
template<class S>Json run(){
  Json out=Json::array();
  using V2=Eigen::Matrix<S,2,1>;using V3=Eigen::Matrix<S,3,1>;using V4=Eigen::Matrix<S,4,1>;
  for(const std::string name:{"eucm","ds","kb4","ucm","pinhole","pinhole-radtan8","unknown","fov","bal"}){
    auto camera=basalt::GenericCamera<S>::fromString(name);
    camera.setFromInit(V4(S(200),S(220),S(100),S(90)));
    Eigen::Matrix<S,Eigen::Dynamic,1> inc(camera.getN());for(int i=0;i<inc.size();++i)inc[i]=S(.001)*S(i+1);camera.applyInc(inc);
    Json record={{"requested_name",name},{"name",camera.getName()},{"n",camera.getN()},{"param",values(camera.getParam())}};
    V4 point(S(.2),S(.3),S(2),S(1));V2 pixel;Eigen::Matrix<S,2,4> jp;
    bool valid=camera.project(point,pixel,&jp);record["single_project"]={{"point",values(point)},{"pixel",values(pixel)},{"jacobian",values(jp)},{"valid",valid}};
    V4 ray;Eigen::Matrix<S,4,2> ju;
    if(camera.getName()=="pinhole-radtan8")valid=camera.unproject(pixel,ray);
    else valid=camera.unproject(pixel,ray,&ju);
    record["single_unproject"]={{"pixel",values(pixel)},{"ray",values(ray)},{"valid",valid}};
    if(camera.getName()!="pinhole-radtan8")record["single_unproject"]["jacobian"]=values(ju);
    Eigen::aligned_vector<V4> points{point,V4(S(-.1),S(.4),S(1),S(.5)),V4(S(0),S(0),S(-1),S(0))};
    Eigen::aligned_vector<V2> pixels,angles;std::vector<bool> valid_batch;
    camera.project(points,pixels,valid_batch);record["batch"]={{"points",rows(points)},{"pixels",rows(pixels)},{"valid",valid_batch}};
    Eigen::Matrix<S,4,4> transform=Eigen::Matrix<S,4,4>::Identity();transform(0,0)=S(0);transform(0,1)=S(-1);transform(1,0)=S(1);transform(1,1)=S(0);transform(0,3)=S(.3);transform(1,3)=S(-.2);transform(2,3)=S(.1);
    camera.project(points,transform,pixels,valid_batch,angles);record["transformed"]={{"transform",values(transform)},{"pixels",rows(pixels)},{"valid",valid_batch},{"angles",rows(angles)}};
    Eigen::aligned_vector<V3> points3;for(const auto&p:points)points3.push_back(p.template head<3>());
    camera.project(points3,transform,pixels,valid_batch);record["transformed3"]={{"pixels",rows(pixels)},{"valid",valid_batch}};
    Eigen::aligned_vector<V2> input_pixels{V2(S(120),S(130)),V2(S(100),S(90))};
    Eigen::aligned_vector<V4> rays;camera.unproject(input_pixels,rays,valid_batch);record["unproject_batch"]={{"pixels",rows(input_pixels)},{"rays",rows(rays)},{"valid",valid_batch}};
    out.push_back(record);
  }return out;
}
int main(int argc,char**argv){if(argc!=2)return 2;std::string scalar=argv[1];if(scalar=="f32")std::cout<<run<float>().dump(2);else if(scalar=="f64")std::cout<<run<double>().dump(2);else return 2;std::cout<<'\n';}
