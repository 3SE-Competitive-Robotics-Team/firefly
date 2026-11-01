// Independent Eigen/Sophus oracle for Basalt's gravity initialization.
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <sophus/so3.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
template<class S> nlohmann::json run() {
  nlohmann::json out=nlohmann::json::array();
  const double inputs[][3]={{0,0,1},{0,0,-1},{0.000001,0,-1},{0.0001,-0.0003,-1},{1,2,3},{0,0,0},{0,1,-1e8}};
  for(auto &input:inputs){Eigen::Matrix<S,3,1>a{S(input[0]),S(input[1]),S(input[2])};
    Eigen::Matrix<S,3,1>b=Eigen::Matrix<S,3,1>::UnitZ();
    Sophus::SO3<S> rotation(Eigen::Quaternion<S>::FromTwoVectors(a,b));
    auto q=rotation.unit_quaternion();out.push_back({q.x(),q.y(),q.z(),q.w()});}
  return out;
}
int main(){std::cout<<nlohmann::json{{"f32",run<float>()},{"f64",run<double>()}}.dump(2)<<'\n';}
