// Official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/camera/stereographic_param.hpp>
#include <nlohmann/json.hpp>
#include <iostream>

template <typename Derived>
nlohmann::json values(const Eigen::MatrixBase<Derived>& matrix) {
  nlohmann::json result = nlohmann::json::array();
  for (int row = 0; row < matrix.rows(); ++row)
    for (int col = 0; col < matrix.cols(); ++col)
      result.push_back(matrix(row, col));
  return result;
}
template <typename S>
nlohmann::json run() {
  nlohmann::json result = nlohmann::json::array();
  for (const auto& coordinates : {std::array<double,4>{1,2,3,7}, {-2,0.3,-1,0}, {0,0,1,1}}) {
    Eigen::Matrix<S,4,1> point;
    for (int i = 0; i < 4; ++i) point[i] = S(coordinates[i]);
    Eigen::Matrix<S,2,4> j_project;
    auto projection = basalt::StereographicParam<S>::project(point, &j_project);
    Eigen::Matrix<S,4,2> j_unproject;
    auto ray = basalt::StereographicParam<S>::unproject(projection, &j_unproject);
    result.push_back({{"point", values(point)}, {"projection", values(projection)},
      {"j_project", values(j_project)}, {"ray", values(ray)}, {"j_unproject", values(j_unproject)}});
  }
  return result;
}
int main() {
  std::cout << nlohmann::json({{"f32",run<float>()}, {"f64",run<double>()}}).dump(2) << '\n';
}
