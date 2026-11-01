// Official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/camera/pinhole_camera.hpp>
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
  for (const auto& camera : basalt::PinholeCamera<S>::getTestProjections()) {
    for (S depth : {S(2), S(-1), Sophus::Constants<S>::epsilonSqrt()}) {
      Eigen::Matrix<S, 4, 1> point;
      point << S(0.25), S(-0.75), depth, S(1);
      Eigen::Matrix<S, 2, 1> pixel;
      Eigen::Matrix<S, 2, 4> j_point, j_param;
      const bool valid = camera.project(point, pixel, &j_point, &j_param);
      Eigen::Matrix<S, 4, 1> ray;
      Eigen::Matrix<S, 4, 2> j_pixel;
      Eigen::Matrix<S, 4, 4> j_intrinsics;
      camera.unproject(pixel, ray, &j_pixel, &j_intrinsics);
      result.push_back({{"param", values(camera.getParam())}, {"point", values(point)},
        {"pixel", values(pixel)}, {"valid", valid}, {"j_point", values(j_point)},
        {"j_param", values(j_param)}, {"ray", values(ray)},
        {"j_pixel", values(j_pixel)}, {"j_intrinsics", values(j_intrinsics)}});
    }
  }
  return result;
}
int main() {
  std::cout << nlohmann::json({{"f32", run<float>()}, {"f64", run<double>()}}).dump(2) << '\n';
}
