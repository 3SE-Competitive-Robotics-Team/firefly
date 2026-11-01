// Official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/imu/imu_types.h>
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
  using V3 = Eigen::Matrix<S,3,1>;
  nlohmann::json result = nlohmann::json::array();
  for (double magnitude : {0., 1e-12, 0.0002, 0.3, 1.}) {
    const V3 initial_rotation(S(0.2), S(-0.1), S(0.4));
    basalt::PoseVelBiasState<S> initial(1234567890123,
      Sophus::SE3<S>(Sophus::SO3<S>::exp(initial_rotation), V3(S(1), S(-2), S(3))),
      V3(S(0.1), S(0.2), S(-0.3)), V3(S(0.001), S(-0.002), S(0.003)),
      V3(S(-0.04), S(0.05), S(0.06)));
    Eigen::Matrix<S,15,1> increment;
    for(int i=0;i<15;++i) increment[i] = S((i-7)*0.1*magnitude);
    auto updated = initial;
    updated.applyInc(increment);
    Eigen::Matrix<S,3,3> jr, jl, jri, jli;
    V3 rotation = increment.template segment<3>(3);
    Sophus::rightJacobianSO3(rotation,jr);
    Sophus::leftJacobianSO3(rotation,jl);
    Sophus::rightJacobianInvSO3(rotation,jri);
    Sophus::leftJacobianInvSO3(rotation,jli);
    result.push_back({{"increment",values(increment)}, {"pose",values(updated.T_w_i.matrix())},
      {"velocity",values(updated.vel_w_i)}, {"gyro",values(updated.bias_gyro)},
      {"accel",values(updated.bias_accel)}, {"diff",values(initial.diff(updated))},
      {"jr",values(jr)}, {"jl",values(jl)}, {"jri",values(jri)}, {"jli",values(jli)}});
  }
  return result;
}
int main() {
  std::cout << nlohmann::json({{"f32",run<float>()}, {"f64",run<double>()}}).dump(2) << '\n';
}
