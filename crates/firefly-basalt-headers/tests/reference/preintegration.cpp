// Official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/imu/preintegration.h>
#include <nlohmann/json.hpp>
#include <iostream>

template <typename Derived>
nlohmann::json values(const Eigen::MatrixBase<Derived>& matrix) {
  nlohmann::json result = nlohmann::json::array();
  for (int row = 0; row < matrix.rows(); ++row)
    for (int col = 0; col < matrix.cols(); ++col)
      result.push_back(matrix(row,col));
  return result;
}
template <typename S>
nlohmann::json run() {
  using V3 = Eigen::Matrix<S,3,1>;
  using M9 = Eigen::Matrix<S,9,9>;
  using M93 = Eigen::Matrix<S,9,3>;
  nlohmann::json result = nlohmann::json::array();
  for (int count : {0,3,12}) {
    const V3 bg(S(0.01),S(-0.02),S(0.03)), ba(S(0.1),S(-0.2),S(0.3));
    basalt::IntegratedImuMeasurement<S> measurement(1000000000,bg,ba);
    nlohmann::json samples = nlohmann::json::array();
    for (int i=1;i<=count;++i) {
      basalt::ImuData<S> data;
      data.t_ns = 1000000000 + i*5000000;
      data.accel = V3(S(0.1*i),S(-0.05*i),S(9.81+0.02*i));
      data.gyro = V3(S(0.2+0.01*i),S(-0.1+0.02*i),S(0.3-0.01*i));
      measurement.integrate(data,V3(S(0.01),S(0.02),S(0.03)),V3(S(0.001),S(0.002),S(0.003)));
      samples.push_back({{"t_ns",data.t_ns},{"accel",values(data.accel)},{"gyro",values(data.gyro)}});
    }
    basalt::PoseVelState<S> initial(1000000000,
      Sophus::SE3<S>(Sophus::SO3<S>::exp(V3(S(0.2),S(-0.1),S(0.3))),V3(S(1),S(2),S(-3))),
      V3(S(0.1),S(-0.2),S(0.3)));
    const V3 gravity(S(0),S(0),S(-9.81));
    basalt::PoseVelState<S> predicted;
    predicted.t_ns=777;
    measurement.predictState(initial,gravity,predicted);
    auto next=predicted;
    Eigen::Matrix<S,9,1> perturb;
    for (int i=0;i<9;++i) perturb[i]=S(0.002*(i-4));
    next.applyInc(perturb);
    M9 j0,j1; M93 jbg,jba;
    const V3 current_bg=bg+V3(S(0.001),S(-0.002),S(0.003));
    const V3 current_ba=ba+V3(S(-0.01),S(0.02),S(-0.03));
    auto residual=measurement.residual(initial,gravity,next,current_bg,current_ba,&j0,&j1,&jbg,&jba);
    result.push_back({{"samples",samples},{"delta_pose",values(measurement.getDeltaState().T_w_i.matrix())},
      {"delta_velocity",values(measurement.getDeltaState().vel_w_i)}, {"cov",values(measurement.get_cov())},
      {"sqrt_cov_inv",values(measurement.get_sqrt_cov_inv())},{"cov_inv",values(measurement.get_cov_inv())},
      {"dba",values(measurement.get_d_state_d_ba())},{"dbg",values(measurement.get_d_state_d_bg())},
      {"predicted_pose",values(predicted.T_w_i.matrix())},{"predicted_velocity",values(predicted.vel_w_i)},
      {"predicted_t_ns",predicted.t_ns},{"residual",values(residual)},
      {"j0",values(j0)},{"j1",values(j1)},{"jbg",values(jbg)},{"jba",values(jba)}});
  }
  return result;
}
int main() { std::cout << nlohmann::json({{"f32",run<float>()},{"f64",run<double>()}}).dump(2) << '\n'; }
