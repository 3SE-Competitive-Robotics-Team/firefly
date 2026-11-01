// Original Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc.
#include <basalt/utils/imu_types.h>
#include <nlohmann/json.hpp>
#include <iostream>
template <typename Derived>
nlohmann::json values(const Eigen::MatrixBase<Derived>& matrix) {
  nlohmann::json result=nlohmann::json::array();
  for(int row=0;row<matrix.rows();++row)
    for(int col=0;col<matrix.cols();++col) result.push_back(matrix(row,col));
  return result;
}
template<typename S>
nlohmann::json snapshot(const basalt::PoseVelBiasStateWithLin<S>& state) {
  const auto& current=state.getState();
  basalt::PoseStateWithLin<S> pose(state);
  return {{"linearized",state.isLinearized()},{"t_ns",state.getT_ns()},
    {"pose",values(current.T_w_i.matrix())},{"velocity",values(current.vel_w_i)},
    {"gyro",values(current.bias_gyro)},{"accel",values(current.bias_accel)},
    {"delta",values(state.getDelta())},{"linearized_pose",values(state.getStateLin().T_w_i.matrix())},
    {"pose_only",values(pose.getPose().matrix())},{"pose_delta",values(pose.getDelta())}};
}
template<typename S>
nlohmann::json run() {
  using V3=Eigen::Matrix<S,3,1>;
  basalt::PoseVelBiasStateWithLin<S> state(123,Sophus::SE3<S>(Sophus::SO3<S>::exp(V3(S(0.2),S(-0.3),S(0.4))),V3(S(1),S(2),S(3))),V3(S(0.1),S(-0.2),S(0.3)),V3::Zero(),V3::Zero(),false);
  Eigen::Matrix<S,15,1> increment;
  for(int i=0;i<15;++i) increment[i]=S((i-7)*0.01);
  nlohmann::json result=nlohmann::json::array();
  result.push_back(snapshot(state));
  state.applyInc(increment);result.push_back(snapshot(state));
  state.setLinTrue();state.backup();
  state.applyInc(increment);state.applyInc(increment);result.push_back(snapshot(state));
  state.restore();result.push_back(snapshot(state));
  state.applyInc(increment);state.setLinFalse();result.push_back(snapshot(state));
  state.applyInc(increment);result.push_back(snapshot(state));
  return result;
}
int main(){std::cout<<nlohmann::json({{"f32",run<float>()},{"f64",run<double>()}}).dump(2)<<'\n';}
