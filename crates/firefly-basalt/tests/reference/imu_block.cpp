// Original Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc.
#include <basalt/linearization/imu_block.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
template<typename Derived>
nlohmann::json values(const Eigen::MatrixBase<Derived>& matrix) {
  nlohmann::json result=nlohmann::json::array();
  for(int row=0;row<matrix.rows();++row)for(int col=0;col<matrix.cols();++col)result.push_back(matrix(row,col));
  return result;
}
template<typename S>
nlohmann::json run() {
  using V3=Eigen::Matrix<S,3,1>;using VX=Eigen::Matrix<S,Eigen::Dynamic,1>;using MX=Eigen::Matrix<S,Eigen::Dynamic,Eigen::Dynamic>;
  const V3 bg(S(0.01),S(-0.02),S(0.03)),ba(S(0.1),S(-0.2),S(0.3));
  basalt::IntegratedImuMeasurement<S> measurement(1000000000,bg,ba);
  for(int i=1;i<=6;++i){
    basalt::ImuData<S> sample;sample.t_ns=1000000000+i*5000000;
    sample.accel=V3(S(0.1*i),S(-0.05*i),S(9.81+0.02*i));sample.gyro=V3(S(0.2+0.01*i),S(-0.1+0.02*i),S(0.3-0.01*i));
    measurement.integrate(sample,V3::Constant(S(0.01)),V3::Constant(S(0.001)));
  }
  const V3 gravity(S(0),S(0),S(-9.81)),gyro_weight(S(10),S(20),S(30)),accel_weight(S(2),S(3),S(4));
  basalt::PoseVelBiasState<S> initial(1000000000,Sophus::SE3<S>(Sophus::SO3<S>::exp(V3(S(0.2),S(-0.1),S(0.3))),V3(S(1),S(2),S(-3))),V3(S(0.1),S(-0.2),S(0.3)),bg,ba);
  basalt::PoseVelBiasState<S> next=initial;next.t_ns=1030000000;measurement.predictState(initial,gravity,next);
  Eigen::aligned_map<int64_t,basalt::PoseVelBiasStateWithLin<S>> states;
  states.emplace(initial.t_ns,basalt::PoseVelBiasStateWithLin<S>(initial));states.emplace(next.t_ns,basalt::PoseVelBiasStateWithLin<S>(next));
  Eigen::Matrix<S,15,1> update;for(int i=0;i<15;++i)update[i]=S((i-7)*0.0001);
  for(auto& [timestamp,state]:states){state.setLinTrue();state.applyInc(timestamp==initial.t_ns?update:-update);}
  basalt::AbsOrderMap order;order.abs_order_map={{1000000000,{5,15}},{1030000000,{20,15}}};order.items=2;order.total_size=37;
  basalt::ImuLinData<S> data{gravity,gyro_weight,accel_weight,{}};
  basalt::ImuBlock<S> block(&measurement,&data,order);
  S error=block.linearizeImu(states);
  MX jacobian=MX::Zero(17,37);VX residual=VX::Zero(17);block.add_dense_Q2Jp_Q2r(jacobian,residual,1);
  basalt::DenseAccumulator<S> accumulator;accumulator.reset(37);block.add_dense_H_b(accumulator);
  VX diagonal=VX::Zero(37);block.addJp_diag2(diagonal);
  VX scaling(37),increment(37);for(int i=0;i<37;++i){scaling[i]=S(1)/(S(1)+S(0.03)*S(i));increment[i]=S((i-18)*0.00001);}
  block.scaleJp_cols(scaling);MX scaled=MX::Zero(17,37);VX scaled_residual=VX::Zero(17);block.add_dense_Q2Jp_Q2r(scaled,scaled_residual,1);
  S decrease=S(7);block.backSubstitute(increment,decrease);
  return {{"error",error},{"jacobian",values(jacobian)},{"residual",values(residual)},
    {"h",values(accumulator.getH())},{"b",values(accumulator.getB())},{"diagonal",values(diagonal)},
    {"scaled",values(scaled)},{"scaled_residual",values(scaled_residual)},{"decrease",decrease}};
}
int main(){std::cout<<nlohmann::json({{"f32",run<float>()},{"f64",run<double>()}}).dump(2)<<'\n';}
