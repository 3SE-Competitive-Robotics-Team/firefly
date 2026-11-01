// SPDX-License-Identifier: BSD-3-Clause
// Runs the complete original gen_data(), including both original RNG streams.
#define main original_vio_sim_main
#include "/home/zhongjin/Projects/basalt/src/vio_sim.cpp"
#undef main
#include <nlohmann/json.hpp>
#include <filesystem>
using Json=nlohmann::json;
template<class M>Json vector(const M& value){Json result=Json::array();for(int i=0;i<value.size();++i)result.push_back(value[i]);return result;}
Json pose(const Sophus::SE3d& value){return {{"translation",vector(value.translation())},{"quaternion",vector(value.unit_quaternion().coeffs())}};}
int main(){
    std::srand(1);gen.seed(1);NUM_POINTS=32;
    auto cam=basalt::GenericCamera<double>::fromString("pinhole");cam.setFromInit(Eigen::Vector4d(320,315,320,240));
    calib.intrinsics={cam,cam};calib.T_i_c={Sophus::SE3d(),Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.125,0,0))};calib.resolution={Eigen::Vector2i(640,480),Eigen::Vector2i(640,480)};
    calib.imu_update_rate=200;calib.gyro_noise_std=Eigen::Vector3d(.001,.002,.003);calib.accel_noise_std=Eigen::Vector3d(.01,.02,.03);calib.gyro_bias_std=Eigen::Vector3d(.0001,.0002,.0003);calib.accel_bias_std=Eigen::Vector3d(.001,.002,.003);
    marg_data_path="/home/zhongjin/Projects/basalt/build/results/vio-sim-reference";std::filesystem::create_directories(marg_data_path);
    std::ostringstream discarded;auto* old=std::cout.rdbuf(discarded.rdbuf());gen_data();std::cout.rdbuf(old);
    Json knots=Json::array(),frames=Json::array(),imu=Json::array(),points=Json::array(),observations=Json::array();
    for(size_t i=0;i<gt_spline.numKnots();++i)knots.push_back(pose(gt_spline.getKnot(i)));
    for(size_t i:{size_t(0),size_t(1),size_t(200),size_t(999)}){frames.push_back({{"index",i},{"time",gt_frame_t_ns[i]},{"pose",pose(gt_frame_T_w_i[i])}});for(size_t camera=0;camera<2;++camera){basalt::TimeCamId key(gt_frame_t_ns[i],camera);auto& exact=gt_observations[key];auto& noisy=noisy_observations[key];Json xy=Json::array(),noise=Json::array();for(const auto& p:exact.pos)xy.push_back(vector(p));for(const auto& p:noisy.pos)noise.push_back(vector(p));observations.push_back({{"frame",i},{"camera",camera},{"ids",exact.id},{"exact",xy},{"noisy",noise}});}}
    for(size_t i:{size_t(0),size_t(1),size_t(100),gt_imu_t_ns.size()-1})imu.push_back({{"index",i},{"time",gt_imu_t_ns[i]},{"accel",vector(gt_accel[i])},{"gyro",vector(gt_gyro[i])},{"velocity",vector(gt_vel[i])},{"noisy_accel",vector(noisy_accel[i])},{"noisy_gyro",vector(noisy_gyro[i])},{"accel_bias",vector(gt_accel_bias[i])},{"gyro_bias",vector(gt_gyro_bias[i])}});
    for(const auto& p:gt_points)points.push_back(vector(p));
    std::cout<<Json{{"knots",knots},{"frames",frames},{"imu",imu},{"points",points},{"observations",observations},{"imu_count",gt_imu_t_ns.size()},{"final_accel_bias",vector(gt_accel_bias.back())},{"final_gyro_bias",vector(gt_gyro_bias.back())}}.dump()<<'\n';
}
