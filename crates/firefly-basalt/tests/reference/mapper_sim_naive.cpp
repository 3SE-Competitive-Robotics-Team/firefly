// SPDX-License-Identifier: BSD-3-Clause
// Calls the original archive loading / keyframe-selection translation unit.
#define main original_mapper_sim_naive_main
#include "/home/zhongjin/Projects/basalt/src/mapper_sim_naive.cpp"
#undef main
#include <nlohmann/json.hpp>
using Json=nlohmann::json;
template<class M>Json vector(const M& v){Json out=Json::array();for(int i=0;i<v.size();++i)out.push_back(v[i]);return out;}
int main(int argc,char**argv){if(argc!=3)return 1;marg_data_path=argv[1];
 auto data=std::make_shared<basalt::MargData>();data->aom.total_size=18;data->abs_H=Eigen::MatrixXd::Identity(18,18);data->abs_b=Eigen::VectorXd::Zero(18);data->use_imu=true;
 for(int i=0;i<3;++i){int64_t t=i*100000000LL;data->kfs_all.insert(t);data->frame_poses.emplace(t,basalt::PoseStateWithLin<double>(t,Sophus::SE3d()));data->aom.abs_order_map[t]={i*6,6};auto flow=std::make_shared<basalt::OpticalFlowResult>();flow->t_ns=t;flow->observations.resize(2);data->opt_flow_res.push_back(flow);}data->kfs_to_marg={0};
 {basalt::MargDataSaver saver(marg_data_path);saver.in_marg_queue.push(data);saver.in_marg_queue.push(nullptr);}
 auto camera=basalt::GenericCamera<double>::fromString("pinhole");camera.setFromInit(Eigen::Vector4d(300,300,320,240));calib.intrinsics={camera,camera};calib.T_i_c={Sophus::SE3d(),Sophus::SE3d(Sophus::SO3d(),Eigen::Vector3d(.1,0,0))};calib.resolution={Eigen::Vector2i(640,480),Eigen::Vector2i(640,480)};calib.imu_update_rate=200;
 {std::ofstream stream(marg_data_path+"/calibration.json");cereal::JSONOutputArchive archive(stream);archive(calib);}
 int64_t interval=100000000;Eigen::aligned_vector<Sophus::SE3d> knots(8,Sophus::SE3d());Eigen::aligned_vector<Eigen::Vector3d> accel,gyro,exact_accel,bias;
 std::vector<int64_t> frames{0,100000000,200000000},imu;for(int i=0;i<=60;++i){imu.push_back(i*5000000LL);accel.emplace_back(.001*i,0,9.81);gyro.emplace_back(0,0,0);exact_accel.emplace_back(0,0,9.81);bias.emplace_back(0,0,0);}
 Eigen::aligned_vector<Eigen::Vector3d> points;for(int i=0;i<8;++i)points.emplace_back(-.4+.1*i,.1*(i%3),3.+.1*i);
 std::map<basalt::TimeCamId,basalt::SimObservations> observations;for(auto t:frames)for(int cam=0;cam<2;++cam){auto &obs=observations[basalt::TimeCamId(t,cam)];for(int i=0;i<int(points.size());++i){Eigen::Vector2d pixel;calib.intrinsics[cam].project(Eigen::Vector3d(points[i]-calib.T_i_c[cam].translation()),pixel);obs.id.push_back(i);obs.pos.push_back(pixel);}}
 {std::ofstream stream(marg_data_path+"/gt_spline.cereal");cereal::JSONOutputArchive archive(stream);archive(cereal::make_nvp("t_ns",interval),cereal::make_nvp("knots",knots),cereal::make_nvp("noisy_accel",accel),cereal::make_nvp("noisy_gyro",gyro),cereal::make_nvp("noisy_accel",exact_accel),cereal::make_nvp("gt_gyro",gyro),cereal::make_nvp("gt_points",points),cereal::make_nvp("gt_accel_bias",bias),cereal::make_nvp("gt_gyro_bias",bias),cereal::make_nvp("gt_observations",observations),cereal::make_nvp("noisy_observations",observations),cereal::make_nvp("gt_points",points),cereal::make_nvp("gt_frame_t_ns",frames),cereal::make_nvp("gt_imu_t_ns",imu));}
 gen_data();Json out={{"keyframes",kf_t_ns},{"frames",gt_frame_t_ns},{"imu",gt_imu_t_ns}};for(auto v:gt_accel)out["accel"].push_back(vector(v));for(auto v:noisy_accel)out["noisy_accel"].push_back(vector(v));for(auto v:gt_frame_t_w_i)out["positions"].push_back(vector(v));for(auto v:gt_points)out["points"].push_back(vector(v));std::ofstream(argv[2])<<out.dump(2)<<'\n';
}
