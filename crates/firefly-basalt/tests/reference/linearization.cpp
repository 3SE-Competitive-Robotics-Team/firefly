// SPDX-License-Identifier: BSD-3-Clause
// Official Basalt 0f3b2b52, Eigen 5.0.1. Template definitions are included
// directly so this probe executes the unmodified upstream implementations.
#include "/home/zhongjin/Projects/basalt/src/vi_estimator/ba_base.cpp"
#include "/home/zhongjin/Projects/basalt/src/vi_estimator/landmark_database.cpp"
#include "/home/zhongjin/Projects/basalt/src/vi_estimator/sc_ba_base.cpp"
#include "/home/zhongjin/Projects/basalt/src/linearization/landmark_block.cpp"
#include "/home/zhongjin/Projects/basalt/src/linearization/linearization_abs_qr.cpp"
#include "/home/zhongjin/Projects/basalt/src/linearization/linearization_abs_sc.cpp"
#include "/home/zhongjin/Projects/basalt/src/linearization/linearization_rel_sc.cpp"
#include "/home/zhongjin/Projects/basalt/src/linearization/linearization_base.cpp"
#include <nlohmann/json.hpp>
#include <iostream>
using Json = nlohmann::json;
template<class M> Json matrix(const M& m) {
  Json values = Json::array();
  for (int r = 0; r < m.rows(); ++r)
    for (int c = 0; c < m.cols(); ++c) values.push_back(m(r,c));
  return {{"rows",m.rows()},{"cols",m.cols()},{"values",values}};
}
template<class S> Json run() {
  using V = Eigen::Matrix<S,Eigen::Dynamic,1>;
  using M = Eigen::Matrix<S,Eigen::Dynamic,Eigen::Dynamic>;
  Json output = Json::array();
  for (int scenario : {0,1,2}) for (auto type : {basalt::LinearizationType::ABS_QR,
                    basalt::LinearizationType::ABS_SC,
                    basalt::LinearizationType::REL_SC}) {
    basalt::BundleAdjustmentBase<S> ba;
    bool with_prior=scenario==1,with_imu=scenario==2;
    ba.obs_std_dev = 1.5;
    ba.huber_thresh = 2.;
    auto camera = basalt::GenericCamera<S>::fromString("pinhole");
    camera.setFromInit(Eigen::Matrix<S,4,1>(320.,315.,320.,240.));
    ba.calib.intrinsics = {camera,camera};
    ba.calib.T_i_c = {Sophus::SE3<S>(),Sophus::SE3<S>(Sophus::SO3<S>(),Eigen::Matrix<S,3,1>(.125,0.,0.))};
    basalt::AbsOrderMap order;
    for (int i=0;i<3;++i) {
      int64_t timestamp = (i+1)*10;
      Sophus::SE3<S> pose(Sophus::SO3<S>(),Eigen::Matrix<S,3,1>(i*.25,i*-.0625,0.));
      ba.frame_poses.emplace(timestamp,basalt::PoseStateWithLin<S>(timestamp,pose,with_prior));
      if(with_prior) {Eigen::Matrix<S,6,1> delta;delta<<.001*i,-.002*i,.0005*i,.000125*i,-.00025*i,.000375*i;ba.frame_poses.at(timestamp).applyInc(delta);}
      order.abs_order_map[timestamp] = {i*6,6};
    }
    order.total_size=18;order.items=3;
    using V3=Eigen::Matrix<S,3,1>;
    basalt::IntegratedImuMeasurement<S> measurement(1000000000,V3::Zero(),V3::Zero());
    V3 gravity=V3::Zero(),gyro_weight=V3::Constant(S(2)),accel_weight=V3::Constant(S(3));
    basalt::ImuLinData<S> imu{gravity,gyro_weight,accel_weight,{}};
    if(with_imu){
      for(int i=1;i<=2;++i){basalt::ImuData<S> sample;sample.t_ns=1000000000+i*50000000;sample.accel.setZero();sample.gyro.setZero();measurement.integrate(sample,V3::Constant(S(.01)),V3::Constant(S(.001)));}
      for(int i=0;i<2;++i){int64_t timestamp=1000000000+i*100000000;basalt::PoseVelBiasState<S> state(timestamp,Sophus::SE3<S>(Sophus::SO3<S>(),V3(S(.001*i),S(-.002*i),S(.003*i))),V3::Zero(),V3::Zero(),V3::Zero());ba.frame_states.emplace(timestamp,basalt::PoseVelBiasStateWithLin<S>(state));order.abs_order_map[timestamp]={18+i*15,15};}
      order.total_size=48;order.items=5;imu.imu_meas.emplace(1000000000,&measurement);
    }
    for (int id=0;id<3;++id) {
      basalt::Keypoint<S> point;
      point.direction = Eigen::Matrix<S,2,1>(.125+id*.03125,-.125+id*.015625);
      point.inv_dist = .25+id*.0625;
      point.host_kf_id=basalt::TimeCamId(10,0);
      ba.lmdb.addLandmark(id,point);
      for (int i=0;i<3;++i) for (int cam=0;cam<2;++cam) {
        basalt::KeypointObservation<S> observation;
        observation.kpt_id=id;
        observation.pos=Eigen::Matrix<S,2,1>(400.+id*8.-i*12.-cam*7.,160.+id*4.+i*3.);
        ba.lmdb.addObservation(basalt::TimeCamId((i+1)*10,cam),observation);
      }
    }
    typename basalt::LinearizationBase<S,6>::Options options;
    options.linearization_type=type;
    options.lb_options.obs_std_dev=ba.obs_std_dev;
    options.lb_options.huber_parameter=ba.huber_thresh;
    basalt::MargLinData<S> prior;
    prior.order=order;prior.is_sqrt=true;
    prior.H=M::Identity(18,18)*S(.5);
    prior.b=V::LinSpaced(18,-.125,.125);
    Json schur;
    if(with_prior && type!=basalt::LinearizationType::ABS_QR) {
      Eigen::aligned_vector<typename basalt::ScBundleAdjustmentBase<S>::RelLinData> blocks;
      S block_error=0;
      basalt::ScBundleAdjustmentBase<S>::linearizeHelperStatic(blocks,ba.lmdb.getObservations(),&ba,block_error);
      const auto& block=blocks.at(0);
      Json landmarks=Json::array(),frames=Json::array();
      for(int id=0;id<3;++id) landmarks.push_back({{"hll",matrix(block.Hll.at(id))},{"inverse",matrix(block.Hllinv.at(id))},{"bl",matrix(block.bl.at(id))}});
      for(size_t i=0;i<block.Hpppl.size();++i) {
        const auto& frame=block.Hpppl[i];Json cross=Json::array();
        for(const auto& value:frame.Hpl)cross.push_back(matrix(value));
        auto host=block.order[i].first,target=block.order[i].second;
        Eigen::Matrix<S,4,4> transform=Eigen::Matrix<S,4,4>::Identity();
        if(host!=target)transform=basalt::computeRelPose(ba.getPoseStateWithLin(host.frame_id).getPose(),ba.calib.T_i_c[host.cam_id],ba.getPoseStateWithLin(target.frame_id).getPose(),ba.calib.T_i_c[target.cam_id]).matrix();
        Json observations=Json::array();
        for(int id:frame.lm_id) {
          const auto& point=ba.lmdb.getLandmark(id);
          Eigen::Matrix<S,2,1> residual;Eigen::Matrix<S,2,6> jp;Eigen::Matrix<S,2,3> jl;
          std::visit([&](const auto& cam){basalt::linearizePoint(point.obs.at(target),point,transform,cam,residual,&jp,&jl);},ba.calib.intrinsics[target.cam_id].variant);
          observations.push_back({{"residual",matrix(residual)},{"jp",matrix(jp)},{"jl",matrix(jl)}});
        }
        frames.push_back({{"transform",matrix(transform)},{"observations",observations},{"dh",matrix(block.d_rel_d_h[i])},{"dt",matrix(block.d_rel_d_t[i])},{"hpp",matrix(frame.Hpp)},{"bp",matrix(frame.bp)},{"cross",cross}});
      }
      M reduced_h;V reduced_b;
      basalt::ScBundleAdjustmentBase<S>::linearizeRel(block,reduced_h,reduced_b);
      schur={{"landmarks",landmarks},{"frames",frames},{"reduced_h",matrix(reduced_h)},{"reduced_b",matrix(reduced_b)}};
    }
    auto linearization=basalt::LinearizationBase<S,6>::create(&ba,order,options,with_prior?&prior:nullptr,with_imu?&imu:nullptr);
    bool valid=true;
    S error=linearization->linearizeProblem(&valid);
    linearization->performQR();
    M h;V b;
    linearization->get_dense_H_b(h,b);
    V increment(order.total_size);for(int i=0;i<increment.size();++i)increment[i]=(i-8)*.000125;
    Json root;
    if(type==basalt::LinearizationType::ABS_QR) {M jacobian;V residual;linearization->get_dense_Q2Jp_Q2r(jacobian,residual);root={{"jacobian",matrix(jacobian)},{"residual",matrix(residual)}};}
    S decrease=linearization->backSubstitute(increment);
    Json points=Json::array();for(int id=0;id<3;++id){const auto&p=ba.lmdb.getLandmark(id);points.push_back({{"direction",matrix(p.direction)},{"inverse_distance",p.inv_dist}});}
    output.push_back({{"schur",schur},{"root",root},{"with_imu",with_imu},{"with_prior",with_prior},{"mode",int(type)},{"error",error},{"valid",valid},{"h",matrix(h)},{"b",matrix(b)},{"model_decrease",decrease},{"points",points}});
  }
  return output;
}
int main(int argc,char**argv) {
  if(argc==2&&std::string(argv[1])=="f32")std::cout<<run<float>().dump()<<'\n';
  else std::cout<<run<double>().dump()<<'\n';
}
