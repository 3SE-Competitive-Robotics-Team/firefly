// SPDX-License-Identifier: BSD-3-Clause
// Invokes the original, unmodified controller object. Private access is probe-only.
#include <pangolin/pangolin.h>
#include <pangolin/display/image_view.h>
#include <basalt/calibration/calibration_helper.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/optimization/spline_optimize.h>
#include <basalt/utils/test_utils.h>
#include <fstream>
#include <nlohmann/json.hpp>
#define private public
#include <basalt/calibration/cam_imu_calib.h>
#undef private
using Json=nlohmann::json;
class Dataset:public basalt::VioDataset {
 public:std::vector<int64_t> times;Eigen::aligned_vector<basalt::AccelData> accel;Eigen::aligned_vector<basalt::GyroData> gyro;std::vector<int64_t> gt;Eigen::aligned_vector<Sophus::SE3d> poses;
 size_t get_num_cams()const override{return 2;}std::vector<int64_t>&get_image_timestamps()override{return times;}const Eigen::aligned_vector<basalt::AccelData>&get_accel_data()const override{return accel;}const Eigen::aligned_vector<basalt::GyroData>&get_gyro_data()const override{return gyro;}const std::vector<int64_t>&get_gt_timestamps()const override{return gt;}const Eigen::aligned_vector<Sophus::SE3d>&get_gt_pose_data()const override{return poses;}int64_t get_mocap_to_imu_offset_ns()const override{return 0;}std::vector<basalt::ImageData>get_image_data(int64_t)override{return std::vector<basalt::ImageData>(2);}
};
Json matrix(const Sophus::SE3d &pose){Json result=Json::array();auto m=pose.matrix();for(int r=0;r<4;++r)for(int c=0;c<4;++c)result.push_back(m(r,c));return result;}
int main(int argc,char**argv){if(argc!=3)return 1;
 basalt::CamImuCalib c("","bag",argv[2],"","probe",1,{.02,.001,.0001,.00001},false);
 auto d=std::make_shared<Dataset>();c.vio_dataset=d;c.calib_opt=std::make_shared<basalt::SplineOptimization<5,double>>();auto &opt=*c.calib_opt;opt.resetCalib(2,{"pinhole","pinhole"});opt.resetMocapCalib();
 for(int camera=0;camera<2;++camera){opt.calib->intrinsics[camera].setFromInit(Eigen::Vector4d(350,370,320,240));opt.calib->T_i_c[camera]=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.01,-.02*camera,.03)),Eigen::Vector3d(.1*camera,.02,-.01));}
 Json out;for(int i=0;i<=200;++i){basalt::AccelData a;a.timestamp_ns=i*5000000LL;a.data=Eigen::Vector3d(.1+.001*i,-.2,9.7);d->accel.push_back(a);basalt::GyroData g;g.timestamp_ns=a.timestamp_ns;g.data=Eigen::Vector3d(.2+.001*i,-.1+.00001*i*i,.3-.000003*i*i);d->gyro.push_back(g);}
 for(int i=1;i<=8;++i){double x=.1*i;d->gt.push_back(i*100000000LL);d->poses.emplace_back(Sophus::SO3d::exp(Eigen::Vector3d(.1*x,.2*x*x,.3*x*x*x)),Eigen::Vector3d(x,-.2*x,-1.));}
 for(int frame=1;frame<=6;++frame){int64_t t=frame*100000000LL;d->times.push_back(t);for(int camera=0;camera<2;++camera){auto id=basalt::TimeCamId(t,camera);basalt::CalibInitPoseData p;p.T_a_c=d->poses[frame-1]*opt.calib->T_i_c[camera];p.num_inliers=36;c.calib_init_poses[id]=p;out["input_poses"].push_back({{"time",t},{"camera",camera},{"matrix",matrix(p.T_a_c)}});auto &corners=c.calib_corners[id];int count=frame==2&&camera==0?14:36;for(int k=0;k<count;++k){Eigen::Vector2d pixel;opt.calib->intrinsics[camera].project(Eigen::Vector4d(p.T_a_c.inverse().matrix()*c.april_grid.aprilgrid_corner_pos_3d[k]),pixel);corners.corners.push_back(pixel);corners.corner_ids.push_back(k);corners.radii.push_back(2.);}}}
 c.initOptimization();out["gravity"]={opt.getG().x(),opt.getG().y(),opt.getG().z()};out["rate"]=opt.calib->imu_update_rate;out["min"]=opt.getMinTimeNs();out["max"]=opt.getMaxTimeNs();
 for(auto t:d->times)out["spline_poses"].push_back(matrix(opt.getT_w_i(t)));
 for(const auto &[id,p]:c.reprojected_corners){Json points=Json::array();for(auto v:p.corners_proj)points.push_back({v.x(),v.y()});out["projections"].push_back({{"time",id.frame_id},{"camera",id.cam_id},{"points",points},{"valid",p.corners_proj_success}});}
 opt.mocap_calib->T_i_mark=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.1,-.03,.04)),Eigen::Vector3d(.02,.01,-.03));
 c.initMocap();out["t_w_moc"]=matrix(opt.getT_w_moc());out["t_mark_i"]=matrix(opt.getT_mark_i());out["mocap_initialized"]=opt.mocap_initialized;
 c.recomputeDataLog();for(auto entry:std::vector<std::pair<const char*,pangolin::DataLog*>>{{"imu",&c.imu_data_log},{"poses",&c.pose_data_log},{"mocap",&c.mocap_data_log}}){auto log=entry.second;for(size_t i=0;i<log->Samples();++i){Json row=Json::array();for(size_t k=0;k<log->FirstBlock()->Dimensions();++k)row.push_back(log->Sample(i)[k]);out["diagnostics"][entry.first].push_back(row);}}
 std::map<std::string,double> stats;c.optimizeWithParam(false,&stats);out["stats"]=stats;
 std::ofstream(argv[1])<<out.dump(2)<<'\n';
}
