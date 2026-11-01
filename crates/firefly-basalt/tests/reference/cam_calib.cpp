// SPDX-License-Identifier: BSD-3-Clause
// Calls the unmodified original CamCalib object; private exposure is probe-only.
#include <pangolin/pangolin.h>
#include <pangolin/display/image_view.h>
#include <basalt/calibration/calibration_helper.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/optimization/poses_optimize.h>
#include <basalt/utils/test_utils.h>
#include <fstream>
#include <nlohmann/json.hpp>
#define private public
#include <basalt/calibration/cam_calib.h>
#undef private
using Json=nlohmann::json;
class Dataset:public basalt::VioDataset {
 public:std::vector<int64_t> times{100,200,300};Eigen::aligned_vector<basalt::AccelData> accel;Eigen::aligned_vector<basalt::GyroData> gyro;std::vector<int64_t> gt;Eigen::aligned_vector<Sophus::SE3d> poses;
 size_t get_num_cams()const override{return 3;}std::vector<int64_t>&get_image_timestamps()override{return times;}const Eigen::aligned_vector<basalt::AccelData>&get_accel_data()const override{return accel;}const Eigen::aligned_vector<basalt::GyroData>&get_gyro_data()const override{return gyro;}const std::vector<int64_t>&get_gt_timestamps()const override{return gt;}const Eigen::aligned_vector<Sophus::SE3d>&get_gt_pose_data()const override{return poses;}int64_t get_mocap_to_imu_offset_ns()const override{return 0;}std::vector<basalt::ImageData>get_image_data(int64_t)override{return std::vector<basalt::ImageData>(3);}
};
Json matrix(const Sophus::SE3d &pose){Json result=Json::array();auto m=pose.matrix();for(int r=0;r<4;++r)for(int c=0;c<4;++c)result.push_back(m(r,c));return result;}
int main(int argc,char**argv){if(argc!=4)return 1;
 {std::ofstream grid(argv[2]);grid<<R"({"tagCols":3,"tagRows":3,"tagSize":0.04,"tagSpacing":0.3})";}
 basalt::CamCalib controller("","bag",argv[2],argv[3],"cam_calib_probe",1,{"pinhole","pinhole","pinhole"},false);
 controller.vio_dataset=std::make_shared<Dataset>();controller.calib_opt=std::make_shared<basalt::PosesOptimization>();controller.calib_opt->resetCalib(3,{"pinhole","pinhole","pinhole"});
 for(auto &camera:controller.calib_opt->calib->intrinsics)camera.setFromInit(Eigen::Vector4d(350,370,320,240));
 for(int frame=0;frame<3;++frame){int64_t time=(frame+1)*100;for(int camera=0;camera<3;++camera){
 auto id=basalt::TimeCamId(time,camera);basalt::CalibInitPoseData init;init.T_a_c=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.02*frame,-.03*camera,.01)),Eigen::Vector3d(.1*camera,-.02*frame,-1.-.1*frame));init.num_inliers=frame==0?(camera==2?15:20):frame==1?(camera==0?14:25):(camera==2?14:30);controller.calib_init_poses[id]=init;
 auto &corners=controller.calib_corners[id];auto transform=init.T_a_c.inverse().matrix();int k=0;for(const auto &point:controller.april_grid.aprilgrid_corner_pos_3d){Eigen::Vector2d projected;controller.calib_opt->calib->intrinsics[camera].project(Eigen::Vector4d(transform*point),projected);corners.corners.push_back(projected+Eigen::Vector2d(.1,-.2));corners.corner_ids.push_back(k++);corners.radii.push_back(2.);}
 }}
 controller.initCamExtrinsics();Json output;for(auto &pose:controller.calib_opt->calib->T_i_c)output["extrinsics"].push_back(matrix(pose));
 controller.initOptimization();for(int64_t time:controller.vio_dataset->get_image_timestamps())output["poses"].push_back(matrix(controller.calib_opt->getT_w_i(time)));
 for(const auto &[id,projection]:controller.reprojected_corners){Json points=Json::array();for(auto p:projection.corners_proj)points.push_back({p.x(),p.y()});output["projections"].push_back({{"time",id.frame_id},{"camera",id.cam_id},{"points",points},{"valid",projection.corners_proj_success}});}
 std::map<std::string,double> stats;controller.optimizeWithParam(false,&stats);output["stats"]=stats;
 std::ofstream(argv[1])<<output.dump(2)<<'\n';std::remove(argv[2]);
}
