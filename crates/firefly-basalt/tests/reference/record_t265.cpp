// SPDX-License-Identifier: BSD-3-Clause
#define main original_record_main
#include "/home/zhongjin/Projects/basalt/src/rs_t265_record.cpp"
#undef main
#include <nlohmann/json.hpp>
int main() {
  auto session = std::make_shared<RecordingSession>();
  session->dataset_dir = "./";
  for (int camera=0; camera<2; ++camera) {
    std::string path="mav0/cam"+std::to_string(camera);
    basalt::fs::create_directories(path+"/data");
    session->cam_data[camera].open(path+"/data.csv");
    session->exposure_data[camera].open(path+"/exposure.csv");
  }
  session->imu0_data.open("imu.csv"); session->pose_data.open("pose.csv");
  current_session=session; stop_workers=false;
  std::thread exposure_worker(exposure_save_worker), image_worker(image_save_worker), imu_worker(imu_save_worker), pose_worker(pose_save_worker);
  auto image = std::make_shared<basalt::OpticalFlowInput>(); image->t_ns=123456789; image->img_data.resize(2);
  for (int c=0;c<2;++c) {
    image->img_data[c].exposure=.00125*(c+1);
    image->img_data[c].img = std::make_shared<basalt::ManagedImage<uint16_t>>(16,16);
    for(int i=0;i<256;++i)image->img_data[c].img->ptr[i]=uint16_t(((i*17+c*31)%256)<<8)|uint16_t(i);
  }
  image_data_queue.push(image);
  auto imu=std::make_shared<basalt::ImuData<double>>(); imu->t_ns=123456780; imu->gyro<<.123456789,-.0000123456789,1234567.89; imu->accel<<1.,-0.,9.80665; imu_data_queue.push(imu);
  basalt::RsPoseData pose; pose.t_ns=123456781; pose.data=Sophus::SE3d(Sophus::SO3d::exp(Eigen::Vector3d(.1,-.03,.02)),Eigen::Vector3d(.123456789,1000000.1,-.000001234567)); pose_data_queue.push(pose);
  for(int wait=0;wait<500;++wait){ if(basalt::fs::exists("mav0/cam1/data/123456789.webp") && image_data_queue.empty() && image_data_queue2.empty() && imu_data_queue.empty() && pose_data_queue.empty() && !image_in_flight && !exposure_in_flight && !imu_in_flight && !pose_in_flight)break; std::this_thread::sleep_for(std::chrono::milliseconds(10)); }
  stop_workers=true; exposure_worker.join(); image_worker.join(); imu_worker.join(); pose_worker.join();
  for(int c=0;c<2;++c){session->cam_data[c].close();session->exposure_data[c].close();}session->imu0_data.close();session->pose_data.close();
  auto read=[](const std::string& path){std::ifstream file(path);return std::string(std::istreambuf_iterator<char>(file),{});};
  nlohmann::json output{{"imu",read("imu.csv")},{"pose",read("pose.csv")},{"camera",read("mav0/cam0/data.csv")},{"exposure",read("mav0/cam0/exposure.csv")}};
  for(int quality:{0,90,100,101}){
    cv::Mat source(16,16,CV_8U); for(int i=0;i<256;++i)source.ptr()[i]=(i*17)%256;
    std::vector<uchar> encoded;cv::imencode(".webp",source,encoded,{cv::IMWRITE_WEBP_QUALITY,quality});
    cv::Mat decoded=cv::imdecode(encoded,cv::IMREAD_GRAYSCALE);
    output["images"][std::to_string(quality)]={{"bytes",encoded},{"decoded",std::vector<uint8_t>(decoded.ptr(),decoded.ptr()+256)}};
  }
  std::vector<double> values={0.,-0.,1.23456789,999999.9,1e-4,9.999999e-5,1e-5,-1e8};
  for(double value:values){std::ostringstream text;text<<value;output["scalars"].push_back({value,text.str()});}
  std::cout<<output.dump()<<'\n';
}
