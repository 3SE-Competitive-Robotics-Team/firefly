// SPDX-License-Identifier: BSD-3-Clause
// Execute the unmodified official tracker and its original dependencies.
#include <basalt/optical_flow/frame_to_frame_optical_flow.h>
#include <basalt/optical_flow/patch_optical_flow.h>
#include <basalt/optical_flow/multiscale_frame_to_frame_optical_flow.h>
#include <src/utils/keypoints.cpp>
#include <src/utils/vio_config.cpp>
#include <nlohmann/json.hpp>
#include <iostream>

template<class Tracker>nlohmann::json run(int cameras) {
  basalt::VioConfig config;
  config.optical_flow_skip_frames=2;
  basalt::Calibration<double> calibration;
  for(int i=0;i<cameras;++i) {
    auto cam=basalt::GenericCamera<double>::fromString("pinhole");
    cam.setFromInit(Eigen::Vector4d(200,200,128,96));
    calibration.intrinsics.push_back(cam);
    calibration.T_i_c.emplace_back(Sophus::SO3d(),Eigen::Vector3d(i*0.1,0,0));
  }
  Tracker tracker(config,calibration);
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> output;
  tracker.output_queue=&output;
  nlohmann::json frames=nlohmann::json::array();
  for(int frame=0;frame<3;++frame) {
    auto input=std::make_shared<basalt::OpticalFlowInput>();
    input->t_ns=1000000000+frame*20000000;
    for(int camera=0;camera<cameras;++camera) {
      basalt::ImageData image;
      image.img=std::make_shared<basalt::ManagedImage<uint16_t>>(256,192);
      for(int y=0;y<192;++y)for(int x=0;x<256;++x) {
        double px=x-frame-camera;
        (*image.img)(x,y)=30000+10000*std::sin(.13*px)+8000*std::cos(.17*y)+6000*std::sin(.11*px+.07*y);
      }
      input->img_data.push_back(image);
    }
    tracker.processFrame(input->t_ns,input);
    if(frame%2==0) {
      basalt::OpticalFlowResult::Ptr result;output.pop(result);
      nlohmann::json observations=nlohmann::json::array();
      for(auto&cam:result->observations) {
        nlohmann::json points=nlohmann::json::array();
        for(auto&[id,t]:cam) {
          nlohmann::json values=nlohmann::json::array();
          for(int r=0;r<2;++r)for(int c=0;c<3;++c)values.push_back(t.matrix()(r,c));
          points.push_back({{"id",id},{"transform",values}});
        }
        observations.push_back(points);
      }
      frames.push_back({{"timestamp",result->t_ns},{"observations",observations},{"pyramid_levels",result->pyramid_levels}});
    }
  }
  tracker.input_queue.push(nullptr);
  // Wait for the worker sentinel before the queue is destroyed.
  basalt::OpticalFlowResult::Ptr sentinel;output.pop(sentinel);
  return frames;
}
int main(int argc,char**argv) {
  int cameras=argc>1?std::stoi(argv[1]):1;
  std::string mode=argc>2?argv[2]:"frame_to_frame";
  nlohmann::json output;
  if(mode=="patch")output=run<basalt::PatchOpticalFlow<float,basalt::Pattern51>>(cameras);
  else if(mode=="multiscale_frame_to_frame")output=run<basalt::MultiscaleFrameToFrameOpticalFlow<float,basalt::Pattern51>>(cameras);
  else output=run<basalt::FrameToFrameOpticalFlow<float,basalt::Pattern51>>(cameras);
  std::cout<<output.dump(2)<<"\n";
}
