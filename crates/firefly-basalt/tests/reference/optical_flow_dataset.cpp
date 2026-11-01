// SPDX-License-Identifier: BSD-3-Clause
// Calls the unmodified official dataset reader and optical-flow factory.
#include <basalt/io/dataset_io.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/vio_config.h>
#include <nlohmann/json.hpp>
#include <fstream>
#ifdef BASALT_FLOW_DIRECT
#include <basalt/optical_flow/frame_to_frame_optical_flow.h>
#endif
int main(int argc,char**argv) {
  if(argc!=6)return 1;
  basalt::VioConfig config;config.load(argv[2]);
  basalt::Calibration<double> calib;
  {std::ifstream input(argv[3]);cereal::JSONInputArchive archive(input);archive(calib);}
  auto reader=basalt::DatasetIoFactory::getDatasetIo("bag");reader->read(argv[1]);auto data=reader->get_data();
  tbb::concurrent_bounded_queue<basalt::OpticalFlowResult::Ptr> output;
#ifdef BASALT_FLOW_DIRECT
  auto flow=std::make_shared<basalt::FrameToFrameOpticalFlow<float,basalt::Pattern51>>(config,calib);
#else
  auto flow=basalt::OpticalFlowFactory::getOpticalFlow(config,calib);
#endif
  flow->output_queue=&output;
  nlohmann::json frames=nlohmann::json::array();
  const size_t count=std::stoul(argv[5]);
  for(size_t frame=0;frame<count;++frame){
    auto timestamp=data->get_image_timestamps().at(frame);
    auto input=std::make_shared<basalt::OpticalFlowInput>();input->t_ns=timestamp;input->img_data=data->get_image_data(timestamp);
    nlohmann::json images=nlohmann::json::array();
    for(auto &camera:input->img_data){uint64_t hash=14695981039346656037ULL;auto &img=*camera.img;for(size_t y=0;y<img.h;++y)for(size_t x=0;x<img.w;++x){hash^=img(x,y);hash*=1099511628211ULL;}images.push_back({{"width",img.w},{"height",img.h},{"hash",hash},{"exposure",camera.exposure}});}
    flow->input_queue.push(input);basalt::OpticalFlowResult::Ptr result;output.pop(result);
    nlohmann::json cameras=nlohmann::json::array();
    for(auto &camera:result->observations){nlohmann::json points=nlohmann::json::array();for(auto &[id,affine]:camera){auto &m=affine.matrix();points.push_back({{"id",id},{"matrix",{m(0,0),m(0,1),m(0,2),m(1,0),m(1,1),m(1,2)}}});}cameras.push_back(points);}
    frames.push_back({{"timestamp",timestamp},{"images",images},{"cameras",cameras}});
  }
  flow->input_queue.push(nullptr);basalt::OpticalFlowResult::Ptr end;output.pop(end);flow.reset();
  std::ofstream out(argv[4]);out<<frames.dump(2)<<'\n';
}
