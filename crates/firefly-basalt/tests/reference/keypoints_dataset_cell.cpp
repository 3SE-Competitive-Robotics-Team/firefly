// SPDX-License-Identifier: BSD-3-Clause
// Official Basalt detection on the first divergent room1 cell (frame 239).
#include <basalt/io/dataset_io.h>
#include <basalt/utils/keypoints.h>
#include <opencv2/features2d.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <fstream>
using Json=nlohmann::json;
Json points(const std::vector<cv::KeyPoint>& input) {
  Json result=Json::array();
  for(const auto& p:input)result.push_back({p.pt.x,p.pt.y,p.response});
  return result;
}
int main(int argc,char**argv) {
  if(argc!=3)return 1;
  auto reader=basalt::DatasetIoFactory::getDatasetIo("bag");reader->read(argv[1]);
  auto data=reader->get_data();auto input=data->get_image_data(data->get_image_timestamps().at(239));
  const auto& image=*input[0].img;
  basalt::ManagedImage<uint16_t> isolated(512,512);isolated.Fill(0);
  cv::Mat cell(40,40,CV_8U);Json pixels=Json::array();
  for(int y=0;y<40;++y)for(int x=0;x<40;++x) {
    const uint16_t value=image(x+256,y+336);pixels.push_back(value);
    cell.at<uint8_t>(y,x)=value>>8;isolated(x+256,y+336)=value;
  }
  int threshold=40;
  std::vector<cv::KeyPoint> detected;cv::FAST(cell,detected,threshold);
  while(detected.empty() && threshold>5){threshold/=2;cv::FAST(cell,detected,threshold);}
  Json raw=points(detected);
  std::sort(detected.begin(),detected.end(),[](const auto&a,const auto&b){return a.response>b.response;});
  basalt::KeypointsData features;Eigen::aligned_vector<Eigen::Vector2d> occupied;
  basalt::Image<const uint16_t> view(isolated.ptr,isolated.w,isolated.h,isolated.pitch);
  basalt::detectKeypoints(view,features,40,1,occupied);
  Json selected=Json::array();for(const auto& p:features.corners)selected.push_back({p.x(),p.y()});
  Json output={{"frame",239},{"cell_origin",{256,336}},{"cell_size",40},{"threshold",threshold},{"pixels",pixels},{"raw_fast",raw},{"sorted_fast",points(detected)},{"selected",selected},{"compiler",__VERSION__}};
  std::ofstream stream(argv[2]);stream<<output.dump(2)<<'\n';
}
