// SPDX-License-Identifier: BSD-3-Clause
// Actual official bag image, raw detector, OpenCV refinement and quality decisions.
#include <basalt/io/dataset_io.h>
#include <basalt/utils/apriltag.h>
#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
int main(int argc,char**argv) {
  if(argc!=5)return 1;
  auto io=basalt::DatasetIoFactory::getDatasetIo("bag");io->read(argv[1]);
  auto source=io->get_data()->get_image_data(std::stoll(argv[2]))[std::stoul(argv[3])].img;
  cv::Mat image(source->h,source->w,CV_8UC1);
  for(size_t y=0;y<source->h;++y)for(size_t x=0;x<source->w;++x)image.at<uint8_t>(y,x)=(*source)(x,y)>>8;
  AprilTags::TagDetector detector(AprilTags::tagCodes36h11,2);
  nlohmann::json result; result["width"]=source->w;result["height"]=source->h;
  result["pixels"]=std::vector<uint8_t>(image.data,image.data+image.total());result["tags"]=nlohmann::json::array();
  for(const auto& tag:detector.extractTags(image)) {
    std::vector<cv::Point2f> refined; nlohmann::json initial=nlohmann::json::array();
    for(auto p:tag.p){refined.emplace_back(p.first,p.second);initial.push_back({p.first,p.second});}
    double radius=std::max(2.,.075*tag.observedPerimeter/4.-1.);int window=std::ceil(radius+1.);
    cv::cornerSubPix(image,refined,cv::Size(window,window),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,.01));
    nlohmann::json points=nlohmann::json::array();for(auto p:refined)points.push_back({p.x,p.y});
    result["tags"].push_back({{"id",tag.id},{"good",tag.good},{"perimeter",tag.observedPerimeter},{"radius",radius},{"window",window},{"initial",initial},{"refined",points}});
  }
  std::ofstream(argv[4])<<result.dump();
}
