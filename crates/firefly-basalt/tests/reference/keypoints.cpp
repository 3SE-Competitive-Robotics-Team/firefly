// SPDX-License-Identifier: BSD-3-Clause
// Executes the complete official Basalt source without editing its functions.
#include <src/utils/keypoints.cpp>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
Json run(const std::string& scenario) {
  const int w=128,h=96;
  const int stride=scenario=="pitched"?132:w;
  std::vector<uint16_t> pixels(stride*h,65535);
  for(int y=0;y<h;++y)for(int x=0;x<w;++x) {
    int v=1000+(17*x*x+31*y*y+7*x*y)%64000;
    if(scenario=="checker")v=((x/8+y/8)%2)?60000:3000;
    if(scenario=="black")v=0;
    pixels[y*stride+x]=v;
  }
  basalt::Image<const uint16_t> image(pixels.data(),w,h,stride*sizeof(uint16_t));
  basalt::KeypointsData kd;
  Eigen::aligned_vector<Eigen::Vector2d> occupied;
  if(scenario=="occupied")occupied.emplace_back(45.2,44.7);
  basalt::detectKeypoints(image,kd,32,2,occupied);
  Json corners=Json::array(); for(auto&p:kd.corners)corners.push_back({p.x(),p.y()});
  basalt::detectKeypointsMapping(image,kd,20);
  Json mapping=Json::array();for(auto&p:kd.corners)mapping.push_back({p.x(),p.y()});
  kd.corners={Eigen::Vector2d(31.25,30.75),Eigen::Vector2d(53.8,46.2),Eigen::Vector2d(81,62)};
  basalt::computeAngles(image,kd,true);
  basalt::computeDescriptors(image,kd);
  Json descriptors=Json::array();for(auto&d:kd.corner_descriptors) {
    Json words=Json::array();for(int word=0;word<4;++word) {
      uint64_t v=0;for(int bit=0;bit<64;++bit)if(d[word*64+bit])v|=uint64_t(1)<<bit;
      words.push_back(v);
    }descriptors.push_back(words);
  }
  std::vector<std::bitset<256>> a={std::bitset<256>(0),std::bitset<256>(3),std::bitset<256>(3),std::bitset<256>(255)};
  std::vector<std::bitset<256>> b={std::bitset<256>(3),std::bitset<256>(0),std::bitset<256>(127)};
  std::vector<std::pair<int,int>> matches;
  basalt::matchDescriptors(a,b,matches,10,1.2);
  std::sort(matches.begin(),matches.end());
  std::vector<std::pair<int,int>> empty_matches;
  basalt::matchDescriptors(a,{},empty_matches,501,1.0);
  return {{"scenario",scenario},{"corners",corners},{"mapping",mapping},{"angles",kd.corner_angles},{"descriptors",descriptors},{"matches",matches},{"empty_matches",empty_matches}};
}
int main(int argc,char**argv){std::cout<<run(argc>1?argv[1]:"normal").dump(2)<<"\n";}
