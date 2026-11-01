// SPDX-License-Identifier: BSD-3-Clause
// Independent unmodified ETH/Basalt detector probe.
#include <basalt/utils/apriltag.h>
#include <apriltags/TagDetector.h>
#include <apriltags/Tag36h11.h>
#include <nlohmann/json.hpp>
#include <fstream>
using Json=nlohmann::json;
int main(int argc,char**argv){
if(argc!=2)return 1;
Json output;
for(int test=0;test<4;++test){
const int width=240,height=240;
cv::Mat image(height,width,CV_8UC1,cv::Scalar(255));
for(int tag=0;tag<4;++tag){
int ox=30+(tag%2)*100,oy=30+(tag/2)*100;int id=tag+test*4;
for(int y=0;y<80;++y)for(int x=0;x<80;++x){
int cx=x/8,cy=y/8;uint8_t value=0;
if(cx>=2&&cx<8&&cy>=2&&cy<8){int bit=35-((cy-2)*6+cx-2);value=(AprilTags::t36h11[id]&(1ULL<<bit))?255:0;}
image.at<uint8_t>(oy+y,ox+x)=value;
}}
if(test==1)cv::GaussianBlur(image,image,cv::Size(3,3),.7);
if(test==2)cv::rotate(image,image,cv::ROTATE_90_CLOCKWISE);
if(test==3)image.setTo(128);
basalt::ManagedImage<uint16_t> input(width,height);
for(int y=0;y<height;++y)for(int x=0;x<width;++x)input(x,y)=uint16_t(image.at<uint8_t>(y,x))<<8;
AprilTags::TagDetector raw(AprilTags::tagCodes36h11,2);auto detections=raw.extractTags(image);
std::sort(detections.begin(),detections.end(),[](auto&a,auto&b){return a.id<b.id;});
Json result;result["raw"]=Json::array();for(auto&d:detections){Json p=Json::array();for(auto c:d.p)p.push_back({c.first,c.second});auto pose=d.getRelativeTransform(.16,350.,370.,120.,120.);Json transform=Json::array();for(int i=0;i<4;++i)for(int j=0;j<4;++j)transform.push_back(pose(i,j));result["raw"].push_back({{"id",d.id},{"points",p},{"hamming",d.hammingDistance},{"perimeter",d.observedPerimeter},{"pose",transform}});}
basalt::ApriltagDetector detector(16);Eigen::aligned_vector<Eigen::Vector2d> good,bad;std::vector<int>ids,rejected;std::vector<double>radii,rejected_radii;
detector.detectTags(input,good,ids,radii,bad,rejected,rejected_radii);
result["ids"]=ids;result["rejected_ids"]=rejected;result["radii"]=radii;result["rejected_radii"]=rejected_radii;
result["points"]=Json::array();for(auto&p:good)result["points"].push_back({p.x(),p.y()});result["rejected_points"]=Json::array();for(auto&p:bad)result["rejected_points"].push_back({p.x(),p.y()});
result["pixels"]=std::vector<uint8_t>(image.data,image.data+width*height);
if(test==0){
for(int view=0;view<4;++view){
std::vector<cv::Point3f> obj{{-.08f,-.08f,0},{.08f,-.08f,0},{.08f,.08f,0},{-.08f,.08f,0}};
cv::Vec3d rv(.3+view*.1,-.2+view*.08,.1),tv(.04,-.03,.6+view*.2);
std::vector<cv::Point2f> image_points;cv::projectPoints(obj,rv,tv,cv::Matx33f(350,0,120,0,370,120,0,0,1),cv::Vec4f(0,0,0,0),image_points);
AprilTags::TagDetection tag;Json input=Json::array();for(int i=0;i<4;++i){image_points[i].x+=float(view)*.03f*(i-1);tag.p[i]={image_points[i].x,image_points[i].y};input.push_back({tag.p[i].first,tag.p[i].second});}
auto pose=tag.getRelativeTransform(.16,350,370,120,120);Json transform=Json::array();for(int i=0;i<4;++i)for(int j=0;j<4;++j)transform.push_back(pose(i,j));result["pose_cases"].push_back({{"points",input},{"pose",transform}});
}}
if(test==0){
cv::Mat patch(24,24,CV_8UC1);for(int y=0;y<24;++y)for(int x=0;x<24;++x)patch.at<uint8_t>(y,x)=(x*17+y*23+(x*y)%97)%256;
std::vector<cv::Point2f> initial{{0.2f,0.3f},{23.1f,22.9f},{12.0f,11.25f},{3.2f,20.9f},{8.5f,8.5f}};
auto refined=initial;cv::cornerSubPix(patch,refined,cv::Size(3,3),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,100,.01));
Json sub;sub["pixels"]=std::vector<uint8_t>(patch.data,patch.data+24*24);
for(auto p:initial)sub["initial"].push_back({p.x,p.y});for(auto p:refined)sub["refined"].push_back({p.x,p.y});result["subpixel"]=sub;
}
output.push_back(result);
}
std::ofstream(argv[1])<<output.dump()<<'\n';
}
