// SPDX-License-Identifier: BSD-3-Clause
// Unmodified official DatasetIoFactory, all metadata and sampled pixel values.
#include <basalt/io/dataset_io.h>
#include <nlohmann/json.hpp>
#include <cstring>
#include <fstream>
using Json=nlohmann::json;
struct Hash {
 uint64_t value=14695981039346656037ULL;
 void integer(uint64_t bits){for(int i=0;i<8;++i){value^=(bits>>(i*8))&255;value*=1099511628211ULL;}}
 void scalar(double x){uint64_t bits;std::memcpy(&bits,&x,8);integer(bits);}
};
int main(int argc,char**argv){
 if(argc!=3)return 1;
 auto reader=basalt::DatasetIoFactory::getDatasetIo("bag");reader->read(argv[1]);auto data=reader->get_data();Json result;
 result["cameras"]=data->get_num_cams();result["offset"]=data->get_mocap_to_imu_offset_ns();result["timestamps"]=data->get_image_timestamps();
 for(int kind=0;kind<2;++kind){Hash hash;Json samples=Json::array();size_t count=kind?data->get_accel_data().size():data->get_gyro_data().size();
 for(size_t i=0;i<count;++i){auto timestamp=kind?data->get_accel_data()[i].timestamp_ns:data->get_gyro_data()[i].timestamp_ns;auto vector=kind?data->get_accel_data()[i].data:data->get_gyro_data()[i].data;hash.integer(timestamp);for(int k=0;k<3;++k)hash.scalar(vector[k]);if(i%137==0||i+1==count)samples.push_back({i,timestamp,vector.x(),vector.y(),vector.z()});}
 result[kind?"accel":"gyro"]={{"count",count},{"hash",hash.value},{"samples",samples}};}
 Hash gt_hash;Json gt=Json::array();auto &poses=data->get_gt_pose_data();auto &stamps=data->get_gt_timestamps();
 for(size_t i=0;i<poses.size();++i){gt_hash.integer(stamps[i]);auto t=poses[i].translation();auto q=poses[i].unit_quaternion();for(double x:{t.x(),t.y(),t.z(),q.x(),q.y(),q.z(),q.w()})gt_hash.scalar(x);if(i%137==0||i+1==poses.size())gt.push_back({i,stamps[i],t.x(),t.y(),t.z(),q.x(),q.y(),q.z(),q.w()});}
 result["gt"]={{"count",poses.size()},{"hash",gt_hash.value},{"samples",gt}};
 Json frames=Json::array();auto &timestamps=data->get_image_timestamps();
 for(size_t i=0;i<timestamps.size();++i){if(timestamps.size()>10&&i%97!=0&&i+1!=timestamps.size())continue;Json images=Json::array();
 for(auto &entry:data->get_image_data(timestamps[i])){if(!entry.img){images.push_back(nullptr);continue;}auto &img=*entry.img;Hash hash;uint16_t minimum=65535,maximum=0;Json pixels=Json::array();
 for(size_t y=0;y<img.h;++y)for(size_t x=0;x<img.w;++x){auto value=img(x,y);hash.integer(value);minimum=std::min(minimum,value);maximum=std::max(maximum,value);if((y==0||y+1==img.h)&&(x==0||x+1==img.w))pixels.push_back({x,y,value});}
 pixels.push_back({img.w/2,img.h/2,img(img.w/2,img.h/2)});images.push_back({{"width",img.w},{"height",img.h},{"hash",hash.value},{"minimum",minimum},{"maximum",maximum},{"pixels",pixels},{"exposure",entry.exposure}});}
 frames.push_back({{"index",i},{"timestamp",timestamps[i]},{"images",images}});}
 result["images"]=frames;std::ofstream(argv[2])<<result.dump(2)<<'\n';
}
