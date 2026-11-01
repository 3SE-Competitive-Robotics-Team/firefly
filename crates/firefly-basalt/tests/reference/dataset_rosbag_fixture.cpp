// SPDX-License-Identifier: BSD-3-Clause
// Independent ROS writer from the official release build: none, bz2, lz4.
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <filesystem>
#include <array>
int main(int argc,char**argv){
 if(argc!=2)return 1;
 const std::array<const char*,3> names{"none","bz2","lz4"};
 const std::array<rosbag::CompressionType,3> modes{rosbag::compression::Uncompressed,rosbag::compression::BZ2,rosbag::compression::LZ4};
 for(int mode=0;mode<3;++mode){
 rosbag::Bag bag((std::filesystem::path(argv[1])/ (std::string("dataset_rosbag_")+names[mode]+".bag")).string(),rosbag::bagmode::Write);bag.setCompression(modes[mode]);bag.setChunkThreshold(300);
 for(int frame=0;frame<5;++frame){
 ros::Time timestamp(100,frame*1000000),arrival(99,frame*1000000);
 sensor_msgs::Imu imu;imu.header.stamp=timestamp;imu.angular_velocity.x=frame*.125;imu.angular_velocity.y=-1.5;imu.angular_velocity.z=2.75;imu.linear_acceleration.x=-9.81;imu.linear_acceleration.y=frame*.5;imu.linear_acceleration.z=.001;bag.write("/imu",arrival,imu);
 geometry_msgs::PoseStamped pose;pose.header.stamp=ros::Time(98,frame*1000000);pose.pose.position.x=frame*.25;pose.pose.position.y=-.2;pose.pose.position.z=1.;pose.pose.orientation.w=1.;bag.write("/pose",arrival,pose);
 if(frame>=3)continue;
 for(int camera=1;camera>=0;--camera){sensor_msgs::Image image;image.header.stamp=timestamp;image.header.frame_id=frame==1?"camera":"12345ns";image.width=4;image.height=2;image.encoding=camera?"mono16":"mono8";image.is_bigendian=frame==2;image.step=4*(camera?2:1);
 if(camera){for(uint16_t value:{uint16_t(0),uint16_t(1),uint16_t(255),uint16_t(256),uint16_t(32767),uint16_t(32768),uint16_t(65534),uint16_t(65535)}){image.data.push_back(value&255);image.data.push_back(value>>8);}}
 else image.data={0,1,127,128,254,255,16,240};bag.write(camera?"/camZ":"/camA",arrival,image);}
 }bag.close();}
}
