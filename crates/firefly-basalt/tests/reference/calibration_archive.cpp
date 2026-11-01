// SPDX-License-Identifier: BSD-3-Clause
// Actual original Cereal overloads, little-endian 64-bit native archive.
#include <basalt/calibration/calibration_helper.h>
#include <basalt/serialization/headers_serialization.h>
#include <cereal/archives/binary.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iomanip>
template<class T>std::string bytes(T&v){std::ostringstream s;{cereal::BinaryOutputArchive a(s);a(v);}std::ostringstream out;for(unsigned char c:s.str())out<<std::hex<<std::setw(2)<<std::setfill('0')<<unsigned(c);return out.str();}
int main(int argc,char**argv){if(argc!=2)return 1;
basalt::CalibCornerData c;c.corners={{1.25,-2.5},{0.,1024.125}};c.corner_ids={-1,321};c.radii={0.,2.75};
basalt::CalibInitPoseData p;p.T_a_c=Sophus::SE3d(Eigen::Quaterniond(.5,.5,.5,.5),Eigen::Vector3d(1.5,-2.,.125));p.num_inliers=17;p.reprojected_corners={{12.5,-6.25},{3.,4.}};
basalt::CalibCornerData empty;basalt::CalibInitPoseData empty_pose;empty_pose.num_inliers=0;
nlohmann::json result={{"corners",bytes(c)},{"pose",bytes(p)},{"empty_corners",bytes(empty)},{"empty_pose",bytes(empty_pose)}};
basalt::CalibCornerMap corners;corners.emplace(basalt::TimeCamId(123,1),c);basalt::CalibInitPoseMap poses;poses.emplace(basalt::TimeCamId(123,1),p);result["corner_map"]=bytes(corners);result["pose_map"]=bytes(poses);
std::ofstream(argv[1])<<result.dump(2)<<'\n';}
