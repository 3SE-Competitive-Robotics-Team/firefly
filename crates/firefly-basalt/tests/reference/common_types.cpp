// SPDX-License-Identifier: BSD-3-Clause
// Executes official Basalt common_types serializers and hash specializations.
#include <basalt/utils/common_types.h>
#include <basalt/serialization/eigen_io.h>
#include <cereal/types/vector.hpp>
#include <cereal/types/bitset.hpp>
#include <cereal/types/utility.hpp>
#include <iostream>
#include <sstream>

int main() {
  basalt::TimeCamId id(-123, 2);
  basalt::KeypointsData points;
  points.corners = {{1.25, -2.5}, {4., 8.}};
  points.corner_angles = {0.5, -0.75};
  points.corner_descriptors.resize(3);
  points.corner_descriptors[0].set(0);
  points.corner_descriptors[1].set(255);
  points.corner_descriptors[2].set();
  basalt::MatchData matches;
  matches.T_i_j = Sophus::SE3d(Eigen::Quaterniond(0.5,0.5,0.5,0.5),Eigen::Vector3d(1,2,3));
  matches.matches = {{-1,2},{3,4}};
  matches.inliers = {{3,4}};
  std::ostringstream stream;
  { cereal::BinaryOutputArchive binary(stream); binary(id,points,matches); }
  std::string hex;
  const char* digits = "0123456789abcdef";
  for (unsigned char byte : stream.str()) {
    hex += digits[byte >> 4]; hex += digits[byte & 15];
  }
  std::size_t hash = std::hash<basalt::TimeCamId>{}(id);
  std::size_t pair_hash = std::hash<std::pair<basalt::TimeCamId,basalt::TimeCamId>>{}({id,{456,1}});
  cereal::JSONOutputArchive json(std::cout);
  json(cereal::make_nvp("id",id),cereal::make_nvp("points",points),cereal::make_nvp("matches",matches),cereal::make_nvp("binary",hex),cereal::make_nvp("hash",hash),cereal::make_nvp("pair_hash",pair_hash));
}
