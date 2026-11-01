// SPDX-License-Identifier: BSD-3-Clause
// Calls the unmodified official readers on the same on-disk test input as Rust.
#include <basalt/io/dataset_io_euroc.h>
#include <basalt/io/dataset_io_kitti.h>
#include <basalt/io/dataset_io_uzh.h>
#include <nlohmann/json.hpp>
#include <iostream>
using nlohmann::json;
int main(int argc, char** argv) {
  if (argc != 3) return 2;
  std::unique_ptr<basalt::DatasetIoInterface> reader;
  const std::string type(argv[1]);
  if (type == "euroc") reader.reset(new basalt::EurocIO(false));
  else if (type == "mocap") reader.reset(new basalt::EurocIO(true));
  else if (type == "uzh") reader.reset(new basalt::UzhIO);
  else if (type == "kitti") reader.reset(new basalt::KittiIO);
  else return 3;
  reader->read(argv[2]); auto data=reader->get_data(); json result;
  result["timestamps"]=data->get_image_timestamps();
  result["gt_timestamps"]=data->get_gt_timestamps();
  result["gyro"]=json::array(); result["accel"]=json::array();
  for (const auto& m:data->get_gyro_data()) result["gyro"].push_back({m.timestamp_ns,m.data.x(),m.data.y(),m.data.z()});
  for (const auto& m:data->get_accel_data()) result["accel"].push_back({m.timestamp_ns,m.data.x(),m.data.y(),m.data.z()});
  result["poses"]=json::array();
  for (const auto& p:data->get_gt_pose_data()) {
    auto t=p.translation();auto q=p.unit_quaternion();
    result["poses"].push_back({t.x(),t.y(),t.z(),q.x(),q.y(),q.z(),q.w()});
  }
  result["images"]=json::array();
  for (auto stamp:data->get_image_timestamps()) {
    json frame=json::array();
    for (const auto& image:data->get_image_data(stamp)) {
      if (!image.img) {frame.push_back(nullptr);continue;}
      std::vector<uint16_t> pixels(image.img->ptr,image.img->ptr+image.img->size());
      frame.push_back({{"width",image.img->w},{"height",image.img->h},{"pixels",pixels},{"exposure",image.exposure}});
    }
    result["images"].push_back(frame);
  }
  std::cout << "REFERENCE_JSON=" << result.dump() << '\n';
}
