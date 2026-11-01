// SPDX-License-Identifier: BSD-3-Clause
#define main original_opt_flow_main
#include "/home/zhongjin/Projects/basalt/src/opt_flow.cpp"
#undef main
#include <nlohmann/json.hpp>
int main() {
  for (int frame = 0; frame < 3; ++frame) {
    auto result = std::make_shared<basalt::OpticalFlowResult>();
    result->t_ns = frame == 2 ? 0 : frame;
    result->observations.resize(2);
    result->input_images = std::make_shared<basalt::OpticalFlowInput>();
    for (int camera = 0; camera < 2; ++camera) {
      result->observations[camera][7].setIdentity();
      result->observations[camera][10 + frame + camera].setIdentity();
    }
    observations_queue.push(result);
  }
  observations_queue.push(nullptr);
  std::ostringstream discarded;
  auto old = std::cout.rdbuf(discarded.rdbuf());
  read_result();
  std::cout.rdbuf(old);
  nlohmann::json counts = nlohmann::json::object();
  for (const auto& item : keypoint_stats) counts[std::to_string(item.first)] = item.second;
  bool detached = true;
  for (const auto& item : observations) detached &= !item.second->input_images;
  std::cout << nlohmann::json{{"counts", counts}, {"stored", observations.size()}, {"detached", detached}}.dump() << '\n';
}
