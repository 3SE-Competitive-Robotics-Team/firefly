// SPDX-License-Identifier: BSD-3-Clause
// Run the unchanged original main, then export its retained visualization
// states. Their latest poses must equal its independently collected state positions.
#define main original_vio_sim_main
#include "/home/zhongjin/Projects/basalt/src/vio_sim.cpp"
#undef main
int main(int argc, char** argv) {
  const int status = original_vio_sim_main(argc, argv);
  if (status) return status;
  if (vio_t_w_i.size() != gt_frame_t_ns.size()) return 2;
  std::ofstream output("trajectory.txt");
  output << std::setprecision(17);
  for (size_t i = 0; i < gt_frame_t_ns.size(); ++i) {
    const auto& pose = vis_map.at(gt_frame_t_ns[i])->states.back();
    if ((pose.translation().array() != vio_t_w_i[i].array()).any()) return 3;
    const auto& p = pose.translation();
    const auto& q = pose.unit_quaternion();
    output << gt_frame_t_ns[i] * 1e-9 << " " << p.x() << " " << p.y() << " " << p.z()
           << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << '\n';
  }
  return output.good() ? 0 : 4;
}
