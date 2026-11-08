// SPDX-License-Identifier: GPL-3.0-only
// Harness for verbatim EGO-Planner-v2 functions inserted by generate.py.
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <vector>
#include <plan_env/raycast.h>

#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false
#define ROS_ERROR(...) ((void)0)
#define ROS_WARN(...) ((void)0)
using namespace std;

namespace ros {
struct Duration {
  double value;
  double toSec() const { return value; }
};
struct Time {
  double value = 0;
  static inline double clock = 0;
  static Time now() { return Time{clock}; }
  double toSec() const { return value; }
  void fromSec(double v) { value = v; }
  Duration operator-(Time other) const { return Duration{value - other.value}; }
};
struct TimerEvent {};
}

// Image storage only; projection is the unmodified upstream function.
namespace cv {
struct Mat {
  int rows = 0, cols = 0;
  vector<uint16_t> data;
  template<typename T> T *ptr(int row) {
    static_assert(sizeof(T) == sizeof(uint16_t));
    return data.data() + row * cols;
  }
  template<typename T> T &at(int row, int col) {
    static_assert(sizeof(T) == sizeof(uint16_t));
    return data[row * cols + col];
  }
};
}

// UPSTREAM_STRUCTS

struct GridMap {
  MappingParameters mp_;
  MappingData md_;
  static constexpr int INVALID_IDX = -10000;

  // UPSTREAM_METHODS

  void setup(int variant) {
    mp_.resolution_ = 0.5;
    mp_.resolution_inv_ = 1.0 / mp_.resolution_;
    mp_.local_update_range3d_ = Eigen::Vector3d(2., 2., 1.5);
    mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.resolution_inv_).array().ceil().cast<int>();
    mp_.obstacles_inflation_ = 0.5;
    mp_.inf_grid_ = ceil((mp_.obstacles_inflation_ - 1e-5) / mp_.resolution_);
    mp_.enable_virtual_walll_ = true;
    mp_.virtual_ground_ = -0.1;
    mp_.virtual_ceil_ = 2.0;
    mp_.fx_ = 2.; mp_.fy_ = 3.; mp_.cx_ = 1.; mp_.cy_ = 1.;
    mp_.use_depth_filter_ = variant == 1;
    mp_.depth_filter_margin_ = variant == 1 ? 1 : 0;
    mp_.depth_filter_mindist_ = 0.2;
    mp_.depth_filter_tolerance_ = 0.15;
    mp_.k_depth_scaling_factor_ = 1000.;
    mp_.skip_pixel_ = variant == 2 ? 2 : 1;
    mp_.p_hit_ = 0.70; mp_.p_miss_ = 0.35;
    mp_.p_min_ = 0.12; mp_.p_max_ = 0.97; mp_.p_occ_ = 0.80;
    auto logit = [](double p) { return log(p / (1. - p)); };
    mp_.prob_hit_log_ = logit(mp_.p_hit_);
    mp_.prob_miss_log_ = logit(mp_.p_miss_);
    mp_.clamp_min_log_ = logit(mp_.p_min_);
    mp_.clamp_max_log_ = logit(mp_.p_max_);
    mp_.min_occupancy_log_ = logit(mp_.p_occ_);
    mp_.odom_depth_timeout_ = 1.;
    mp_.fading_time_ = 0.25;
    mp_.min_ray_length_ = 0.1;
    mp_.show_occ_time_ = false;
    md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;
    md_.ringbuffer_inf_size3i_ = md_.ringbuffer_size3i_ + Eigen::Vector3i::Constant(2 * mp_.inf_grid_);
    md_.ringbuffer_origin3i_.setZero();
    md_.ringbuffer_inf_origin3i_.setZero();
    int capacity = md_.ringbuffer_size3i_.prod();
    md_.occupancy_buffer_.assign(capacity, mp_.clamp_min_log_);
    md_.occupancy_buffer_inflate_.assign(md_.ringbuffer_inf_size3i_.prod(), 0);
    md_.count_hit_.assign(capacity, 0);
    md_.count_hit_and_miss_.assign(capacity, 0);
    md_.flag_traverse_.assign(capacity, -1);
    md_.flag_rayend_.assign(capacity, -1);
    md_.cache_voxel_.resize(capacity);
    md_.cache_voxel_cnt_ = 0;
    md_.proj_points_.resize(16);
    md_.proj_points_cnt_ = 0;
    md_.raycast_num_ = 0;
    md_.occ_need_update_ = false;
    md_.has_first_depth_ = false;
    md_.has_odom_ = false;
    md_.flag_have_ever_received_depth_ = false;
    md_.flag_depth_odom_timeout_ = false;
    md_.last_occ_update_time_.fromSec(0);
    md_.camera_pos_.setZero();
    md_.camera_r_m_.setIdentity();
    md_.last_camera_r_m_.setIdentity();
    md_.depth_image_.rows = md_.depth_image_.cols = 4;
    md_.depth_image_.data.resize(16);
  }

  void dump() {
    cout << mp_.have_initialized_ << ' ' << md_.flag_depth_odom_timeout_ << ' ' << md_.proj_points_cnt_ << ' ';
    for (int i = 0; i < md_.proj_points_cnt_; ++i)
      for (int axis = 0; axis < 3; ++axis) cout << md_.proj_points_[i][axis] << ' ';
    for (double v : md_.occupancy_buffer_) cout << v << ' ';
    for (uint16_t v : md_.occupancy_buffer_inflate_) cout << v << ' ';
    if (mp_.have_initialized_) {
      for (int axis = 0; axis < 3; ++axis) cout << md_.ringbuffer_lowbound3i_[axis] << ' ';
      for (int axis = 0; axis < 3; ++axis) cout << md_.ringbuffer_upbound3i_[axis] << ' ';
      for (Eigen::Vector3d point : {Eigen::Vector3d(0.,0.,0.), Eigen::Vector3d(0.25,0.25,1.25),
             md_.ringbuffer_upbound3d_, md_.ringbuffer_lowbound3d_, Eigen::Vector3d(100.,0.,3.)})
        cout << getOccupancy(point) << ' ' << getInflateOccupancy(point) << ' ';
    }
    cout << '\n';
  }
};

int main() {
  cout << setprecision(17);
  int variant;
  if (!(cin >> variant)) return 1;
  GridMap map;
  map.setup(variant);
  char operation;
  while (cin >> operation) {
    if (operation == 'D') {
      double x, y, z, angle;
      cin >> ros::Time::clock >> x >> y >> z >> angle;
      map.md_.camera_pos_ = Eigen::Vector3d(x,y,z);
      map.md_.camera_r_m_ = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      for (auto &pixel : map.md_.depth_image_.data) cin >> pixel;
      map.md_.occ_need_update_ = true;
      map.md_.flag_have_ever_received_depth_ = true;
      map.updateOccupancyCallback(ros::TimerEvent{});
    } else if (operation == 'T') {
      cin >> ros::Time::clock;
      map.updateOccupancyCallback(ros::TimerEvent{});
    } else if (operation == 'F') {
      map.fadingCallback(ros::TimerEvent{});
    } else return 2;
    map.dump();
  }
}
