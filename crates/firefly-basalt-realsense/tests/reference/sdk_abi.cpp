// SPDX-License-Identifier: BSD-3-Clause
// Compile with SDK 2.51.1 headers: c++ -std=c++17 -fsyntax-only sdk_abi.cpp.
// The Rust counterpart checks exactly these consumed realsense-sys 2.44 layouts.
#include <librealsense2/rs.h>
#include <cstddef>
static_assert(RS2_API_VERSION / 100 == 251);
static_assert(sizeof(rs2_intrinsics)==48 && alignof(rs2_intrinsics)==4);
static_assert(sizeof(rs2_motion_device_intrinsic)==72 && alignof(rs2_motion_device_intrinsic)==4);
static_assert(sizeof(rs2_extrinsics)==48 && alignof(rs2_extrinsics)==4);
static_assert(sizeof(rs2_pose)==84 && alignof(rs2_pose)==4);
static_assert(offsetof(rs2_pose,rotation)==36 && offsetof(rs2_pose,tracker_confidence)==76);
static_assert(offsetof(rs2_motion_device_intrinsic,noise_variances)==48 && offsetof(rs2_motion_device_intrinsic,bias_variances)==60);
static_assert(offsetof(rs2_intrinsics,coeffs)==28 && offsetof(rs2_extrinsics,translation)==36);
static_assert(RS2_EXTENSION_VIDEO_FRAME==8 && RS2_EXTENSION_MOTION_FRAME==9 && RS2_EXTENSION_COMPOSITE_FRAME==10 && RS2_EXTENSION_POSE_FRAME==20);
static_assert(RS2_STREAM_FISHEYE==4 && RS2_STREAM_GYRO==5 && RS2_STREAM_ACCEL==6 && RS2_STREAM_POSE==8);
static_assert(RS2_FORMAT_Y8==9 && RS2_FORMAT_MOTION_XYZ32F==16 && RS2_FORMAT_6DOF==18);
static_assert(RS2_FRAME_METADATA_ACTUAL_EXPOSURE==3 && RS2_OPTION_EXPOSURE==3 && RS2_OPTION_ENABLE_AUTO_EXPOSURE==10);
