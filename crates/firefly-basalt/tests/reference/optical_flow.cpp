// SPDX-License-Identifier: BSD-3-Clause
// Execute the pinned upstream implementation; no Rust algorithm is used here.
#include <basalt/optical_flow/patch.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using Json = nlohmann::json;

template <class S> Json scalar(S x) {
  if (std::isnan(x)) return "NaN";
  if (std::isinf(x)) return x > 0 ? "+Inf" : "-Inf";
  return x;
}
template <class M> Json matrix(const M& m) {
  Json values = Json::array();
  for (int r = 0; r < m.rows(); ++r)
    for (int c = 0; c < m.cols(); ++c) values.push_back(scalar(m(r, c)));
  return {{"rows", m.rows()}, {"cols", m.cols()}, {"row_major", values}};
}

template <class S, class Pattern> Json sample(const char* name, const std::string& scenario) {
  using Patch = basalt::OpticalFlowPatch<S, Pattern>;
  constexpr int width = 48, height = 40, stride = 52;
  std::vector<uint16_t> storage(stride * height, 65535);
  for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
    auto value = 1000 + (17*x*x + 31*y*y + 7*x*y) % 4000;
    if (scenario == "black") value = 0;
    if (scenario == "constant") value = 400;
    if (scenario == "plane") value = 900 + 17*x + 23*y;
    storage[y*stride+x] = value;
  }
  basalt::Image<const uint16_t> image(storage.data(), width, height, stride*sizeof(uint16_t));
  typename Patch::Vector2 position(S(22.25), S(18.5));
  if (scenario == "edge") position << S(3.25), S(4.5);
  if (scenario == "outside") position << S(-20), S(-20);
  S mean = 0, jacobian_mean = 0, warped_mean = 0;
  typename Patch::VectorP data, jacobian_data, warped_data;
  typename Patch::MatrixP3 j;
  Patch::setData(image,position,mean,data);
  Patch::setDataJacSe2(image,position,jacobian_mean,jacobian_data,j);
  typename Patch::Matrix3 hessian = j.transpose() * j;
  typename Patch::Matrix3 hessian_inverse = Patch::Matrix3::Identity();
  hessian.ldlt().solveInPlace(hessian_inverse);
  Sophus::SE2<S> warp(S(0.03),typename Patch::Vector2(S(0.1),S(-0.2)));
  Patch::setData(image,position,warped_mean,warped_data,&warp);
  Patch patch(image,position);
  typename Patch::Matrix2P transformed = Patch::pattern2;
  transformed.colwise() += position + typename Patch::Vector2(S(0.125),S(-0.25));
  typename Patch::VectorP residual;
  const bool residual_valid = patch.residual(image,transformed,residual);
  return {{"scalar",sizeof(S)==4?"f32":"f64"}, {"pattern",name}, {"scenario",scenario},
    {"width",width},{"height",height},{"stride",stride},{"position",matrix(position)},
    {"pattern2",matrix(Patch::pattern2)}, {"set_data_mean",scalar(mean)}, {"set_data",matrix(data)},
    {"set_data_jac_mean",scalar(jacobian_mean)}, {"set_data_jac_data",matrix(jacobian_data)},
    {"jacobian",matrix(j)}, {"hessian",matrix(hessian)}, {"hessian_inverse",matrix(hessian_inverse)},
    {"warped_mean",scalar(warped_mean)}, {"warped_data",matrix(warped_data)},
    {"mean",scalar(patch.mean)},{"data",matrix(patch.data)},
    {"h_inv_jt",matrix(patch.H_se2_inv_J_se2_T)}, {"valid",patch.valid},
    {"transformed_pattern",matrix(transformed)}, {"residual",matrix(residual)}, {"residual_valid",residual_valid}};
}
template<class S> void cases(Json& output, const std::string& selected) {
  for(const auto& scenario: {"normal","edge","outside","black","constant","plane"}) {
    if (selected != scenario) continue;
    output.push_back(sample<S,basalt::Pattern24<S>>("24",scenario));
    output.push_back(sample<S,basalt::Pattern52<S>>("52",scenario));
    output.push_back(sample<S,basalt::Pattern51<S>>("51",scenario));
    output.push_back(sample<S,basalt::Pattern50<S>>("50",scenario));
  }
}
int main(int argc,char** argv) {
  Json output = Json::array();
  if (argc != 3) return 2;
  if (std::string(argv[1]) == "f32") cases<float>(output,argv[2]);
  else if (std::string(argv[1]) == "f64") cases<double>(output,argv[2]);
  else return 2;
  std::cout << output.dump(2) << '\n';
}
