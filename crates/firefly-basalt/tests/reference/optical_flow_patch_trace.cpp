// SPDX-License-Identifier: BSD-3-Clause
// Diagnostic for the first differing room1 stereo track (frame 0, keypoint 6).
// Uses official Basalt 0f3b2b52 and headers aa441ba3, without editing either.
#include <basalt/io/dataset_io.h>
#include <basalt/image/image_pyr.h>
#include <basalt/optical_flow/patch.h>
#include <basalt/optical_flow/patterns.h>
#include <basalt/utils/vio_config.h>
#include <nlohmann/json.hpp>
#include <fstream>
using Json = nlohmann::json;
template<class M> Json matrix(const M& m) {
  Json out=Json::array();
  for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c)out.push_back(m(r,c));
  return out;
}
int main(int argc,char**argv) {
  if(argc!=4)return 1;
  basalt::VioConfig config;config.load(argv[2]);
  auto reader=basalt::DatasetIoFactory::getDatasetIo("bag");reader->read(argv[1]);
  auto data=reader->get_data();auto images=data->get_image_data(data->get_image_timestamps().front());
  basalt::ManagedImagePyr<uint16_t> old(*images[0].img,config.optical_flow_levels),next(*images[1].img,config.optical_flow_levels);
  using Patch=basalt::OpticalFlowPatch<float,basalt::Pattern51<float>>;
  Eigen::Vector2f source(89,195);
  Eigen::AffineCompact2f transform;transform.setIdentity();transform.translation()=source;
  Json output=Json::array();
  for(int level=config.optical_flow_levels;level>=0;--level) {
    float scale=1<<level;transform.translation()/=scale;
    auto image=old.lvl(level);auto target=next.lvl(level);
    Patch patch(image,source/scale);float mean;Patch::VectorP values;Patch::MatrixP3 jac;
    Patch::setDataJacSe2(image,(source/scale).eval(),mean,values,jac);
    Eigen::Matrix3f h=jac.transpose()*jac;
    Eigen::Matrix3f h_inv=Eigen::Matrix3f::Identity();h.ldlt().solveInPlace(h_inv);
    Json pixels=Json::array(),target_pixels=Json::array();
    for(size_t y=0;y<image.h;++y)for(size_t x=0;x<image.w;++x){pixels.push_back(image(x,y));target_pixels.push_back(target(x,y));}
    Json item={{"level",level},{"width",image.w},{"height",image.h},{"pixels",pixels},{"pos",matrix((source/scale).eval())},{"mean",mean},{"data",matrix(values)},{"jac",matrix(jac)},{"hinvjt",matrix(patch.H_se2_inv_J_se2_T)},{"iterations",Json::array()}};
    item["target_pixels"]=target_pixels;
    item["h"]=matrix(h);item["h_inv"]=matrix(h_inv);
    item["arithmetic_profile"]={{"compiler",__VERSION__},{"eigen_simd",Eigen::SimdInstructionSetsInUse()}};
#ifdef EIGEN_VECTORIZE_FMA
    item["arithmetic_profile"]["eigen_fma"]=true;
#else
    item["arithmetic_profile"]["eigen_fma"]=false;
#endif
    for(int iter=0;iter<config.optical_flow_max_iterations;++iter) {
      Patch::Matrix2P pattern=transform.linear().matrix()*Patch::pattern2;
      pattern.colwise()+=transform.translation();
      Patch::VectorP residual;bool valid=patch.residual(target,pattern,residual);
      Eigen::Vector3f increment=-patch.H_se2_inv_J_se2_T*residual;
      auto update=Sophus::SE2f::exp(increment).matrix().eval();
      item["iterations"].push_back({{"transform",matrix(transform.matrix())},{"pattern",matrix(pattern)},{"residual",matrix(residual)},{"increment",matrix(increment)},{"update",matrix(update)},{"valid",valid}});
      if(!valid)return 2;
      transform*=update;
      item["iterations"].back()["after"]=matrix(transform.matrix());
    }
    transform.translation()*=scale;output.push_back(item);
  }
  std::ofstream stream(argv[3]);stream<<output.dump(2)<<'\n';
}
