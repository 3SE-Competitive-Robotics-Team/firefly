// SPDX-License-Identifier: BSD-3-Clause
// Official Basalt 0f3b2b52, headers aa441ba3, locked Eigen 5 / cereal 1.3.2.
#include <basalt/io/marg_data_io.h>
#include <basalt/serialization/headers_serialization.h>
// Serialization overloads reside in this official translation unit.
#include "src/io/marg_data_io.cpp"
#include <fstream>

int main(int argc, char** argv) {
  if (argc != 3) return 1;
  basalt::MargData m;
  m.aom.total_size=21;m.aom.items=2;
  m.aom.abs_order_map.emplace(100,std::make_pair(0,6));
  m.aom.abs_order_map.emplace(200,std::make_pair(6,15));
  m.abs_H.resize(2,3);m.abs_H<<1,2,3,-4,-5,-6;
  m.abs_b.resize(2);m.abs_b<<0.125,-1.5;
  for(int64_t t : {100,200}) {
    Sophus::SE3d pose(Sophus::SO3d::exp(Eigen::Vector3d(.1,-.2,.3)),Eigen::Vector3d(1,2,3));
    basalt::PoseStateWithLin<double> p(t,pose,true);
    Eigen::Matrix<double,6,1> delta;delta<<.01,-.02,.03,.001,.002,-.003;p.applyInc(delta);
    m.frame_poses.emplace(t,p);
    basalt::PoseVelBiasState<double> state(t,pose,Eigen::Vector3d(4,5,6),Eigen::Vector3d(.1,.2,.3),Eigen::Vector3d(.4,.5,.6));
    basalt::PoseVelBiasStateWithLin<double> s(state);s.setLinTrue();
    Eigen::Matrix<double,15,1> d;for(int i=0;i<15;++i)d[i]=.001*(i+1);s.applyInc(d);
    m.frame_states.emplace(t,s);
  }
  m.kfs_all={100,200};m.kfs_to_marg={100};m.use_imu=true;
  {std::ofstream stream(argv[1],std::ios::binary);cereal::BinaryOutputArchive a(stream);a(m);}
  auto flow=std::make_shared<basalt::OpticalFlowResult>();flow->t_ns=100;flow->observations.resize(2);
  Eigen::AffineCompact2f affine;affine.matrix()<<1,.25,10,-.5,2,20;
  flow->observations[0].emplace(7,affine);flow->observations[1].emplace(9,affine);
  flow->input_images=std::make_shared<basalt::OpticalFlowInput>();flow->input_images->t_ns=101;
  auto image=std::make_shared<basalt::ManagedImage<uint16_t>>(3,2);
  for(size_t i=0;i<6;++i)image->ptr[i]=i*11111;
  basalt::ImageData data;data.exposure=.005;data.img=image;
  flow->input_images->img_data={data,data,basalt::ImageData()};
  {std::ofstream stream(argv[2],std::ios::binary);cereal::BinaryOutputArchive a(stream);a(flow,flow);}
}
