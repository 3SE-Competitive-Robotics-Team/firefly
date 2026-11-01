// SPDX-License-Identifier: BSD-3-Clause
// Numeric oracle compiled against basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/image/image.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>

using Json = nlohmann::json;
template<class S> Json cubic_gradient(const basalt::Image<const uint16_t>& image, const Eigen::Matrix<S,2,1>& p) {
  if constexpr (std::is_same_v<S, double>) {
    auto g = image.template interpGradCubicSplines<S>(p);
    return Json::array({g[0],g[1],g[2]});
  } else return "unsupported_upstream_float_instantiation";
}
template<class M> Json vector(const M& m) {
  Json result = Json::array();
  for(int i=0;i<m.size();++i) result.push_back(m[i]);
  return result;
}
template<class S> Json sample() {
  constexpr int width=7,height=6,stride=10;
  std::vector<uint16_t> data(stride*height,65535);
  for(int y=0;y<height;++y) for(int x=0;x<width;++x) data[y*stride+x]=100+13*x*x+17*y*y+5*x*y;
  basalt::Image<const uint16_t> image(data.data(),width,height,stride*sizeof(uint16_t));
  Json samples=Json::array();
  for (Eigen::Matrix<S,2,1> p: {Eigen::Matrix<S,2,1>(S(1),S(1)),{S(2.25),S(2.5)},{S(4.5),S(3.125)}}) {
    samples.push_back({{"position",vector(p)}, {"interp",image.template interp<S>(p)},
      {"central_gradient",vector(image.template interpGrad<S>(p))},
      {"exact_gradient",vector(image.template interpGradBilinearExact<S>(p))},
      {"cubic",image.template interpCubicSplines<S>(p)},
      {"cubic_gradient",cubic_gradient(image,p)}});
  }
  Json borders=Json::array();
  for(Eigen::Matrix<S,2,1> p: {Eigen::Matrix<S,2,1>(S(0),S(0)),{S(0.25),S(0.5)},{S(5.999),S(4.999)}}) {
    borders.push_back({{"position",vector(p)},{"in_bounds_0",image.InBounds(p,0)},
      {"in_bounds_1",image.InBounds(p,1)},{"in_bounds_2",image.InBounds(p,2)},
      {"cubic",image.template interpCubicSplines<S>(p)},
      {"cubic_gradient",cubic_gradient(image,p)}});
  }
  return {{"scalar",sizeof(S)==4?"f32":"f64"},{"width",width},{"height",height},{"stride",stride},{"pixels",data},{"samples",samples},{"borders",borders}};
}
int main() { std::cout << Json::array({sample<float>(),sample<double>()}).dump(2) << '\n'; }
