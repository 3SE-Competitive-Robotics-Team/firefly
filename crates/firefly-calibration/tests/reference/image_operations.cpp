// SPDX-License-Identifier: BSD-3-Clause
// Execute official basalt-headers aa441ba3 without modifying upstream code.
#include <basalt/image/image.h>
#include <nlohmann/json.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>
using Json=nlohmann::json;
Json value(double x) { if(std::isnan(x))return "NaN"; if(std::isinf(x))return x>0?"+Inf":"-Inf"; return x; }
template<class M> Json vector(const M& m) { Json a=Json::array();for(int i=0;i<m.size();++i)a.push_back(value(m[i]));return a; }
int main() {
  Json out;
  std::vector<double> pixels(30,987654321.0);
  for(int y=0;y<5;++y)for(int x=0;x<5;++x)pixels[y*6+x]=1000000000.125+0.177*x*x+0.299*y*y+0.043*x*y;
  basalt::Image<const double> image(pixels.data(),5,5,6*sizeof(double));
  Eigen::Vector2f p(1.371f,1.719f);
  Json bits=Json::array();for(double x:pixels){uint64_t b;std::memcpy(&b,&x,sizeof(b));bits.push_back(b);}
  out["mixed"]={{"pixels",pixels},{"position",vector(p)},{"interp",value(image.interp(p))},
    {"pixels_bits",bits},{"gradient",vector(image.interpGrad(p))},{"exact_gradient",vector(image.interpGradBilinearExact(p))}};
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for(std::vector<double> samples: {std::vector<double>{nan,2.,3.},{2.,nan,3.},{2.,3.,nan},{0.,-0.},{-0.,0.}}) {
    basalt::Image<double> view(samples.data(),samples.size(),1,samples.size()*sizeof(double));
    auto limits=view.MinMax();
    Json input=Json::array();for(double x:samples)input.push_back(value(x));
    out["min_max"].push_back({{"pixels",input},{"min",value(limits.first)},{"max",value(limits.second)},
      {"min_sign",std::signbit(limits.first)},{"max_sign",std::signbit(limits.second)}});
  }
  std::cout<<out.dump(2)<<'\n';
}
