// SPDX-License-Identifier: BSD-3-Clause
// Official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/spline/rd_spline.h>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class M> Json values(const M& m){
  Json out=Json::array();for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c)out.push_back(m(r,c));return out;
}
template<int D,class Curve> Json eval(const Curve& curve,int64_t time){
  typename Curve::JacobianStruct j;auto value=curve.template evaluate<D>(time,&j);
  return {{"time",time},{"derivative",D},{"value",values(value)},{"start",j.start_idx},{"jacobian",j.d_val_d_knot}};
}
template<int D,class Curve> Json integral(const Curve& curve){
  typename Curve::IntegralJacobianStruct j;
  auto value=curve.template evaluateIntegralSquared<D>(&j);Json rows=Json::array();
  for(const auto& knot:j.d_int_d_knot)rows.push_back(values(knot));
  return {{"derivative",D},{"value",value},{"jacobian",rows}};
}
template<class S,int N> Json run(){
  using Curve=basalt::RdSpline<3,N,S>;
  Curve curve(200000000,123456789);
  for(int i=0;i<N+3;++i)curve.knotsPushBack(typename Curve::VecD(S(i*.25),S((i%3)-1),S(i*i*.125)));
  Json out={{"blend",values(basalt::computeBlendingMatrix<N,S>())},{"cumulative",values(basalt::computeBlendingMatrix<N,S,true>())},
      {"bezier_blend",values(basalt::computeBlendingMatrixBezier<N,S>())},{"base",values(basalt::computeBaseCoefficients<N,S>())}};
  for(const auto& q:basalt::computeQuadraticCoefficients<N,S>())out["quadratic"].push_back(values(q));
  for(const auto& knot:curve.getKnots())out["knots"].push_back(values(knot));
  out["min"]=curve.minTimeNs();out["max"]=curve.maxTimeNs();
  for(int64_t time:{curve.minTimeNs(),curve.minTimeNs()+91234567,curve.minTimeNs()+200000000,curve.maxTimeNs()}){
    out["evaluate"].push_back(eval<0>(curve,time));out["evaluate"].push_back(eval<1>(curve,time));out["evaluate"].push_back(eval<2>(curve,time));
  }
  if constexpr(std::is_same_v<S,double>){
    out["integral"]={integral<0>(curve),integral<1>(curve),integral<2>(curve),integral<N>(curve)};
    for(int start=0;start<4;++start){
      auto segment=curve.getSegmentBezierCurve(start);Json b;
      for(int i=0;i<N;++i)b["knots"].push_back(values(segment.getKnot(i)));
      for(int64_t time:{segment.minTimeNs(),segment.minTimeNs()+91234567,segment.maxTimeNs()}){
        b["evaluate"].push_back(eval<0>(segment,time));b["evaluate"].push_back(eval<1>(segment,time));b["evaluate"].push_back(eval<2>(segment,time));
      }
      b["integral"]={integral<0>(segment),integral<1>(segment),integral<2>(segment)};out["bezier"].push_back(b);
    }
  }
  curve.knotsPopFront();curve.knotsPopBack();out["popped_min"]=curve.minTimeNs();out["popped_max"]=curve.maxTimeNs();
  out["popped_front"]=values(curve.knotsFront());
  return out;
}
int main(){
  Json out={{"f64_n4",run<double,4>()},{"f64_n5",run<double,5>()},{"f32_n4",run<float,4>()},{"f32_n5",run<float,5>()}};
  std::cout<<out.dump(2)<<'\n';
}
