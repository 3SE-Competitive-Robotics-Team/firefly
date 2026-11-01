// SPDX-License-Identifier: BSD-3-Clause
// Original Basalt accumulator.h and its default Eigen 5.0.1 sparse solvers.
#include <basalt/optimization/accumulator.h>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class M>Json vector(const M&m){Json result=Json::array();for(int i=0;i<m.size();++i)result.push_back(m[i]);return result;}
int main(){Json out=Json::array();for(int size:{1,5,18,40}){basalt::SparseHashAccumulator<double>accumulator;accumulator.reset(size);Eigen::Matrix<double,1,1>value;Json additions=Json::array();for(int i=0;i<size;++i){value[0]=4.+i*.125;accumulator.addH<1,1>(i,i,value);additions.push_back({i,i,value[0]});value[0]=(i%7-3)*.25;accumulator.addB<1>(i,value);for(int j=0;j<i;++j)if((i*13+j*7)%11<2){value[0]=.0625;accumulator.addH<1,1>(i,j,value);accumulator.addH<1,1>(j,i,value);additions.push_back({i,j,value[0]});additions.push_back({j,i,value[0]});}}accumulator.setup_solver();Eigen::VectorXd damping=Eigen::VectorXd::Constant(size,.25);Json item={{"size",size},{"additions",additions},{"rhs",vector(accumulator.getB())},{"diagonal",vector(accumulator.Hdiagonal())},{"direct",vector(accumulator.solve(nullptr))},{"damped",vector(accumulator.solve(&damping))}};accumulator.iterative_solver=true;item["cg"]=vector(accumulator.solve(nullptr));item["cg_damped"]=vector(accumulator.solve(&damping));out.push_back(item);}std::cout<<out.dump()<<'\n';}
