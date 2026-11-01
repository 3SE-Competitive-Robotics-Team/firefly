// SPDX-License-Identifier: BSD-3-Clause
// Original Basalt 0f3b2b52c807f70ff4e2973ce253c73329eea7bc.
#include <basalt/utils/assert.h>
#include <basalt/vi_estimator/marg_helper.h>
#include <basalt/linearization/block_diagonal.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
using Json=nlohmann::json;
template<class M>Json matrix(const M&m){Json a=Json::array();for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c)a.push_back(m(r,c));return {{"rows",m.rows()},{"cols",m.cols()},{"values",a}};}
template<class S>Json run(){
  using M=Eigen::Matrix<S,Eigen::Dynamic,Eigen::Dynamic>;using V=Eigen::Matrix<S,Eigen::Dynamic,1>;
  Json out;std::set<int>keep{1,4,5},marg{0,2,3};
  for(const std::string name:{"full","rank_deficient","with_prior","zero","near_threshold"}){
    int rows=name=="with_prior"?13:10;M j=M::Zero(rows,6);V r(rows);
    for(int row=0;row<10;++row){r[row]=S((row*7)%11-5)*S(.125);for(int col=0;col<6;++col)j(row,col)=S(((row+1)*(col+3))%17-8)*S(.125)+(row==col?S(1.5):S(0));}
    if(name=="rank_deficient"){j.col(2)=j.col(0)*S(2);j.col(3).setZero();}
    if(name=="with_prior"){j(10,1)=S(1.25);j(11,4)=S(2);j(12,5)=S(.75);r.tail(3)<<S(.25),S(-.5),S(.125);}
    if(name=="zero")j.setZero();
    if(name=="near_threshold"){j.col(0).setZero();j(0,0)=S(1e-4);j.col(2).setZero();j(1,2)=S(1e-9);}
    M h=j.transpose()*j;V b=j.transpose()*r;Json item={{"name",name},{"j",matrix(j)},{"r",matrix(r)},{"h",matrix(h)},{"b",matrix(b)}};
    M output;V rhs;M work=h;V work_b=b;
    basalt::MargHelper<S>::marginalizeHelperSqToSq(work,work_b,keep,marg,output,rhs);item["sq_sq"]={{"h",matrix(output)},{"b",matrix(rhs)},{"consumed",work.size()==0&&work_b.size()==0}};
    work=h;work_b=b;basalt::MargHelper<S>::marginalizeHelperSqToSqrt(work,work_b,keep,marg,output,rhs);item["sq_sqrt"]={{"h",matrix(output)},{"b",matrix(rhs)},{"consumed",work.size()==0&&work_b.size()==0}};
    work=j;work_b=r;basalt::MargHelper<S>::marginalizeHelperSqrtToSqrt(work,work_b,keep,marg,output,rhs);item["sqrt_sqrt"]={{"h",matrix(output)},{"b",matrix(rhs)},{"consumed",work.size()==0&&work_b.size()==0}};
    out["marginalization"].push_back(item);
  }
  for(int mode=0;mode<4;++mode){M a=M::Zero(5,4);for(int r=0;r<5;++r)for(int c=0;c<4;++c)a(r,c)=S((r*3+c*7)%13-6)*S(.25);if(mode==1)a.col(3)=a.col(0)*S(2);if(mode==2)a.setZero();if(mode==3){a.setZero();a(0,0)=S(1);a(1,1)=S(1e-8);a(2,2)=S(1e-16);}
    auto cod=a.completeOrthogonalDecomposition();M inverse=cod.pseudoInverse();out["cod"].push_back({{"input",matrix(a)},{"rank",cod.rank()},{"inverse",matrix(inverse)}});
  }
  basalt::BlockDiagonalAccumulator<S>acc,other;M first(2,2);first<<S(1),S(2),S(3),S(4);acc.add(0,M(first));other.add(0,M::Identity(2,2));other.add(2,M(first*S(2)));acc.join(other);
  V diagonal(6);diagonal<<S(.1),S(.2),S(.3),S(.4),S(.5),S(.6);acc.add_diag(3,2,diagonal);
  V scaling(6);scaling<<S(2),S(-1),S(.5),S(3),S(-2),S(.25);basalt::scale_jacobians(acc.block_diagonal_,3,2,scaling);
  for(int i=0;i<3;++i)out["blocks"].push_back(matrix(acc.block_diagonal_.at(i)));
  out["joined_existing"]=matrix(other.block_diagonal_.at(0));out["joined_moved"]=matrix(other.block_diagonal_.at(2));
  for(int mode=0;mode<4;++mode){M a=M::Zero(5,5);for(int r=0;r<5;++r)for(int c=0;c<5;++c)a(r,c)=S((r*3+c*7)%13-6)*S(.25);a=(a.transpose()*a).eval();if(mode==1){a.row(2).setZero();a.col(2).setZero();}if(mode==2)a=-a;if(mode==3){a.setZero();a.diagonal()<<S(2),S(-3),std::numeric_limits<S>::min(),std::numeric_limits<S>::min()/S(2),S(0);}M b(5,2);for(int r=0;r<5;++r){b(r,0)=S(r+1);b(r,1)=S(2-r);}auto decomposition=a.ldlt();M solution=decomposition.solve(b);out["ldlt"].push_back({{"input",matrix(a)},{"rhs",matrix(b)},{"solution",matrix(solution)},{"diagonal",matrix(decomposition.vectorD())},{"lower",matrix(M(decomposition.matrixL()))}});}
  return out;
}
int main(int argc,char**argv){if(argc!=2)return 2;std::string scalar=argv[1];if(scalar=="f32")std::cout<<run<float>().dump(2);else if(scalar=="f64")std::cout<<run<double>().dump(2);else return 2;std::cout<<'\n';}
