// SPDX-License-Identifier: MPL-2.0
// Independent Eigen 5.0.1 AMD reference, used by the official Basalt default sparse solver.
#include <Eigen/Sparse>
#include <nlohmann/json.hpp>
#include <iostream>
int main(){nlohmann::json out=nlohmann::json::array();for(int size:{0,1,2,5,18,40,100})for(int mode=0;mode<4;++mode){std::vector<Eigen::Triplet<double>>entries;for(int i=0;i<size;++i){entries.emplace_back(i,i,1);for(int j=0;j<i;++j)if((mode==0&&i==j+1)||(mode==1&&(i*13+j*7)%23<3)||(mode==2&&(i%7==j%7))||(mode==3&&(i<3||j<3))){entries.emplace_back(i,j,1);entries.emplace_back(j,i,1);}}Eigen::SparseMatrix<double>matrix(size,size);matrix.setFromTriplets(entries.begin(),entries.end());nlohmann::json item;item["outer"]=std::vector<int>(matrix.outerIndexPtr(),matrix.outerIndexPtr()+size+1);item["inner"]=std::vector<int>(matrix.innerIndexPtr(),matrix.innerIndexPtr()+matrix.nonZeros());Eigen::PermutationMatrix<Eigen::Dynamic>permutation;Eigen::internal::minimum_degree_ordering(matrix,permutation);item["permutation"]=std::vector<int>(permutation.indices().data(),permutation.indices().data()+size);out.push_back(item);}std::cout<<out.dump()<<'\n';}
