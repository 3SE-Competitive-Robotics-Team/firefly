// SPDX-License-Identifier: BSD-3-Clause
// Oracle using the unmodified upstream Basalt headers.
#include <basalt/utils/tracks.h>
#include <basalt/hash_bow/hash_bow.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <iostream>
using Json = nlohmann::json;
Json state(const UnionFind& u) { return {{"parent",u.m_cc_parent},{"rank",u.m_cc_rank},{"size",u.m_cc_size}}; }
int main() {
  Json out;
  UnionFind uf; uf.InitSets(8);
  out["union_find"].push_back(state(uf));
  for(auto [a,b]: {std::pair{0,1},{2,3},{1,3},{4,5},{6,7},{5,7},{3,7},{0,7}}) {
    uf.Union(a,b); out["union_find"].push_back(state(uf));
  }
  for(int i=0;i<8;++i) uf.Find(i);
  out["compressed"]=state(uf);
  uf.InitSets(4); out["reinitialized"]=state(uf);
  basalt::HashBow<256> bow(20);
  std::vector<std::bitset<256>> descriptors(4);
  descriptors[1].set();
  for(int i=0;i<256;++i) if(i%3==0) descriptors[2].set(i);
  descriptors[3]=descriptors[2];
  std::vector<basalt::FeatureHash> hashes;
  basalt::HashBowVector words;
  bow.compute_bow(descriptors,hashes,words);
  for(auto h:hashes) out["hashes"].push_back(h.to_ulong());
  std::sort(words.begin(),words.end(),[](auto a,auto b){return a.first.to_ulong()<b.first.to_ulong();});
  for(auto [h,w]:words) out["bow"].push_back({h.to_ulong(),w});
  bow.add_to_database({10,0},words);
  basalt::HashBowVector single{{hashes[0],1.0}};
  bow.add_to_database({20,1},single);
  for(int cutoff: {0,20,21}) {
    std::vector<std::pair<basalt::TimeCamId,double>> results;
    int64_t time=cutoff;
    bow.querry_database(words,10,results,cutoff?&time:nullptr);
    std::sort(results.begin(),results.end(),[](auto a,auto b){return a.first<b.first;});
    Json rows=Json::array();
    for(auto [id,score]:results) rows.push_back({id.frame_id,id.cam_id,score});
    out["query"].push_back({{"cutoff",cutoff},{"results",rows}});
  }
  // One image pair avoids hash-container iteration affecting root identifiers.
  basalt::Matches matches;
  matches[{{100,0},{200,1}}].inliers={{0,10},{1,11},{2,12},{3,12}};
  basalt::TrackBuilder tracks;
  tracks.Build(matches);
  out["tracks_before"]=state(tracks.uf_tree);
  out["filter_return"]=tracks.Filter();
  out["tracks_after"]=state(tracks.uf_tree);
  out["track_count"]=tracks.TrackCount();
  basalt::FeatureTracks exported;
  tracks.Export(exported);
  std::map<basalt::TrackId,basalt::FeatureTrack> sorted(exported.begin(),exported.end());
  out["tracks"]=Json::array();
  for(const auto& [id,features]:sorted) {
    Json rows=Json::array();
    for(auto [image,feature]:features) rows.push_back({image.frame_id,image.cam_id,feature});
    out["tracks"].push_back({{"id",id},{"features",rows}});
  }
  std::cout<<out.dump(2)<<'\n';
}
