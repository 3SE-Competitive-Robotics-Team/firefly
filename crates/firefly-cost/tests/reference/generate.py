"""Generate the C++ oracle fixture from the pinned EGO-Planner-v2 checkout.

Usage: python3 crates/firefly-cost/tests/reference/generate.py --eigen <Eigen include root>
The checkout's actual MINCO and cost functions are compiled, without ROS.
Generated build files live in target/ego-cost-reference; only numeric fixtures are committed.
"""

import argparse
from pathlib import Path
import subprocess

REVISION = "9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--upstream", type=Path, default=Path.home() / "Projects/EGO-Planner-v2")
parser.add_argument("--eigen", type=Path, required=True)
args = parser.parse_args()
assert subprocess.check_output(["git", "-C", str(args.upstream), "rev-parse", "HEAD"], text=True).strip() == REVISION
include = args.upstream / "swarm-playground/main_ws/src/planner/traj_opt/include/optimizer"
source = (include.parent.parent / "src/poly_traj_optimizer.cpp").read_text()


def function(name):
    marker = "PolyTrajOptimizer::" + name + "("
    start = source.rfind("\n", 0, source.index(marker)) + 1
    if name in {"addPVAJGradCost2CT", "VirtualTGradCost", "RealT2VirtualT"}:
        start = source.rfind("  template", 0, start)
    brace = source.index("{", source.index(marker))
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end].replace("PolyTrajOptimizer::", "")


functions = "\n".join(function(name) for name in [
    "addPVAJGradCost2CT", "obstacleGradCostP", "swarmGradCostP",
    "feasibilityGradCostV", "feasibilityGradCostA", "feasibilityGradCostJ",
    "distanceSqrVarianceWithGradCost2p", "VirtualTGradCost", "RealT2VirtualT",
])
header = r'''
#include "poly_traj_utils.hpp"
#include <iomanip>
struct ConstraintPoints {
  Eigen::MatrixXd points;
  std::vector<std::vector<Eigen::Vector3d>> base_point, direction;
  static int two_thirds_id(Eigen::MatrixXd &points, bool touch_goal) {
    return touch_goal ? points.cols()-1 : points.cols()-1-(points.cols()-2)/3;
  }
};
struct Peer {
  int drone_id = 1;
  double start_time = 0.15, duration = 1.4, des_clearance = 0.45;
  poly_traj::Trajectory traj;
};
struct PolyTrajOptimizer {
  poly_traj::MinJerkOpt jerkOpt_;
  ConstraintPoints cps_;
  bool touch_goal_ = false;
  int drone_id_ = 0;
  double t_now_ = 0, wei_time_ = 0.4;
  double obs_clearance_ = 0.1, obs_clearance_soft_ = 0.5;
  double wei_obs_ = 0, wei_obs_soft_ = 0, wei_swarm_mod_ = 0, wei_feas_ = 0, wei_sqrvar_ = 0;
  double swarm_clearance_ = 0.5, max_vel_ = 1.2, max_acc_ = 2.0, max_jer_ = 4.0;
  std::vector<Peer> peers;
  std::vector<Peer> *swarm_trajs_ = &peers;
  std::vector<double> min_ellip_dist2_;
'''
main = r'''
};
void dump(const Eigen::MatrixXd &matrix) {
  for (int row=0; row<matrix.rows(); ++row)
    for (int col=0; col<matrix.cols(); ++col) std::cout << matrix(row,col) << ' ';
  std::cout << '\n';
}
int main() {
  std::cout << std::setprecision(17);
  Eigen::Matrix3d start, end;
  start << 0,0.5,0.1, 0,-0.2,0, 0,0.1,-0.1;
  end << 4,0.2,0, 2,0.1,-0.1, 1,-0.1,0;
  Eigen::MatrixXd waypoints(3,2);
  waypoints << 1.5,2.8, 0.8,1.5, 0.3,0.6;
  Eigen::VectorXd durations(3); durations << 0.8,1.0,1.2;
  Eigen::Matrix3d peer_start = Eigen::Matrix3d::Zero(), peer_end = Eigen::Matrix3d::Zero();
  peer_start.col(0) << 1.8,0.9,0.4;
  peer_start.col(1) << 0.2,-0.1,0.05;
  peer_end.col(0) << 2.1,1.1,0.5;
  peer_end.col(1) << 0.3,0.1,-0.05;
  poly_traj::MinJerkOpt peer_opt;
  peer_opt.reset(peer_start,peer_end,1);
  Eigen::VectorXd peer_time(1); peer_time << 1.4;
  peer_opt.generate(Eigen::MatrixXd(3,0),peer_time);
  for (int variant=0; variant<3; ++variant) {
    PolyTrajOptimizer opt;
    opt.touch_goal_ = variant == 1;
    if (variant == 2) {
      opt.max_vel_ = opt.max_acc_ = opt.max_jer_ = 1e9;
      opt.obs_clearance_ = opt.obs_clearance_soft_ = -100;
    }
    opt.peers.push_back(Peer()); opt.peers[0].traj=peer_opt.getTraj();
    opt.min_ellip_dist2_.resize(1,1e100);
    opt.cps_.points.resize(3,16);
    opt.cps_.base_point.resize(16); opt.cps_.direction.resize(16);
    for (int index=0; index<16; ++index) {
      if (index%4 == 0) continue;
      opt.cps_.base_point[index] = {Eigen::Vector3d(1,0.3,0.2), Eigen::Vector3d(0,1.6,0)};
      opt.cps_.direction[index] = {Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,-1,0)};
    }
    opt.jerkOpt_.reset(start,end,3); opt.jerkOpt_.generate(waypoints,durations);
    std::cout << "TERMS\n";
    dump(opt.jerkOpt_.get_b());
    for (int term=0; term<5; ++term) {
      opt.wei_obs_=term==1?2.3:0; opt.wei_obs_soft_=term==1?0.7:0;
      opt.wei_swarm_mod_=term==2?1.7:0; opt.wei_feas_=term==3?0.3:0; opt.wei_sqrvar_=term==4?0.8:0;
      double cost=0;
      Eigen::VectorXd grad_time(3);
      opt.jerkOpt_.initGradCost(grad_time,cost);
      if (term==0) {
        cost*=1.2; grad_time*=1.2; opt.jerkOpt_.get_gdC()*=1.2;
      } else {
        cost=0; grad_time.setZero(); opt.jerkOpt_.get_gdC().setZero();
        Eigen::VectorXd costs(4);
        opt.addPVAJGradCost2CT(grad_time,costs,5);
        cost=costs.sum();
      }
      std::cout << cost << '\n';
      dump(opt.jerkOpt_.get_gdC()); dump(grad_time);
      Eigen::MatrixXd grad_points(3,2);
      opt.jerkOpt_.getGrad2TP(grad_time,grad_points);
      dump(grad_points); dump(grad_time);
    }
    std::cout << "CALLBACK\n";
    opt.wei_obs_=2.3; opt.wei_obs_soft_=0.7; opt.wei_swarm_mod_=1.7;
    opt.wei_feas_=0.3; opt.wei_sqrvar_=0.8;
    opt.jerkOpt_.generate(waypoints,durations);
    double smoothness=0;
    Eigen::VectorXd grad_time(3);
    opt.jerkOpt_.initGradCost(grad_time,smoothness);
    smoothness*=1.2; grad_time*=1.2; opt.jerkOpt_.get_gdC()*=1.2;
    Eigen::VectorXd costs(4);
    opt.addPVAJGradCost2CT(grad_time,costs,5);
    Eigen::MatrixXd grad_points(3,2);
    opt.jerkOpt_.getGrad2TP(grad_time,grad_points);
    Eigen::VectorXd virtual_time(3), grad_virtual_time(3);
    opt.RealT2VirtualT(durations,virtual_time);
    double time_cost=0;
    opt.VirtualTGradCost(durations,virtual_time,grad_time,grad_virtual_time,time_cost);
    std::cout << smoothness+costs.sum()+time_cost << '\n';
    dump(grad_points); dump(grad_virtual_time); dump(opt.cps_.points);
  }
}
'''
root = Path(__file__).resolve().parents[4]
build = root / "target/ego-cost-reference"
build.mkdir(parents=True, exist_ok=True)
cpp = build / "reference.cpp"
cpp.write_text(header + functions + main)
binary = build / "reference"
subprocess.run(["c++", "-std=c++17", "-O2", "-I" + str(include), "-I" + str(args.eigen), str(cpp), "-o", str(binary)], check=True)
values = subprocess.check_output([str(binary)], text=True)
outputs = {"TERMS": [], "CALLBACK": []}
section = None
for line in values.splitlines():
    if line in outputs:
        section = line
    else:
        outputs[section].append(line.rstrip())
for section, filename in [("TERMS", "ego_cost.txt"), ("CALLBACK", "ego_callback.txt")]:
    Path(__file__).with_name(filename).write_text("# EGO-Planner-v2 " + REVISION + "\n" + "\n".join(outputs[section]) + "\n")
