"""Compile the pinned upstream L-BFGS to generate cancellation-order fixtures."""

import argparse
from pathlib import Path
import subprocess

REVISION = "9d85475ea7b9bf5c112cf7c3c3d0d3f9e96d9010"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--upstream", type=Path, default=Path.home() / "Projects/EGO-Planner-v2")
args = parser.parse_args()
assert subprocess.check_output(["git", "-C", str(args.upstream), "rev-parse", "HEAD"], text=True).strip() == REVISION
include = args.upstream / "swarm-playground/main_ws/src/planner/traj_opt/include/optimizer"
source = r'''
#include "lbfgs.hpp"
#include <iostream>
#include <iomanip>
#include <limits>
struct Case { int mode, calls=0, polls=0; };
double evaluate(void *data, const double *x, double *g, int) {
  Case &test=*static_cast<Case*>(data);
  ++test.calls;
  if (test.mode==3) { g[0]=0; return 0; }
  g[0]=2*x[0];
  if (test.calls==2 && (test.mode==0 || test.mode==2)) {
    g[0]=2;
    return test.mode==0 ? 100 : std::numeric_limits<double>::infinity();
  }
  return x[0]*x[0];
}
int progress(void *data, const double*, const double*, double, double, double, double, int, int, int) {
  Case &test=*static_cast<Case*>(data);
  ++test.polls;
  return test.mode==3 || test.mode==4 || test.calls>=2;
}
int main() {
  std::cout<<std::setprecision(17);
  for (int mode=0;mode<6;++mode) {
    Case test{mode};
    lbfgs::lbfgs_parameter_t config;
    lbfgs::lbfgs_load_default_parameters(&config);
    config.mem_size=16; config.max_iterations=200; config.min_step=1e-32;
    config.g_epsilon=1e-5; config.past=3; config.delta=1e-2;
    double x=mode==1?2:1, cost=0;
    int code=lbfgs::lbfgs_optimize(1,&x,&cost,evaluate,nullptr,progress,&test,&config);
    std::cout<<mode<<' '<<code<<' '<<test.calls<<' '<<test.polls<<' '<<x<<' '<<cost<<'\n';
  }
}
'''
root = Path(__file__).resolve().parents[4]
build = root / "target/ego-lbfgs-reference"
build.mkdir(parents=True, exist_ok=True)
cpp = build / "reference.cpp"
cpp.write_text(source)
binary = build / "reference"
subprocess.run(["c++", "-std=c++17", "-O2", "-I" + str(include), str(cpp), "-o", str(binary)], check=True)
values = subprocess.check_output([str(binary)], text=True)
Path(__file__).with_name("ego_stop.txt").write_text("# EGO-Planner-v2 " + REVISION + "\n" + values)
