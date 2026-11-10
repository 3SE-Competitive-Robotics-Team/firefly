// Numerical oracle using unmodified RootBA d3900037fc4bc98328e5d8d26f1c4eed922f88cc.
// Build with that checkout's src/, Eigen and glog; this file owns only the fixture.
#include <iomanip>
#include <iostream>
#include <sstream>
#include "rootba/cg/conjugate_gradient.hpp"
#include "sophus/common.hpp"

template <class S> struct Operator : rootba::LinearOperator<S> {
  using Vec = typename rootba::LinearOperator<S>::VecX;
  Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic> a;
  size_t num_cols() const override { return 12; }
  Vec right_multiply(const Vec& x) const override {
    return a.transpose() * (a * x) + S(0.01) * x;
  }
};
template <class S> struct Preconditioner : rootba::Preconditioner<S> {
  using Vec = typename rootba::Preconditioner<S>::VecX;
  Eigen::Matrix<S, 6, 6> inverse[2];
  void solve_assign(const Vec& b, Vec& x) const override {
    for (int i = 0; i < 2; ++i) x.template segment<6>(6*i) = inverse[i] * b.template segment<6>(6*i);
  }
};
template <class S> void run(const char* precision) {
  Operator<S> op;
  op.a.resize(18, 12);
  typename Operator<S>::Vec r(18);
  for (int i = 0; i < 18; ++i) {
    r[i] = S(std::cos(i * 5.0 + 1.0));
    for (int j = 0; j < 12; ++j)
      op.a(i,j) = S(std::sin((i+1)*(j+2)*0.37) + (i == j ? 0.5 : 0.0));
  }
  using CG = rootba::ConjugateGradientsSolver<S>;
  typename CG::Options options;
  options.max_num_iterations = 500;
  typename CG::PerSolveOptions solve_options;
  auto preconditioner = std::make_unique<Preconditioner<S>>();
  for (int i = 0; i < 2; ++i) {
    Eigen::Matrix<S,6,6> diagonal = op.a.middleCols(6*i,6).transpose() * op.a.middleCols(6*i,6);
    diagonal.diagonal().array() += S(0.01);
    preconditioner->inverse[i] = diagonal.llt().solve(Eigen::Matrix<S,6,6>::Identity());
  }
  solve_options.preconditioner = std::move(preconditioner);
  typename CG::VecX rhs = -op.a.transpose() * r;
  typename CG::VecX x = CG::VecX::Zero(12);
  const auto result = CG(options).solve(&op, solve_options, rhs, x);
  std::cout << "{\"precision\":\"" << precision << "\",\"iterations\":" << result.num_iterations << ",\"step\":[";
  for (int i = 0; i < 12; ++i) std::cout << (i ? "," : "") << std::setprecision(17) << x[i];
  std::cout << "],\"jacobi_scaling_epsilon\":" << Sophus::Constants<S>::epsilonSqrt() << "}";
}
int main() { std::cout << "["; run<double>("float64"); std::cout << ","; run<float>("float32"); std::cout << "]\n"; }
