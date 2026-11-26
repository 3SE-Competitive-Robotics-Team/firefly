#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/core.hpp>

// Separable edge-weighted least squares. Both right-hand sides (anchor mass
// and disparity residual) use the same tridiagonal factorization on each line.
class AnchorSmoother {
  using Pair = cv::Vec2f;
  std::array<float, 256> affinity_;
  cv::Mat horizontal_, vertical_, across_, transposed_, factors_;

  void solve(cv::Mat &field, const cv::Mat &edges, float strength) {
    factors_.create(1, field.rows * field.cols, CV_32F);
    auto factors = factors_.reshape(1, field.rows);
    for (int y = 0; y < field.rows; ++y) {
      auto *value = field.ptr<Pair>(y);
      auto *factor = factors.ptr<float>(y);
      const auto *edge = edges.ptr<float>(y);
      if (y == 0) {
        for (int x = 0; x < field.cols; ++x) {
          const float next = strength * edge[x];
          const float inv = 1.f / (1.f + next);
          factor[x] = next * inv;
          value[x] *= inv;
        }
        continue;
      }
      const auto *prior = field.ptr<Pair>(y - 1);
      const auto *prior_factor = factors.ptr<float>(y - 1);
      const auto *prior_edge = edges.ptr<float>(y - 1);
      for (int x = 0; x < field.cols; ++x) {
        const float prev = strength * prior_edge[x], next = strength * edge[x];
        const float inv = 1.f / (1.f + next + prev * (1.f - prior_factor[x]));
        factor[x] = next * inv;
        value[x] = (value[x] + prior[x] * prev) * inv;
      }
    }
    for (int y = field.rows - 2; y >= 0; --y) {
      auto *value = field.ptr<Pair>(y);
      const auto *next = field.ptr<Pair>(y + 1);
      const auto *factor = factors.ptr<float>(y);
      for (int x = 0; x < field.cols; ++x)
        value[x] += next[x] * factor[x];
    }
  }

public:
  AnchorSmoother() {
    for (int d = 0; d < 256; ++d)
      affinity_[d] = std::exp(-d / 12.f);
  }

  void filter(const cv::Mat &gray, cv::Mat &field) {
    horizontal_.create(gray.size(), CV_32F);
    vertical_.create(gray.size(), CV_32F);
    for (int y = 0; y < gray.rows; ++y) {
      auto *h = horizontal_.ptr<float>(y), *v = vertical_.ptr<float>(y);
      const auto *row = gray.ptr<uchar>(y);
      for (int x = 0; x < gray.cols - 1; ++x)
        h[x] = affinity_[std::abs(int(row[x]) - row[x + 1])];
      h[gray.cols - 1] = 0;
      if (y + 1 == gray.rows) {
        std::fill_n(v, gray.cols, 0.f);
      } else {
        const auto *below = gray.ptr<uchar>(y + 1);
        for (int x = 0; x < gray.cols; ++x)
          v[x] = affinity_[std::abs(int(row[x]) - below[x])];
      }
    }
    cv::transpose(horizontal_, across_);
    for (float strength : {144.f, 36.f, 9.f}) {
      cv::transpose(field, transposed_);
      solve(transposed_, across_, strength);
      cv::transpose(transposed_, field);
      solve(field, vertical_, strength);
    }
  }
};
