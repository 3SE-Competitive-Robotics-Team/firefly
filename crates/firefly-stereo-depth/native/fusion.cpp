#include "smoothing.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

namespace {
constexpr int width = 320, height = 240;
struct Observation {
  int x, y;
  float disparity, relative;
};
struct Line {
  double scale = 0, offset = 0;
  bool valid = false;
};
float median(std::vector<float> &v) {
  auto mid = v.begin() + v.size() / 2;
  std::nth_element(v.begin(), mid, v.end());
  return *mid;
}
Line fit(const std::vector<Observation> &samples) {
  if (samples.size() < 12)
    return {};
  std::vector<float> weights(samples.size(), 1), errors(samples.size());
  Line line;
  for (int iter = 0; iter < 5; ++iter) {
    double sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
      auto s = samples[i];
      double w = weights[i];
      sw += w;
      sx += w * s.relative;
      sy += w * s.disparity;
      sxx += w * s.relative * s.relative;
      sxy += w * s.relative * s.disparity;
    }
    double variance = sxx - sx * sx / sw;
    if (variance <= 1e-10 * sw)
      break;
    line.scale = (sxy - sx * sy / sw) / variance;
    line.offset = (sy - line.scale * sx) / sw;
    for (size_t i = 0; i < samples.size(); ++i)
      errors[i] = std::abs(samples[i].disparity -
                           line.scale * samples[i].relative - line.offset);
    auto copy = errors;
    double delta = std::max(.25, 1.345 * 1.4826 * median(copy));
    for (size_t i = 0; i < samples.size(); ++i)
      weights[i] = std::min(1., delta / std::max(1e-8, double(errors[i])));
    line.valid = true;
  }
  if (line.valid && line.scale > 0)
    return line;
  std::vector<float> ratios;
  for (auto s : samples)
    if (s.relative > 1e-6)
      ratios.push_back(s.disparity / s.relative);
  if (ratios.size() >= 12)
    return {median(ratios), 0, true};
  return {};
}
double patchCorrelation(const cv::Mat &l, const cv::Mat &r, int x, int y,
                        float rx) {
  int ix = int(std::floor(rx));
  double frac = rx - ix;
  if (x < 3 || x >= width - 3 || y < 3 || y >= height - 3 || ix < 3 ||
      ix >= width - 4)
    return 0;
  double a = 0, b = 0, aa = 0, bb = 0, ab = 0;
  for (int j = -3; j <= 3; ++j)
    for (int k = -3; k <= 3; ++k) {
      auto row = r.ptr<uchar>(y + j);
      double u = l.at<uchar>(y + j, x + k),
             v = (1 - frac) * row[ix + k] + frac * row[ix + k + 1];
      a += u;
      b += v;
      aa += u * u;
      bb += v * v;
      ab += u * v;
    }
  double va = aa - a * a / 49., vb = bb - b * b / 49.;
  return std::min(va, vb) > 49 * 9 ? (ab - a * b / 49.) / std::sqrt(va * vb)
                                   : 0.;
}
void sparse(const cv::Mat &l, const cv::Mat &r, const cv::Mat &excluded,
            std::vector<Observation> &obs) {
  std::vector<cv::Point2f> p, q, back;
  cv::goodFeaturesToTrack(l, p, 350, .005, 5, cv::noArray(), 5);
  // Preserve corner ranking while skipping areas covered by epipolar matches.
  p.erase(std::remove_if(p.begin(), p.end(),
                         [&](const cv::Point2f &point) {
                           return excluded.at<uchar>(cvRound(point.y),
                                                     cvRound(point.x)) != 0;
                         }),
          p.end());
  if (p.empty())
    return;
  std::vector<uchar> f, b;
  std::vector<cv::Mat> lp, rp;
  cv::Size win(15, 15);
  int levels = std::min(cv::buildOpticalFlowPyramid(l, lp, win, 3),
                        cv::buildOpticalFlowPyramid(r, rp, win, 3));
  cv::calcOpticalFlowPyrLK(lp, rp, p, q, f, cv::noArray(), win, levels);
  size_t accepted = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    float d = q[i].x - p[i].x;
    if (!f[i] || std::abs(q[i].y - p[i].y) > .6 || !(d >= .5 && d <= 63))
      continue;
    int x = cvRound(p[i].x), y = cvRound(p[i].y);
    if (patchCorrelation(l, r, x, y, x + d) >= .90) {
      p[accepted] = p[i];
      q[accepted++] = q[i];
    }
  }
  if (accepted == 0)
    return;
  q.resize(accepted);
  cv::calcOpticalFlowPyrLK(rp, lp, q, back, b, cv::noArray(), win, levels);
  for (size_t i = 0; i < accepted; ++i) {
    if (b[i] && cv::norm(back[i] - p[i]) <= .6)
      obs.push_back({cvRound(p[i].x), cvRound(p[i].y), q[i].x - p[i].x, 0});
  }
}

float scanline(const cv::Mat &source, const cv::Mat &target, int x, int y,
               int direction) {
  std::array<std::uint16_t, 64> costs{};
  int end = std::min(63, direction > 0 ? width - 3 - x : x - 2), best = 0;
  if (end < 3)
    return -1;
  // Consecutive lanes compare the same source pixel to adjacent disparities.
  // A 5x5 sum is at most 6375. Reverse search uses the same forward loads.
  for (int j = -2; j <= 2; ++j) {
    auto a = source.ptr<uchar>(y + j) + x - 2;
    auto b = target.ptr<uchar>(y + j) + x - 2 - (direction < 0 ? end : 0);
    for (int k = 0; k < 5; ++k)
      for (int lane = 0; lane <= end; ++lane)
        costs[lane] += std::abs(int(a[k]) - b[k + lane]);
  }
  auto cost = [&](int d) { return int(costs[direction > 0 ? d : end - d]); };
  for (int d = 1; d <= end; ++d) {
    if (cost(d) < cost(best))
      best = d;
  }
  if (best == 0 || best == end || cost(best) > 250)
    return -1;
  for (int d = 0; d <= end; ++d)
    if (std::abs(d - best) > 1 &&
        cost(d) <= cost(best) + std::max(12, cost(best) / 4))
      return -1;
  float curvature = cost(best - 1) + cost(best + 1) - 2 * cost(best);
  return best + (curvature > 0
                     ? .5f * (cost(best - 1) - cost(best + 1)) / curvature
                     : 0);
}
void epipolar(const cv::Mat &l, const cv::Mat &r,
              std::vector<Observation> &obs) {
  for (int cy = 5; cy < height - 5; cy += 9)
    for (int cx = 5; cx < width - 5; cx += 9) {
      int x = cx, y = cy, gradient = 0;
      for (int j = -2; j <= 2; ++j)
        for (int k = -2; k <= 2; ++k) {
          auto a = l.ptr<uchar>(cy + j);
          int dx = std::abs(int(a[cx + k - 1]) - a[cx + k + 1]);
          if (dx > gradient) {
            x = cx + k;
            y = cy + j;
            gradient = dx;
          }
        }
      if (gradient < 12)
        continue;
      float d = scanline(l, r, x, y, 1);
      if (d < .5)
        continue;
      float reverse = scanline(r, l, cvRound(x + d), y, -1);
      if (reverse >= .5 && std::abs(d - reverse) <= .75 &&
          patchCorrelation(l, r, x, y, x + d) >= .9)
        obs.push_back({x, y, d, 0});
    }
}

struct Fusion {
  std::vector<Observation> observations;
  cv::Mat excluded = cv::Mat(height, width, CV_8U),
          correction = cv::Mat(height, width, CV_32FC2);
  AnchorSmoother smoother;
};
template <class F>
int guarded(char *error, size_t capacity, F operation) noexcept {
  try {
    operation();
    return 0;
  } catch (const std::exception &e) {
    std::snprintf(error, capacity, "%s", e.what());
  } catch (...) {
    std::snprintf(error, capacity, "unknown stereo matching error");
  }
  return -1;
}
cv::Mat gray(const unsigned char *pixels) {
  return cv::Mat(height, width, CV_8U, const_cast<unsigned char *>(pixels));
}
} // namespace

extern "C" int depth_fusion_create(void **handle, char *error,
                                   size_t capacity) noexcept {
  return guarded(error, capacity, [&] {
    cv::setNumThreads(1);
    *handle = new Fusion;
  });
}
extern "C" void depth_fusion_destroy(void *handle) noexcept {
  delete static_cast<Fusion *>(handle);
}
extern "C" int depth_fusion_match(void *handle, const unsigned char *left,
                                  const unsigned char *right, char *error,
                                  size_t capacity) noexcept {
  return guarded(error, capacity, [&] {
    auto &f = *static_cast<Fusion *>(handle);
    auto l = gray(left), r = gray(right);
    f.observations.clear();
    epipolar(l, r, f.observations);
    f.excluded.setTo(0);
    for (auto s : f.observations)
      cv::circle(f.excluded, {s.x, s.y}, 2, 255, -1);
    sparse(l, r, f.excluded, f.observations);
  });
}
extern "C" int depth_fusion_restore(void *handle, const unsigned char *left,
                                    const float *relative, float *metres,
                                    char *error, size_t capacity) noexcept {
  return guarded(error, capacity, [&] {
    auto &f = *static_cast<Fusion *>(handle);
    for (auto &s : f.observations)
      s.relative = relative[s.y * width + s.x];
    auto line = fit(f.observations);
    if (!line.valid) {
      std::fill_n(metres, width * height, 0.f);
      return;
    }
    f.correction.setTo(0);
    for (auto s : f.observations) {
      f.correction.at<cv::Vec2f>(s.y, s.x) = {
          1.f, float(s.disparity - (line.scale * s.relative + line.offset))};
    }
    f.smoother.filter(gray(left), f.correction);
    // Rectified HIL stereo rig: fx in pixels times the 5 cm baseline.
    constexpr float bf = 8.43034975f;
    for (int y = 0; y < height; ++y)
      for (int x = 0; x < width; ++x) {
        auto i = y * width + x;
        float d = line.scale * relative[i] + line.offset;
        const auto pair = f.correction.at<cv::Vec2f>(y, x);
        d += pair[1] / (pair[0] + 1.f / 576);
        metres[i] = d > 0 && std::isfinite(d) ? bf / d : 0;
      }
  });
}
