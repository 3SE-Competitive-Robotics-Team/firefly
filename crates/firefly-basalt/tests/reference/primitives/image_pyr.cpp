// Reference: official basalt-headers aa441ba3e51050c47ba1902537792a2e4db7e43d.
#include <basalt/image/image_pyr.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <limits>

template <typename T>
nlohmann::json run(size_t w, size_t h, size_t levels, bool constant) {
  basalt::ManagedImage<T> input(w, h);
  for (size_t y = 0; y < h; ++y)
    for (size_t x = 0; x < w; ++x)
      input(x, y) = constant ? std::numeric_limits<T>::max()
                            : T((x * x * 13 + y * y * 17 + x * y * 5 + 100)
                                % (uint64_t(std::numeric_limits<T>::max()) + 1));
  basalt::ManagedImagePyr<T> pyramid(input, levels);
  nlohmann::json result = {{"width", w}, {"height", h}, {"levels", levels},
                           {"bits", sizeof(T)*8}, {"constant", constant}};
  result["mipmap"] = nlohmann::json::array();
  const auto mipmap = pyramid.mipmap();
  for (size_t y = 0; y < mipmap.h; ++y)
    for (size_t x = 0; x < mipmap.w; ++x)
      result["mipmap"].push_back(mipmap(x, y));
  result["offsets"] = nlohmann::json::array();
  for (size_t i = 0; i <= levels; ++i) {
    const auto offset = pyramid.template lvl_offset<int>(i);
    result["offsets"].push_back({offset.x(), offset.y()});
  }
  return result;
}
int main() {
  nlohmann::json result = nlohmann::json::array();
  result.push_back(run<uint8_t>(8, 8, 2, false));
  result.push_back(run<uint16_t>(7, 9, 2, false));
  result.push_back(run<uint16_t>(8, 8, 2, true));
  result.push_back(run<uint8_t>(7, 9, 0, false));
  std::cout << result.dump(2) << '\n';
}
