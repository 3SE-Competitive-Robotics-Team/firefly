// SPDX-License-Identifier: BSD-3-Clause
// Executes the unmodified official Basalt VioConfig constructor and serializer.
#include <src/utils/vio_config.cpp>
#include <iostream>

int main(int argc, char**) {
  basalt::VioConfig config;
  if (argc > 1) {
    try {
      cereal::JSONInputArchive input(std::cin);
      input(config);
    } catch (const cereal::Exception& error) {
      std::cerr << error.what() << '\n';
      return 1;
    }
  }
  cereal::JSONOutputArchive archive(std::cout);
  archive(config);
}
