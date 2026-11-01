// SPDX-License-Identifier: BSD-3-Clause
#include <basalt/io/dataset_io.h>
#include <basalt/serialization/eigen_io.h>
#include <nlohmann/json.hpp>
#include <iostream>
#include <sstream>
template<class T> std::string binary(T& value) {
  std::ostringstream stream;
  { cereal::BinaryOutputArchive archive(stream); archive(value); }
  std::ostringstream hex;
  for (unsigned char c:stream.str()) hex<<std::hex<<std::setfill('0')<<std::setw(2)<<unsigned(c);
  return hex.str();
}
int main() {
  basalt::GyroData gyro;gyro.timestamp_ns=-123;gyro.data<<1.25,-2.5,3.75;
  basalt::AccelData accel;accel.timestamp_ns=456;accel.data<<-9.81,0,0.125;
  basalt::ManagedImage<uint8_t> image(3,2);
  for(size_t i=0;i<6;i++) image.ptr[i]=i*51;
  nlohmann::json result{{"gyro",binary(gyro)},{"accel",binary(accel)},{"image",binary(image)}};
  std::ostringstream text; {cereal::JSONOutputArchive a(text);a(gyro,accel);}
  result["json"]=nlohmann::json::parse(text.str());
  std::cout<<result.dump(2)<<'\n';
}
