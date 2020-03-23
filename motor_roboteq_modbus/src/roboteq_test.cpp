#include <roboteq_device.h>

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <chrono>
#include <thread>

int main() {
  RoboteqDevice r("/dev/roboteq0", 115200, 2);

  auto fid = r.getFirmwareID();
  std::string fid_string;
  for (const auto &piece : fid) fid_string += std::to_string(piece) + ".";
  std::cout << "Firmware ID: " << fid_string << "\n";

  int i;

  const int TEST_MAX = 100;

  for(i=0;i<TEST_MAX;i++) {
    r.commandGo(1,i);
    r.commandGo(2,i);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  for(i=TEST_MAX;i>=0;i--) {
    r.commandGo(1,i);
    r.commandGo(2,i);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  for(i=0;i<TEST_MAX;i++) {
    r.commandGo(1,-i);
    r.commandGo(2,-i);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  for(i=TEST_MAX;i>=0;i--) {
    r.commandGo(1,-i);
    r.commandGo(2,-i);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }


 for(i=0;i<1000;i++) {
    auto cb = r.getBrushlessCount();
    std::cout << "CB " << cb[0] << " " << cb[1] << " ";

    auto bs = r.getBrushlessSpeed();
    std::cout << "BS " << bs[0] << " " << bs[1] << " ";

    auto v = r.getVoltage();
    std::cout << "V " << v[0] << " " << v[1] << " " << v[2] << " ";

    auto a = r.getCurrent();
    std::cout << "A " << a[0] << " " << a[1] << " ";

    auto t = r.getTemperature();
    std::cout << "T " << t[0] << " " << t[1] << " " << t[2] << "\n";

    std::cout << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::cout << "foo\n";
  return 0;
}
