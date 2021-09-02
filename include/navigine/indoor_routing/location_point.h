#pragma once

#include <vector>

namespace navigine {
namespace indoor_routing {

struct LocationPoint
{
  int level = 0;
  double x = 0.0;
  double y = 0.0;

  bool isValid()const { return level > 0; }
};

using PolyLine = std::vector<LocationPoint>;

} } // namespace navigine::indoor_routing
