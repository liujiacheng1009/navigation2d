#include "navigation2d/exploration/frontier_visit_memory.h"

#include <algorithm>
#include <cmath>
namespace navigation2d::frontier_internal {
void FrontierVisitMemory::ClearFailures() {
  entries_.erase(std::remove_if(entries_.begin(), entries_.end(),
                                [](const VisitCooling& entry) { return !entry.persistent; }),
                 entries_.end());
}

}  // namespace navigation2d::frontier_internal
