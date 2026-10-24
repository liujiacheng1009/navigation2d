#include <cassert>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "navigation2d/costmap/grid_2d.h"
#include "navigation2d/test/temporary_directory.h"
int main() {
  using namespace navigation2d;
  test::TemporaryDirectory directory;
  const auto path = directory.path() / "map.json";
  const auto write = [&](const std::string& content) {
    std::ofstream output(path);
    output << content;
    if (!output) {
      throw std::runtime_error("cannot write grid fixture");
    }
  };
  write(R"({"width":2,"height":2,"resolution":0.5,"cells":[0,100,255,254]})");
  const auto grid = Grid2d::Load(path.string());
  assert(grid.width() == 2 && grid.height() == 2 && grid.resolution() == .5);
  assert(!grid.occupied(0, 0) && grid.occupied(1, 0) && grid.occupied(1, 1));
  assert(grid.unknown(0, 1) && !grid.occupied(0, 1));
  assert(grid.occupied(-1, 0) && !grid.unknown(-1, 0));
  assert(grid.collides(.75, .25, .01) && !grid.collides(.25, .25, .01));
  assert(grid.ToCell(.75, .25) == std::pair(1, 0));
  assert(grid.CellCenter(0, 1) == std::pair(.25, .75));
  write(R"({"width":4,"height":3,"resolution":0.1,
      "rectangles":[[1,1,3,2,100],[3,0,4,1,255],[2,1,3,2,0]]})");
  const auto rectangles = Grid2d::Load(path.string());
  assert(rectangles.occupied(1, 1) && !rectangles.occupied(2, 1));
  assert(rectangles.unknown(3, 0) && !rectangles.occupied(3, 0));
  for (const char* invalid :
       {R"({"width":2,"height":2,"resolution":0.1,"cells":[0,0,0]})",
        R"({"width":1,"height":1,"resolution":0,"cells":[0]})",
        R"({"width":2,"height":2,"resolution":0.1,"rectangles":[[0,0,3,1,100]]})",
        R"({"width":2,"height":2,"resolution":0.1,"rectangles":[[0,0,1,1]]})"}) {
    write(invalid);
    bool rejected = false;
    try {
      static_cast<void>(Grid2d::Load(path.string()));
    } catch (const std::exception&) {
      rejected = true;
    }
    assert(rejected);
  }
}
