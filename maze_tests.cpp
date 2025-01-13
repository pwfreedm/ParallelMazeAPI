#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "maze.hpp"

class MazeTests : public testing::Test
{
protected:
  Maze<std::unique_ptr<Cell[]>> m1{5, 5};
  Maze<std::vector<Cell>> m2{5, 5};

  void
  MazeTest ()
  {
  }
};
// Tests that the cell constructor zero initializes properly
TEST (PublicCellAPI, ConstructionZeroInitialized)
{
  Cell c{};
  EXPECT_FALSE (c.bottom (Side::LEFT));
  EXPECT_FALSE (c.top (Side::LEFT));
  EXPECT_FALSE (c.left (Side::LEFT));
  EXPECT_FALSE (c.right (Side::LEFT));
  EXPECT_FALSE (c.bottom (Side::RIGHT));
  EXPECT_FALSE (c.top (Side::RIGHT));
  EXPECT_FALSE (c.left (Side::RIGHT));
  EXPECT_FALSE (c.right (Side::RIGHT));
}

TEST (PublicCellAPI, AccessorsWorking) { Cell c{}; }