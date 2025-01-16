#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "maze.hpp"

class MazeTest : public testing::Test
{
protected:
  Maze<std::unique_ptr<Cell[]>> m1{5, 5};
  Maze<std::vector<Cell>> m2{5, 5};

};
// Tests that the cell constructor zero initializes properly
TEST_F (MazeTest, ConstructionZeroInitialized)
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

TEST_F (MazeTest, AccessorsWorking) { 
  //test a horizontal connection
  m1.connect(0, 1);
  EXPECT_TRUE(m1[0].right(Side::LEFT));
  EXPECT_TRUE(m1[1].left(Side::RIGHT));

  //test a vertical connection
  m2.connect(0,5);
  EXPECT_TRUE(m2[0].bottom(Side::LEFT));
  EXPECT_TRUE(m2[5].top(Side::RIGHT));

  //test a connection in the last cell of an odd sized maze
  assert(m1.size() % 2 == 1);
  m1.connect(23, 24);
  EXPECT_TRUE(m1[23].right(Side::RIGHT));
  EXPECT_TRUE(m1[24].left(Side::LEFT));
}


TEST_F (MazeTest, ValidMove)
{
 //valid moves in all directions
 EXPECT_TRUE(m1.validMove(0,1));
 EXPECT_TRUE(m1.validMove(0,5));
 EXPECT_TRUE(m1.validMove(1, 0));
 EXPECT_TRUE(m1.validMove(5, 0));
 EXPECT_TRUE(m1.validMove(6, 7));
 EXPECT_TRUE(m1.validMove(4,9));

 //valid direction but >1 cell in that direction
 EXPECT_FALSE(m1.validMove(0, 10));
 EXPECT_FALSE(m1.validMove(0, 2));
 EXPECT_FALSE(m1.validMove(10, 0));
 EXPECT_FALSE(m1.validMove(2, 0));

 //connect adjacent indices in different rows
 EXPECT_FALSE(m1.validMove(4, 5));
 EXPECT_FALSE(m1.validMove(5, 4));

 //one index invalid
 EXPECT_FALSE(m1.validMove(24, m1.size()));
 EXPECT_FALSE(m1.validMove(m1.size(), 24));
 EXPECT_FALSE(m1.validMove(-1, 0));
 EXPECT_FALSE(m1.validMove(0, -1));
}

