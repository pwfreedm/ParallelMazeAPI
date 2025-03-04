#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <vector>

#include "maze.hpp"
class MazeTest : public testing::Test {
protected:
  Maze<std::unique_ptr<Cell[]>> m1{5, 5};
  Maze<std::vector<Cell>> m2{6, 6};
};

// Tests that the cell constructor zero initializes properly
TEST_F(MazeTest, ConstructionZeroInitialized) {
  Cell c{};
  EXPECT_FALSE(c.bottom(Side::LEFT));
  EXPECT_FALSE(c.top(Side::LEFT));
  EXPECT_FALSE(c.left(Side::LEFT));
  EXPECT_FALSE(c.right(Side::LEFT));
  EXPECT_FALSE(c.bottom(Side::RIGHT));
  EXPECT_FALSE(c.top(Side::RIGHT));
  EXPECT_FALSE(c.left(Side::RIGHT));
  EXPECT_FALSE(c.right(Side::RIGHT));
}

TEST_F(MazeTest, SubscriptOperator) {
  // test a horizontal connection
  m1.connect(0, 1);
  EXPECT_TRUE(m1[0].right(Side::LEFT));
  EXPECT_TRUE(m1[1].left(Side::RIGHT));

  // test a vertical connection
  assert(m2.size() == 36);
  m2.connect(0, 6);
  EXPECT_TRUE(m2[0].bottom(Side::LEFT));
  EXPECT_TRUE(m2[6].top(Side::LEFT));

  // test a connection in the last cell of an odd sized maze
  assert(m1.size() % 2 == 1);
  m1.connect(23, 24);
  EXPECT_TRUE(m1[23].right(Side::RIGHT));
  EXPECT_TRUE(m1[24].left(Side::LEFT));
}

TEST_F(MazeTest, ValidMove) {
  // valid moves in all directions
  EXPECT_TRUE(m1.validMove(0, 1));
  EXPECT_TRUE(m1.validMove(0, 5));
  EXPECT_TRUE(m1.validMove(1, 0));
  EXPECT_TRUE(m1.validMove(5, 0));
  EXPECT_TRUE(m1.validMove(6, 7));
  EXPECT_TRUE(m1.validMove(4, 9));

  // valid direction but >1 cell in that direction
  EXPECT_FALSE(m1.validMove(0, 10));
  EXPECT_FALSE(m1.validMove(0, 2));
  EXPECT_FALSE(m1.validMove(10, 0));
  EXPECT_FALSE(m1.validMove(2, 0));

  // connect adjacent indices in different rows
  EXPECT_FALSE(m1.validMove(4, 5));
  EXPECT_FALSE(m1.validMove(5, 4));

  // one index invalid
  EXPECT_FALSE(m1.validMove(24, m1.size()));
  EXPECT_FALSE(m1.validMove(m1.size(), 24));
  EXPECT_FALSE(m1.validMove(-1, 0));
  EXPECT_FALSE(m1.validMove(0, -1));
}

TEST_F(MazeTest, GetSide) {
  EXPECT_TRUE(m1.getSide(0) == Side::LEFT);
  EXPECT_TRUE(m2.getSide(1) == Side::RIGHT);

  EXPECT_FALSE(m1.getSide(5) == Side::LEFT);
  EXPECT_FALSE(m2.getSide(16) == Side::RIGHT);
}

TEST_F (MazeTest, ValueOperator) {
  EXPECT_TRUE(m1.val() == 0);
  EXPECT_TRUE (m2.val() == 0);

  m1.connect(0, 1);
  m2.connect(0, 6);

  EXPECT_TRUE (m1.val() == 132);

  EXPECT_TRUE (m2.val() == 48);
  EXPECT_FALSE(m2.val() == 0);
}

TEST_F(MazeTest, EqualityOperator) {
  Maze<std::unique_ptr<Cell[]>> m3{5, 5};
  Maze<std::vector<Cell>> m4{6, 6};
  Maze<std::vector<Cell>> m5{6, 6};

  // empty mazes of the same size should be equal
  EXPECT_TRUE(m1 == m3);
  EXPECT_TRUE(m2 == m4);

  EXPECT_TRUE(m2 == m5);
  EXPECT_TRUE(m5 == m4);

  // after these three lines, m1 and m4 should be equal but not m3
  m1.connect(16, 17);
  m4.connect(16, 17);
  m3.connect(15, 16);

  EXPECT_FALSE(m1 == m3);
  EXPECT_FALSE(m2 == m4);
  EXPECT_FALSE(m1 == Maze(3, 3));
}

TEST_F(MazeTest, MazeAccessors) {
  EXPECT_TRUE(m1.width() == 5);
  EXPECT_TRUE(m1.size() == 25);
  EXPECT_TRUE(m2.length() == 6);

  Maze<std::vector<Cell>> m3 (4, 6);

  EXPECT_TRUE(m3.length() == 4);
  EXPECT_TRUE(m3.width() == 6);
  EXPECT_TRUE(m3.size() == 24);
}

TEST_F (MazeTest, Connect) {
  //these tests are dependent on m1 being 5x5
  assert(m1.size() == 25);

  //test connections in each direction
  EXPECT_TRUE(m1.connect(0, 1));
  EXPECT_TRUE(m1.connect(2, 1));
  EXPECT_TRUE(m1.connect(0, m1.width()));
  EXPECT_TRUE(m1.connect (m1.width() + 1, 1));

  //valid direction but >1 row/col apart
  EXPECT_FALSE(m1.connect(0, 2));
  EXPECT_FALSE(m1.connect(0, m1.width() * 2));

  //connect adjacent indices in different rows
  EXPECT_FALSE(m1.connect(4, 5));
  EXPECT_FALSE(m1.connect(5,4));

  //one index invalid
  EXPECT_FALSE(m1.connect(65, 0));
  EXPECT_FALSE(m1.connect(24, 25));
}

TEST_F (MazeTest, RandomConnect)
{
  std::minstd_rand r(0);
  //optionals can be evaluated as booleans
  EXPECT_TRUE(m1.connectRandomNeighbor(r, 0).has_value());
  EXPECT_TRUE(m1.connectRandomNeighbor(r, 0).has_value());

  //should be false because no loops are allowed
  EXPECT_FALSE(m1.connectRandomNeighbor(r, 0).has_value());

  //true b/c connections to previously connected cells are allowed
  EXPECT_TRUE(m1.connectRandomNeighbor(r, 0, true).has_value());
}