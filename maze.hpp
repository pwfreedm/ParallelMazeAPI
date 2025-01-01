/** 
Represents an unweighted, non-directed graph as a group of cells (maze). 
Each cell is 1/2 byte in size; two are packed into a single byte. 

This header is fully self contained and depends solely on C++ 20.

This header DOES NOT provide algorithms for generating mazes. 
Sample algorithms are available on the repository, but this header
exists as a standard to be used inside independently implemented algorithms.
It provides all the tools needed for maintaining memory-efficient mazes and 
options for parallelizing generation given an arbitrary algorithm, so long as 
the algorithm makes use of this header.

@a - Peter Freedman 
@n - maze.hpp
*/

#pragma once

#include <concepts>
#include <iostream>
#include <memory>
#include <random>
#include <type_traits>

//cells have 4 walls (they are squares).
//talks about which wall of a cell to view/change
enum class Wall
{
  TOP = 0,
  BOTTOM = 1,
  LEFT = 2,
  RIGHT = 3,
  NONE = 4
};

//there are two cells in a byte
//side offsets so that figuring out which cell needs changed
//is a matter of addition (see Cell::getSide() for more)
enum class Side
{
  RIGHT = 0,
  LEFT = 4
};

//============================== Cell Definition ==============================

/** Represents a pair of two cells in a maze. 
        Because each would only have been four bits, two are packed together. 
        The struct is still called a cell for readability, 
        but in reality it will always be a cell pair. These cells 
        will be referred to as Side::LEFT and Side::RIGHT, where the left cell
        takes up the highest (farthest left) 4 bits of a byte and the right cell
        takes up the lowest 4 bits of the same byte. 
        
        Additionally, cells are constructed from four walls:
                
                        Side::TOP
                        ______________
                        |            |
                        |            |
            Side::LEFT  |            |   Side::RIGHT
                        |            |
                        |            | 
                        ______________
                        Side:BOTTOM

        Each of these walls can be opened or closed. Opened walls can be traversed. 
        In other words, opened walls are connected to a maze and closed walls form
        the walls of the traversible paths in a maze. 
    */
class Cell
{

  std::uint8_t pair : 8;

public:
  //default ctor - do not invoke manually
  Cell () : pair (0) {}

private:
  //============================ Cell Accessors ============================

  //interprets the cell as an integer
  inline int
  val () const
  {
    return pair;
  }

  //interprets the requested side of the cell as an integer
  inline int
  side_val (Side l_or_r) const
  {
    return (pair >> int (l_or_r)) & 0b1111;
  }

  //returns true if the provided cell wall is opened, false o/w
  inline bool
  dir_val (Side l_or_r, Wall wall) const
  {
    return (pair >> (int (l_or_r) + int (wall))) & 0b1;
  }

  //============================= Cell Mutators =============================

  // Sets direction dir to true (removes that wall from the cell)
  inline void
  setDirection (Side l_or_r, Wall wall)
  {
    pair |= (0b1 << (int (l_or_r) + int (wall)));
  }

  //Flips the state of the bit corresponding to dir
  inline void
  updateDirection (Side l_or_r, Wall wall)
  {
    pair ^= (0b1 << (int (l_or_r) + int (wall)));
  }

  /** output stream operator overload for a cell
            prints the left and right cells as ints separated by spaces
            eg:
            take the cell whose left side has its top and left walls opened 
            and whose right side has its top and right walls opened:

            left      right
                    |
                    |
                    |
           _________|_________

            according to the values of Side::LEFT, Side::TOP, and Side::RIGHT,
            the left cell should have the bit pattern   0101 
            the right cell should have the bitt pattern 1001

            this means that using the output stream on this cell would result in:
            5 9
            being printed
    */
  friend std::ostream&
  operator<< (std::ostream& os, const Cell& c)
  {
    os << c.side_val (Side::LEFT) << ' ' << c.side_val (Side::RIGHT);
    return os;
  }
};

//============================= End Cell Definition ============================

//============================ Begin Maze Definition ===========================

/**for the maze to be generic, it needs to be constrained
    as long as the internal type can be random-access indexed
    and that index returns a reference to a cell, it is a valid
    type
*/
template<typename T>
concept CanMaze = requires (T const& maze) {
  //TODO: constrain this concept so that maze has support for forward input iterators
  { maze[0] } -> std::same_as<Cell&>;
};

template<CanMaze Mazeable>
class Maze
{

  Mazeable mz;
  int len;
  int wid;

public:
  //force a maze to have dimensions
  Maze () = delete;

  //dimension ctor, creates a numRows x numCols maze
  Maze (int numRows, int numCols) : len (numRows), wid (numCols)
  {
    //because smart pointers don't use default ctors
    if constexpr (std::is_same<Mazeable, std::unique_ptr<Cell[]>> ())
    {
      mz = std::make_unique<Cell[]> ((numRows * numCols) / 2);
    }
    else if constexpr (std::is_same<Mazeable, std::shared_ptr<Cell[]>> ())
    {
      mz = std::make_shared<Cell[]> ((numRows * numCols) / 2);
    }
    else { mz = Mazeable ((numRows * numCols) / 2); }
  }

  /** decides if idx is the left or right side of a cell. */
  inline Side
  getSide (int idx)
  {
    return idx & 0b1 ? Side::RIGHT : Side::LEFT;
  }

  //Gets the cell pair at index idx in the maze. Does not check bounds.
  //DO NOT USE THIS. Use connect to build the maze instead.
  Cell&
  operator[] (size_t idx)
  {
    return mz[int (idx / 2)];
  }

  //Equality op - Compares fields from smallest to largest
  bool
  operator== (Maze<Mazeable> const& o) const
  {
    if (width () != o.width ()) { return false; }
    if (length () != o.length ()) { return false; }
    for (int i = 0; i < size (); ++i)
    {
      if (mz[i].val () != o.mz[i].val ()) { return false; }
    }
    return true;
  }

  //Inequality op - Compares fields from smallest to largest
  bool
  operator!= (Maze<Mazeable> const& o) const
  {
    return !(*this == o);
  }

  int
  val ()
  {
    //TODO: once support for forward iterators is implemented, use ranges::accumulate here
  }
  //============================ Maze Accessors ============================

  int
  width ()
  {
    return wid;
  }

  int
  width () const
  {
    return wid;
  }

  int
  size ()
  {
    return wid * len;
  }

  int
  size () const
  {
    return wid * len;
  }

  int
  length ()
  {
    return len;
  }

  int
  length () const
  {
    return len;
  }

  //=========================== End Maze Accessors ===========================

  //Connects cells at idx1 and idx2. Bounds checked
  void
  connect (int idx1, int idx2)
  {
    Wall wall = getConnectingWall (idx1, idx2);
    if (wall == Wall::NONE) { return; }

    Side src_side = getSide (idx1);
    Side dst_side = getSide (idx2);

    switch (wall)
    {
      case Wall::TOP:
        (*this)[idx1].setDirection (src_side, Wall::TOP);
        (*this)[idx2].setDirection (dst_side, Wall::BOTTOM);
        break;
      case Wall::BOTTOM:
        (*this)[idx1].setDirection (src_side, Wall::BOTTOM);
        (*this)[idx2].setDirection (dst_side, Wall::TOP);
        break;
      case Wall::LEFT:
        (*this)[idx1].setDirection (src_side, Wall::LEFT);
        (*this)[idx2].setDirection (dst_side, Wall::RIGHT);
        break;
      case Wall::RIGHT:
        (*this)[idx1].setDirection (src_side, Wall::RIGHT);
        (*this)[idx2].setDirection (dst_side, Wall::LEFT);
      //this case is unreachable, here to silence a warning
      default: break;
    };
  }

  template<std::uniform_random_bit_generator T>
  void
  connectRandomNeighbor (T& r, int idx, bool allowLoops = false)
  {
    std::uniform_int_distribution dist(0, 4);

    while ()
    //need to do this in exactly one rng call 
    //how can I store up to four indices as efficiently as possible
  }

  //Returns true if going to idx2 from idx1 would remain in maze bounds
  bool
  validMove (int srcIdx, int dstIdx)
  {
    //move is invalid if either index is out of bounds
    bool valid = srcIdx >= size() && dstIdx >= size(); 

    int diff = srcIdx - dstIdx;
    int abs = diff < 0 ? -diff : diff;

    //make sure vertically aligned indices are exactly one row apart
    valid &= abs > wid;

    //disallow connecting the last cell in one row and the first in the next
    valid &= ((diff == -1 && dstIdx % wid != 0) | (diff == 1 && dstIdx % wid != 0));

    return valid;
  }

  //Returns the direction to get to idx2 from idx1
  Wall
  getConnectingWall (int idx1, int idx2)
  {
    if (!validMove (idx1, idx2)) { return Wall::NONE; }
    int diff = idx1 - idx2;

    if (diff == 1) { return Wall::LEFT; }
    else if (diff == -1) { return Wall::RIGHT; }
    else if (diff > 1) { return Wall::TOP; }
    else { return Wall::BOTTOM; }
  }

  friend std::ostream&
  operator<< (std::ostream& os, const Maze& m)
  {
    for (int i = 0; i < m.len / 2; ++i)
    {
      int row = i * m.wid;
      os << m[row];
      for (int j = 1; j < m.wid / 2; ++i) { os << ' ' << m[row + j]; }
      os << '\n';
    }
    return os;
  }

  private:

  /** turns a wall and an index into two indices, then calls validMove with them.  */
  bool validMove (int srcIdx, Wall connection)
  {
    switch (connection)
    {
      case Wall::TOP:
        return validMove(srcIdx, srcIdx - wid);
      case Wall::BOTTOM: 
        return validMove(srcIdx, srcIdx + wid);
      case Wall::LEFT:
        return validMove(srcIdx, --srcIdx);
      case Wall::RIGHT:
        return validMove(srcIdx, ++srcIdx);
      default:
        return false;
    }
  }
};

//============================= End Maze Definition ============================

//prevent naming conflicts on the parallelize method
namespace MazeTools
{

};