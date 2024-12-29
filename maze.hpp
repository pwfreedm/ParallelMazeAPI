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
#include <sstream>
#include <type_traits>

class Cell;

/**for the maze to be generic, it needs to be constrained
    as long as the internal type can be random-access indexed
    and that index returns a reference to a cell, it is a valid
    type
*/
template<typename T>
concept CanMaze = requires (T const& maze) {
  { maze[0] } -> std::same_as<Cell&>;
};

//cells have 4 walls (they are squares).
//talks about which wall of a cell to view/change
enum class Wall
{
  TOP = 0,
  BOTTOM = 1,
  LEFT = 2,
  RIGHT = 3
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
};