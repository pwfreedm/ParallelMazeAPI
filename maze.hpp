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

#include <bitset>
#include <concepts>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <thread>



// cells have 4 walls (they are squares).
// talks about which wall of a cell to view/change
enum class Wall
{
  TOP = 0,
  BOTTOM = 1,
  LEFT = 2,
  RIGHT = 3
};

// there are two cells in a byte
// side offsets so that figuring out which cell needs changed
// is a matter of addition (see Cell::getSide() for more)
enum class Side
{
  RIGHT = 0,
  LEFT = 4
};

class Cell; 

/**for the maze to be generic, it needs to be constrained
    as long as the internal type can be random-access indexed
    and that index returns a reference to a cell, it is a valid
    type
*/
template<typename T>
concept CanMaze = requires (T const& maze) {
  // convertible to const because it doesn't matter if a ref is const or not;
  // the object at the end of the reference is not const.
  { maze[0] } -> std::convertible_to<const Cell&>;
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

        Each of these walls can be opened or closed. Opened walls can be
   traversed. In other words, opened walls are connected to a maze and closed
   walls form the walls of the traversible paths in a maze.
    */
class Cell
{

  std::uint8_t pair : 8;

  template<CanMaze Mazeable>
  friend class Maze;

public:
  // default ctor - do not invoke manually
  Cell () : pair (0) {}

  //============================ Cell Accessors ============================

  // returns true if the top wall of the cell on side l_or_r is opened
  bool
  top (Side l_or_r)
  {
    return wallIsOpened (l_or_r, Wall::TOP);
  }

  // returns true if the bottom wall of the cell on side l_or_r is opened
  bool
  bottom (Side l_or_r)
  {
    return wallIsOpened (l_or_r, Wall::BOTTOM);
  }

  // returns true if the left wall of the cell on side l_or_r is opened
  bool
  left (Side l_or_r)
  {
    return wallIsOpened (l_or_r, Wall::LEFT);
  }

  // returns true if the right wall of the cell on side l_or_r is opened
  bool
  right (Side l_or_r)
  {
    return wallIsOpened (l_or_r, Wall::RIGHT);
  }

private:
  // interprets the cell as an integer
  inline size_t
  val () const
  {
    return pair;
  }

  // interprets the requested side of the cell as an integer
  inline size_t
  sideVal (Side l_or_r) const
  {
    return (pair >> size_t (l_or_r)) & 0b1111;
  }

  // returns true if the provided cell wall is opened, false o/w
  inline bool
  wallIsOpened (Side l_or_r, Wall wall) const
  {
    return (pair >> (size_t (l_or_r) + size_t (wall))) & 0b1;
  }

  //============================= Cell Mutators =============================

  // Sets direction dir to true (removes that wall from the cell)
  inline void
  setDirection (Side l_or_r, Wall wall)
  {
    pair |= (0b1 << (size_t (l_or_r) + size_t (wall)));
  }

  // Flips the state of the bit corresponding to dir
  inline void
  updateDirection (Side l_or_r, Wall wall)
  {
    pair ^= (0b1 << (size_t (l_or_r) + size_t (wall)));
  }

public:
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

            this means that using the output stream on this cell would result
     in: 5 9 being printed
    */
  friend std::ostream&
  operator<< (std::ostream& os, const Cell& c)
  {
    os << c.sideVal (Side::LEFT) << ' ' << c.sideVal (Side::RIGHT);
    return os;
  }
};

//============================= End Cell Definition ============================

//============================ Begin Maze Definition ===========================



template<CanMaze Mazeable = std::unique_ptr<Cell[]>>
class Maze
{

  Mazeable mz;
  size_t len;
  size_t wid;

public:
  // force a maze to have dimensions
  Maze () = delete;

  // dimension ctor, creates a numRows x numCols maze
  Maze (size_t numRows, size_t numCols) : len (numRows), wid (numCols)
  {
    //if len * wid is odd, make an extra cell struct to store the last literal cell
    //otherwise it will be oob
    size_t size = numRows * numCols;
    size = size % 2 == 1 ? (size / 2) + 1 : size / 2;

    // because smart pointers don't use default ctors
    if constexpr (std::is_same<Mazeable, std::unique_ptr<Cell[]>> ())
    {
      mz = std::make_unique<Cell[]> (size);
    }
    else if constexpr (std::is_same<Mazeable, std::shared_ptr<Cell[]>> ())
    {
      mz = std::make_shared<Cell[]> (size);
    }
    else { mz = Mazeable (size); }
  }

  // move ctor - takes ownership of an existing mazeable type by moving it in.
  // after this constructor is called, mz should not be accessed
  Maze (Mazeable mz, size_t numRows, size_t numCols)
    : mz (std::move (mz)), len (numRows), wid (numCols)
  {
  }

  /** decides if idx is the left or right side of a cell. 
      NOTE: odd indices are ALWAYS on the right side of a cell, even if they should be the start of a new row. */
  inline Side
  getSide (size_t idx)
  {
    return idx & 0b1 ? Side::RIGHT : Side::LEFT;
  }

  // Gets the cell pair at index idx in the maze. Does not check bounds.
  // DO NOT USE THIS. Use connect to build the maze instead.
  Cell&
  operator[] (size_t idx)
  {
    return mz[size_t (idx / 2)];
  }

  // Equality op - Compares fields from smallest to largest
  bool
  operator== (Maze<Mazeable> const& o) const
  {
    if (width () != o.width ()) { return false; }
    if (length () != o.length ()) { return false; }
    for (size_t i = 0; i < size (); ++i)
    {
      if (mz[i].val () != o.mz[i].val ()) { return false; }
    }
    return true;
  }

  // Inequality op - Compares fields from smallest to largest
  bool
  operator!= (Maze<Mazeable> const& o) const
  {
    return !(*this == o);
  }

  // converts the maze to an size_t by adding together the values of all cells
  // can be used to say that a given maze is most likely equal to another maze
  // significantly faster than operator== will find two mazes equal
  size_t
  val ()
  {
    size_t val = 0;
    for (size_t i = 0; i < size () / 2; i += 2) { val += mz[i].val (); }
    return val;
  }
  //============================ Maze Accessors ============================

  size_t
  width ()
  {
    return wid;
  }

  size_t
  width () const
  {
    return wid;
  }

  size_t
  size ()
  {
    return wid * len;
  }

  size_t
  size () const
  {
    return wid * len;
  }

  size_t
  length ()
  {
    return len;
  }

  size_t
  length () const
  {
    return len;
  }

  //=========================== End Maze Accessors ===========================

  // Connects cells at idx1 and idx2. Bounds checked
  void
  connect (size_t idx1, size_t idx2)
  {
    //idx2ToWall validates the move as well as converting idx to wall
    auto wall = idx2ToWall (idx1, idx2);
    if (!wall.has_value()) { return; }

    Side src_side = getSide (idx1);
    Side dst_side = getSide (idx2);

    switch (wall.value())
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
    };
  }

  /** Picks a random neighbor using the provided rng.

    @p allowLoops - allow or disallow connecting to a cell that is already in
    the maze returns an optional that contains either the size_t connected to or
    nothing if there were no valid connections
  */
  template<std::uniform_random_bit_generator T>
  std::optional<size_t>
  connectRandomNeighbor (T& r, size_t idx, bool allowLoops = false)
  {
    std::bitset<4> valid (0b0000);
    std::uniform_int_distribution dist (0, 3);

    for (size_t i = 0; i < 4; ++i)
    {
      // if loops are allowed, the second half of this cnd will always be true
      valid[i] =
        validConnectingWall (idx, Wall (i)) && (cellInMaze (idx) <= allowLoops);
    }

    // since valid has 4 bits, the most this value can ever amount to is 3
    uint8_t num = dist (r);

    if (valid.count () == 0) { return std::nullopt; }

    while (!valid[num]) { num = dist (r); }

    auto idx2 = wallToIdx2 (idx, Wall (num));
    if (idx2) { connect (idx, idx2.value ()); }

    return std::make_optional (idx2);
  }

  // Returns true if going to idx2 from idx1 would remain in maze bounds
  bool
  validMove (size_t srcIdx, size_t dstIdx)
  {
    // move is invalid if either index is out of bounds
    bool valid = (srcIdx < size ()) & (dstIdx < size ());

    int diff = srcIdx - dstIdx;
    if (diff < 0) { std::swap(srcIdx, dstIdx); diff *= -1;}

    // make sure indices are exactly one row/col apart
    valid &= diff == wid | diff == 1;

    // disallow connecting the last cell in one row and the first in the next
    valid &= !(diff == 1 & srcIdx % wid == 0 & ++dstIdx == srcIdx);

    return valid;
  }

  friend std::ostream&
  operator<< (std::ostream& os, const Maze& m)
  {
    size_t row = 0;
    for (size_t i = 0; i < m.len / 2; ++i)
    {
      row += m.wid;
      os << m[row];
      for (size_t j = 1; j < m.wid / 2; ++i) { os << ' ' << m[row + j]; }
      os << '\n';
    }
    return os;
  }

private:

  // Returns the direction to get to idx2 from idx1
  std::optional<Wall>
  idx2ToWall (size_t idx1, size_t idx2)
  {
    if (!validMove (idx1, idx2)) { return std::nullopt; }
    int diff = idx1 - idx2;

    if (diff == 1) { return std::make_optional (Wall::LEFT); }
    if (diff == -1){ return std::make_optional (Wall::RIGHT); }
    if (diff > 1)  { return std::make_optional (Wall::TOP); }
    if (diff < -1) { return std::make_optional (Wall::BOTTOM); }

    return std::nullopt;
  }

  // takes an index and a wall and returns an optional containing the index
  // moving through that wall would reach if the move is legal and nothing
  // otherwise
  std::optional<size_t>
  wallToIdx2 (size_t idx1, Wall connection)
  {

    std::optional<size_t> idx2{std::nullopt};

    switch (connection)
    {
      case Wall::TOP: idx2 = idx1 - wid; break;
      case Wall::BOTTOM: idx2 = idx1 + wid; break;
      case Wall::LEFT: idx2 = idx1 - 1; break;
      case Wall::RIGHT: idx2 = idx1 + 1; break;
    }
    return validMove (idx1, idx2.value ()) ? idx2 : std::nullopt;
  }

  /** turns a wall and an index into two indices, then calls validMove with
   * them.  */
  bool
  validConnectingWall (size_t srcIdx, Wall connection)
  {
    switch (connection)
    {
      case Wall::TOP: return validMove (srcIdx, srcIdx - wid);
      case Wall::BOTTOM: return validMove (srcIdx, srcIdx + wid);
      case Wall::LEFT: return validMove (srcIdx, --srcIdx);
      case Wall::RIGHT: return validMove (srcIdx, ++srcIdx);
      default: return false;
    }
  }

  // returns true if the cell at idx has at least one wall open, false o/w
  bool
  cellInMaze (size_t idx)
  {
    // double negation b/c the only values that matter here are 0 and 1
    return !!(*this)[idx].sideVal (getSide (idx));
  }
};

//============================= End Maze Definition ============================

// prevent naming conflicts on the parallelize method
namespace MazeGeneration
{
/** Parallelizes the generation of @p maze using @p f over @p coreCount cores

    For this method to work properly, a few things are needed:
    1) the first parameter to the algorithm being parallelized needs to ba a
   maze 2) the second parameter to the algorithm being parallelized needs to be
   an rng seed 3) the width of the maze must be at least 128 cells long (64
   bytes, 1 cache line) 4) the length of the maze must be at least 4x the core
   count

    requirements 3 and 4 are imposed to prevent false sharing and save time, as
   the overhead of parallelizing generation of small mazes is too high.

    NOTE: If the first two conditions are not met, the program will crash.
    NOTE: If the last two conditions are not met, the maze will be generated
   serially
*/
template<class Func, CanMaze Mazeable, class... Args>
void
parallelize (Func&& f, Maze<Mazeable>& maze, long long seed, size_t coreCount,
             Args&&... args)
{
  // generate small mazes serially because blocked algorithm restricts domain.
  if (maze.width () * 2 < std::hardware_destructive_interference_size ||
      maze.length () < coreCount * 4)
  {
    f (maze, seed, std::forward<Args> (args)...);
    return;
  }

  // used for smoothing over edges between generated maze segments
  std::vector<size_t> blockStarts (coreCount);

  // blocking and parallel maze generation
  {
    std::vector<std::jthread> threads (coreCount);
    // if the seed is passed to each core, it will create n repeated segments on
    // n cores
    std::minstd_rand0 r (seed);

    // used to compute the subsection of maze to give to each core
    size_t currentBlockStart = 0;
    size_t coresWithExtraRow = maze.length () % coreCount;
    size_t numCols = maze.length () / coreCount;

    std::span mz{maze[0], maze.size ()};

    for (size_t thread = 0; thread < coreCount; ++thread)
    {
      // need one assignment otherwise the wrong one will be passed to the
      // jthread
      const size_t cellCount =
        numCols * maze.width () + (coresWithExtraRow-- > 0 ? maze.width () : 0);

      std::span currentBlock = mz.subspan (0, cellCount);
      mz = mz.subspan (cellCount);
      threads.emplace_back (f, Maze{currentBlock, numCols, maze.width ()}, r (),
                            std::forward<Args> (args)...);
      blockStarts[thread] = currentBlockStart;
      currentBlockStart += cellCount;
    }
  }

  // smoothing is needed because the parallel generation creates n submazes but
  // does not connect any of them.
  std::minstd_rand0 r (seed);

  size_t rownum = 0;
  size_t anti_consecutive_bias = 0;
  for (auto block : blockStarts)
  {
    rownum += block;
    size_t start_idx = rownum * maze.width ();
    for (size_t idx = start_idx; idx < start_idx + maze.width (); ++idx)
    {
      Cell prev_cell = maze[idx - 1];
      auto prev_side = maze.getSide (idx - 1);

      // if the previous cell's top face isnt open and a random check is passed
      if (prev_cell.top (prev_side) == 0 &&
          static_cast<size_t> (r ()) % 11 > anti_consecutive_bias)
      {
        maze.connect (idx, maze.getNeighbor (idx, Wall::TOP));
        anti_consecutive_bias += 3;
      }
      else { --anti_consecutive_bias; }
    }
  }
}

};