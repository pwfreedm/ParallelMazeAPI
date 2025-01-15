

#include "maze_tests.cpp"
#include "maze.hpp"
#include <gtest/gtest.h>

int 
main()
{
    // Maze mz{5, 5};
    // int srcIdx = 6; 
    // int dstIdx = 1;
    // bool valid = (srcIdx < mz.size ()) & (dstIdx < mz.size ());
    // int diff = srcIdx - dstIdx;
    // int abs = diff < 0 ? -diff : diff;
    // valid &= abs == mz.width() | abs == 1;

    // //if indices are one apart but on different rows
    // //eg: src = 4 and dst = 5
    // valid &=
    // ((diff == -1 | dstIdx % mz.width() != 0) & (diff == 1 | srcIdx % mz.width() != 0));

    // std::cout << valid;
    testing::InitGoogleTest();
    return RUN_ALL_TESTS();
}