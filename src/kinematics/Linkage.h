#pragma once

#include <Eigen/Core>
#include <array>

using Eigen::Vector2d;

/*
 *
 *  *               o < p2 = end-effector position
 *                 /
 *                / bar2
 *   a1    bar1  /
 *   o----------o < a2
 *   ^          ^
 *   p0         p1
 */
struct Linkage
{
    Vector2d p0 = {1, -1.2}; // the first point of the linkage
    double length[2] = {1, 1.5}; // the lengths of bar1 and bar2
};
