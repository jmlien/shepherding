

#pragma once
#include <vector>
#include "Point.h"
#include "shepherding_base.h"
using namespace mathtool;

std::vector<Point2d> intersection(const CFlockState& a, const CFlockState& b);
std::vector<Point2d> intersections(const CFlockState& a, const CFlockState& b);
Vector2d parallelComponent(const Vector2d& direction, const Vector2d& collision_vector);


