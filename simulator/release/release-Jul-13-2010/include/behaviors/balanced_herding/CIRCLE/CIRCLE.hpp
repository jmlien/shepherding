

#ifndef BALANCED_CIRCLE_CIRCLE_HPP
#define BALANCED_CIRCLE_CIRCLE_HPP


#include <cassert>
#include <vector>
#include "../Balanced-functions.hpp"


namespace Balanced {


struct CIRCLE
{
    Point2d center;
    float radius;
    
    CIRCLE() { }
    CIRCLE(const Point2d& center, const float radius) : 
        center(center), 
        radius(radius)
    {
        // do nothing
    }
    
    static CIRCLE Enclosing(const std::vector<Point2d>& points)
    {
        const int size = points.size();
        assert(size > 0);
        
        float diameter = 0.0f;
        int max_a = 0, max_b = 0;
        for(int i = 0; i < size; i++)
        {
            for(int j = i + 1; j < size; j++)
            {
                const float distance = (points[i] - points[j]).normsqr();
                if(distance > diameter)
                {
                    diameter = distance;
                    max_a = i;
                    max_b = j;
                }
            }
        }
        const float center_x = 0.5f*(points[max_a][0] + points[max_b][0]);
        const float center_y = 0.5f*(points[max_a][1] + points[max_b][1]);
        return CIRCLE(Point2d(center_x, center_y), 0.5f*sqrt(diameter));
    }
    
    static CIRCLE Enclosing(const std::vector<CFlockState*>& sheep)
    {
        const int size = sheep.size();
        assert(size > 0);
        std::vector<Point2d> points;
        points.reserve(size);
        for(int i = 0; i < size; i++)
        {
            points.push_back(sheep[i]->getPos());
        }
        return CIRCLE::Enclosing(points);
    }
    
    bool Inside(const Point2d& point)
    {
        return ((point - this->center).norm() <= this->radius);
    }
    
    std::vector<Point2d> Intersection(const Point2d& point, const Vector2d& line)
    {
        Point2d middle = Balanced::ClosestPointOnLine(this->center, point, line);
        // pythagorean theorum: b = sqrt(c^2 - a^2)
        const float distance = sqrt(this->radius*this->radius - (middle - this->center).normsqr());
        const Vector2d normalized_line = line.normalize();
        std::vector<Point2d> intersection(2);
        intersection[0] = middle + (normalized_line*distance);
        intersection[1] = middle - (normalized_line*distance);
        return intersection;
    }
};


} // end Balanced

    
#endif // BALANCED_CIRCLE_CIRCLE_HPP


