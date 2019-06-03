#ifndef DEVICESTATE_H
#define DEVICESTATE_H

#include <string>
#include <iostream>
#include <vector>

class DeviceState
{
//    using namespace std;

    public:
        DeviceState()                    { addPoint(0.0, 0.0); }
        DeviceState(double x, double y)  { addPoint(  x,   y); }

        DeviceState(std::string state);

//        DeviceState & operator=(const DeviceState & deviceState);
        
        std::string serialize();
        void instantiate(std::string state);

        size_t size() const                  { return points.size(); };

        double getX(int i)  const            { return points.at(i).at(0); }
        double getY(int i)  const            { return points.at(i).at(1); }

        double getX() const                  { return getX(0); }
        double getY() const                  { return getY(0); }

/*
        void setX(int i, double x)             { points.at(i).at(0) = x;  }
        void setY(int i, double y)             { points.at(i).at(1) = y;  }

        void setX(double x)             { setX(0, x);  }
        void setY(double y)             { setY(0, y);  }
*/
        void addPoint(double x, double y);

        void clear()                    { points.clear(); }

    private:
        std::vector<std::vector<double> >  points;
//        double m_x;
//        double m_y;
};

std::ostream & operator<<(std::ostream & os, const DeviceState & state);

#endif
