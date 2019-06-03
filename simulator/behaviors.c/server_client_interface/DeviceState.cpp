
#include <sstream>
#include "DeviceState.h"

using namespace std;

/** Given a string containing a state representing a device state object,
 * instantiate the object. */
DeviceState::
DeviceState(string state)
{
    instantiate(state);
}



string
DeviceState::
serialize()
{
    ostringstream str_output;
    str_output << size() << " ";
    for (size_t i = 0; i < size(); i++)
        str_output << getX(i) << " " << getY(i) << " ";
    return str_output.str();
}



void
DeviceState::
instantiate(string state)
{
    stringstream str_input(state);

    points.clear();

    int num_points;
    double x, y;

    // Determine how many points we'll be reading.
    str_input >> num_points;

    // Read that many points
    for (int i = 0; i < num_points; i++)
    {
        str_input >> x;
        str_input >> y;
        addPoint(x, y);
    }
}


void
DeviceState::
addPoint(double x, double y)
{
        vector<double> p;

        p.push_back(x);
        p.push_back(y);

        points.push_back(p);
}



std::ostream & operator<<(std::ostream & os, const DeviceState & state)
{
    os << state.getX() << " " << state.getY();
    return os;
}


/*
DeviceState &
DeviceState::
operator=(const DeviceState & deviceState)
{
    this->m_x = deviceState.m_x;
    this->m_y = deviceState.m_y;
    return *this;
}
*/


/*
#include <iostream>
using namespace std;
int main(char argc, char * argv[])
{
    int x, y;
    cout << "Input a pair of numbers: ";
    cin >> x >> y;

    DeviceState d1(x, y);
    cout << "d1.getX(): " << d1.getX() << endl
         << "d1.getY(): " << d1.getY() << endl;

    cout << "d1.getState(): " << d1.getState() << endl;

    DeviceState d2(d1.getState());
    cout << "d2.getX(): " << d2.getX() << endl
         << "d2.getY(): " << d2.getY() << endl;
   
    return 0;
}
*/
