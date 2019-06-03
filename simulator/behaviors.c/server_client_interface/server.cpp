#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <fstream>

#include "DeviceState.h"
#include "DeviceStateServer.h"
// #include "common.h"

using namespace std;

using boost::asio::ip::tcp;



void manuallyUpdateState(DeviceStateServer * server)
{
    DeviceState devState;
    string line;

    if (server == NULL)
        return;

    cout << "Please enter a line containing two numbers: x-position, y-position" << endl
         << "e.g.     .5  .5" << endl
         << "Note that the positions will be adapted to the dimensions at the" << endl
         << "client end, and must therefore be between 0 and 1." << endl;
    
    while ( getline(cin, line) )
    {
        devState.instantiate(line);
        server->updateState(devState);
    }
}



int main(int argc, char * argv[])
{
    DeviceStateServer server;
    string filename;
    bool configured = false;

    
    for ( int i = 1; i < argc; i++)
    {
        if      (!strcmp(argv[i], "-manual-input"))
        {
            configured = true;
            server.setInputSource( DeviceStateServer::MANUAL_INPUT );
            boost::thread second_thread( boost::bind( manuallyUpdateState, & server ) );
        }
        else if (!strcmp(argv[i], "-file-input"))
        {
            configured = true;
            server.setInputSource( DeviceStateServer::FILE_INPUT );
            server.setInputFilename(argv[++i]);
        }
    }
    
    
    if ( ! configured )
    {
        cerr << "usage: Run using either of the following methods:\n\n"
             << "  ./server -manual-input\n"
             << "               # to allow manual control of position\n"
             << "  ./server -file-input [INPUT_FILE]\n"
             << "               # automatic control based on given file\n"
             << "               # (see position_file.txt for an example)\n" << endl;
        return 1;
    }

    
    server.run();
}
