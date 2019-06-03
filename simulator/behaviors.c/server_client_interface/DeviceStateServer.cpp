// portions of run() fall under:

// server.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <fstream>

#include "DeviceState.h"
#include "DeviceStateServer.h"
#include "devicestate_cs_common.h"

using namespace std;

using boost::asio::ip::tcp;



string
DeviceStateServer::
read_from_file(const string filename)
{
    string line;
    if (m_fin == 0)
        m_fin = new ifstream(filename.c_str());
    
    if (getline(*m_fin, line))
        return line;
    else
        return "";
}



void
DeviceStateServer::
updateState(DeviceState & deviceState)
{
    if ( ! m_currentState )
        m_currentState = new DeviceState(0, 0);
    
    *m_currentState = deviceState;
}



void
DeviceStateServer::
setInputFilename(const string filename)
{
    m_filename = filename;
    
    if (m_fin)
    {
        m_fin->close();
        m_fin = NULL;
    }   
}




/** Keep serving connection attempts requesting the current state until
 * terminate() is called, or the input file giving the next state is exhausted.
 */
void
DeviceStateServer::
run()
{
    string message;
    
    try
    {
        boost::asio::io_service io_service;
        tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), SERVER_PORT));

        while (true)
        {
            tcp::socket socket(io_service);
            acceptor.accept(socket);

            if (m_inputSource == FILE_INPUT)
            {
                message = read_from_file(m_filename) + "\n";

                // Terminate once the end of the file has been reached.
                if (! m_fin->good())
                {
                    cout << "END OF FILE. Terminating server..." << endl;
                    return;
                }
            }

            else if (m_inputSource == MANUAL_INPUT)
            {
                message = m_currentState->serialize();
            }
            
                    

            boost::system::error_code ignored_error;
            boost::asio::write(socket, boost::asio::buffer(message),
                               boost::asio::transfer_all(), ignored_error);
        }
    }
    catch (exception& e)
    {
        cerr << e.what() << endl;
    }
}

/*

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
             << "  ./DeviceStateServer -manual-input\n"
             << "               # to allow manual control of position\n"
             << "  ./DeviceStateServer -file-input [INPUT_FILE]\n"
             << "               # automatic control based on given file\n"
             << "               # (see position_file.txt for an example)\n" << endl;
        return 1;
    }

    
    server.run();
}
*/
