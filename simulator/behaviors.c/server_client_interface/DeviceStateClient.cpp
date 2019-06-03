
// parts of requestDeviceState() fall under:

//
// client.cpp
// ~~~~~~~~~~
//
// Copyright (c) 2003-2008 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//


#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>

#include "DeviceStateClient.h"
#include "DeviceState.h"
#include "devicestate_cs_common.h"


using boost::asio::ip::tcp;
using std::string;
using std::cout;
using std::endl;



DeviceStateClient::
DeviceStateClient(string hostname, string_to_void_func callback_func, int waitingPeriod)
{
    m_hostname = hostname;
    std::sprintf(m_port_str, "%d", SERVER_PORT);
    m_callback_func = callback_func;
    m_waitingPeriod = waitingPeriod;
}


string
DeviceStateClient::
requestDeviceState()
{
    string deviceState;
    bool problemsEncountered = false;
    
    try
    {
        boost::asio::io_service io_service;

        tcp::resolver resolver(io_service);
        tcp::resolver::query query(m_hostname, m_port_str);
        
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        tcp::resolver::iterator end;

        tcp::socket socket(io_service);
        boost::system::error_code error = boost::asio::error::host_not_found;
        while (error && endpoint_iterator != end)
        {
            socket.close();
            socket.connect(*endpoint_iterator++, error);
        }
        if ( error )
            throw boost::system::system_error(error);

        while ( true )
        {
            boost::array<char, 256> buf;
            boost::system::error_code error;

            size_t len = socket.read_some(boost::asio::buffer(buf), error);

            if (error == boost::asio::error::eof)
                break; // Connection closed cleanly by peer.
            else if (error)
                throw boost::system::system_error(error); // Some other error.

            deviceState.append(buf.data(), len);
        }
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        problemsEncountered = true;
    }

    if (problemsEncountered)
        return "";
  
    return deviceState;
}



void
DeviceStateClient::
pollContinuously()
{
    DeviceState devState(0, 0);
    string serialState = requestDeviceState();
    
    
    while (serialState.size() != 0)
    {
        (*m_callback_func)(serialState);
        boost::this_thread::sleep(boost::posix_time::milliseconds(m_waitingPeriod));
        serialState = requestDeviceState();
    }
}


/*

void callback(std::string serialState)
{
    cout << "Updating state: ";
    DeviceState devState(serialState);
    cout << devState << endl;
}



int main(int argc, char* argv[])
{
    DeviceStateClient client("127.0.0.1", callback);
    client.pollContinuously();

    return 0;
}
*/
