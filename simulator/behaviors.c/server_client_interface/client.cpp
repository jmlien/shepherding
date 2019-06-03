/** Starts up a DeviceStateClient to test out the positions sent
 * by the DeviceStateServer
 */

#include <iostream>

#include "DeviceStateClient.h"
#include "DeviceState.h"


/** Polls the current device state / positions from the DeviceStateServer
 * every second.
 */
void callback(std::string serialState)
{
    std::cout << "Polling the current state: ";
    DeviceState devState(serialState);
    std::cout << devState << std::endl;
}



int main(int argc, char* argv[])
{
    int msCount = 1000;  // # of milliseconds to wait between polls
    DeviceStateClient client("127.0.0.1", callback, msCount);
    client.pollContinuously();

    return 0;
}
