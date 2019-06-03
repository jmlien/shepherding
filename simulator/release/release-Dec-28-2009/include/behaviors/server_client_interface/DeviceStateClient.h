#ifndef DEVICESTATE_CLIENT_H
#define DEVICESTATE_CLIENT_H

#include <string>

typedef  void (* string_to_void_func )(std::string);

class DeviceStateClient
{
    public:
        DeviceStateClient(std::string hostname, string_to_void_func callback_func, int waitingPeriod);
        std::string requestDeviceState();
        void pollContinuously();

    private:
        std::string m_hostname;
        char m_port_str[10];
        int m_waitingPeriod;    // time to wait between polls, in seconds
        string_to_void_func  m_callback_func;
};


#endif
