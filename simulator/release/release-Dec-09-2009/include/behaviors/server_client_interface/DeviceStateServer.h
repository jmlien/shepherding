#ifndef DEVICESTATE_SERVER
#define DEVICESTATE_SERVER

#include <fstream>
#include <string>

class DeviceStateServer
{
    public:
        enum InputSource { FILE_INPUT, MANUAL_INPUT };

        DeviceStateServer()
        {
            m_fin = NULL;
            m_inputSource = FILE_INPUT;
            m_currentState = new DeviceState(0, 0);
            m_filename = "";
        }
        
        void run();
        void updateState(DeviceState & state);
        void setInputSource(InputSource source)
        {
            m_inputSource = source;
        }

        void setInputFilename(const std::string filename);
            
        
        
    private:
        std::string read_from_file(const std::string filename);

        std::ifstream * m_fin;  // File INput stream
        DeviceState *   m_currentState;
        InputSource     m_inputSource;
        std::string     m_filename;
};

#endif
