#ifdef _WIN32
#ifndef _XBOX_CONTROLLER_H_
#define _XBOX_CONTROLLER_H_

// No MFC
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif 

// We need the Windows Header and the XInput Header
#include <windows.h>
#include <XInput.h>

// Now, the XInput Library
// NOTE: COMMENT THIS OUT IF YOU ARE NOT USING A COMPILER THAT SUPPORTS THIS METHOD OF LINKING LIBRARIES
#pragma comment(lib, "XInput.lib")

// XBOX Controller Class Definition
class CXBOXController
{

public:
	
	CXBOXController(int playerNumber);

	XINPUT_STATE GetState();
	XINPUT_BATTERY_INFORMATION GetBatteryState();
	XINPUT_KEYSTROKE GetKeyStroke();

	bool IsConnected();
	void Vibrate(int leftVal = 0, int rightVal = 0);

private:
	XINPUT_STATE _controllerState;
	XINPUT_BATTERY_INFORMATION _batteryState;
	XINPUT_KEYSTROKE _keystroke;

	int _controllerNum;
};

#endif // _XBOX_CONTROLLER_H_
#endif // _WIN32