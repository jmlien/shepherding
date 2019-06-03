#ifdef _WIN32
#include "CXBOXController.h"

CXBOXController::CXBOXController(int playerNumber)
{
	// Set the Controller Number
	_controllerNum = playerNumber - 1;
}


XINPUT_KEYSTROKE CXBOXController::GetKeyStroke()
{
	// Zeroise the state
	ZeroMemory(&_keystroke, sizeof(XINPUT_KEYSTROKE));

	// Get the state
	XInputGetKeystroke(_controllerNum, 0, &_keystroke);

	return _keystroke;
}


XINPUT_STATE CXBOXController::GetState()
{
	// Zeroise the state
	ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));

	// Get the state
	XInputGetState(_controllerNum, &_controllerState);

	return _controllerState;
}

XINPUT_BATTERY_INFORMATION CXBOXController::GetBatteryState()
{
	// Zeroise the state
	ZeroMemory(&_batteryState, sizeof(XINPUT_BATTERY_INFORMATION));

	// Get the state
	XInputGetBatteryInformation(_controllerNum, BATTERY_DEVTYPE_GAMEPAD, &_batteryState);

	return _batteryState;
}


bool CXBOXController::IsConnected()
{
	// Zeroise the state
	ZeroMemory(&_controllerState, sizeof(XINPUT_STATE));

	// Get the state
	DWORD Result = XInputGetState(_controllerNum, &_controllerState);

	if(Result == ERROR_SUCCESS)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void CXBOXController::Vibrate(int leftVal, int rightVal)
{
	// Create a Vibraton State
	XINPUT_VIBRATION Vibration;

	// Zeroise the Vibration
	ZeroMemory(&Vibration, sizeof(XINPUT_VIBRATION));

	// Set the Vibration Values
	Vibration.wLeftMotorSpeed = leftVal;
	Vibration.wRightMotorSpeed = rightVal;

	// Vibrate the controller
	XInputSetState(_controllerNum, &Vibration);
}
#endif