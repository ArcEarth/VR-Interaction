#include "stdafx.h"
#include "MultiMouseControler.h"
#include <WinUser.h>

static bool is_rm_rdp_mouse(char cDeviceString[])
{
	int i;
	char cRDPString[] = "\\??\\Root#RDP_MOU#0000#";

	if (strlen(cDeviceString) < 22) {
		return 0;
	}

	for (i = 0; i < 22; i++) {
		if (cRDPString[i] != cDeviceString[i]) {
			return 0;
		}
	}  

	return 1;
}

MultiMouseControler::MultiMouseControler(bool IncludeRomoteDesktopMouse , HWND hWnd)
{
	IncludeRomoteDesktopMouse = false;
	m_Initialized = false;
	Initialize(IncludeRomoteDesktopMouse , hWnd);
}


bool MultiMouseControler::Initialize(bool IncludeRomoteDesktopMouse , HWND hWnd){
	unsigned int nInputDevices =0, i;
	PRAWINPUTDEVICELIST pRawInputDeviceList ={0};

	unsigned int nSize = 1024;
	char psName[1024] = {0}; // A fix size buffer for storing name string

	if (m_Initialized) {
		fprintf(stderr, "WARNING: rawmouse init called after initialization already completed.");
		m_Initialized = true;
		return 0;
	}

	// 1st call to GetRawInputDeviceList: Pass nullptr to get the number of devices.
	if ( GetRawInputDeviceList(nullptr, &nInputDevices, sizeof(RAWINPUTDEVICELIST)) != 0) {
		fprintf(stderr, "ERROR: Unable to count raw input devices.\n");
		return 0;
	}
	// Allocate the array to hold the DeviceList
	if ((pRawInputDeviceList = ((PRAWINPUTDEVICELIST)malloc(sizeof(RAWINPUTDEVICELIST) * nInputDevices))) == nullptr) {
		fprintf(stderr, "ERROR: Unable to allocate memory for raw input device list.\n");
		return 0;
	}
	// 2nd call to GetRawInputDeviceList: Pass the pointer to our DeviceList and GetRawInputDeviceList() will fill the array
	if (GetRawInputDeviceList(pRawInputDeviceList, &nInputDevices, sizeof(RAWINPUTDEVICELIST)) == -1)  {
		fprintf(stderr, "ERROR: Unable to get raw input device list.\n");
		return 0;
	}

	// Loop through all devices and count the mice
	for (i = 0; i < nInputDevices; i++) {
		if (pRawInputDeviceList[i].dwType == RIM_TYPEMOUSE) {
			///* Get the device name and use it to determine if it's the RDP Terminal Services virtual device. */
			//// 2nd call to GetRawInputDeviceInfo: Pass our pointer to get the device name
			//if ((int)GetRawInputDeviceInfo(pRawInputDeviceList[i].hDevice, RIDI_DEVICENAME, nullptr, &nSize) < 0) {
			//	fprintf(stderr, "ERROR: Unable to get raw input device name.\n");
			//	return 0;
			//} 
			//if ((psName = (char *)malloc(sizeof(TCHAR) * nSize)) == NULL)  {
			//	fprintf(stderr, "ERROR: Unable to allocate memory for device name.\n");
			//	return 0;
			//}
			nSize = 1024;
			auto hr = (int)GetRawInputDeviceInfoA(pRawInputDeviceList[i].hDevice, RIDI_DEVICENAME, psName, &nSize);
			if (hr < 0) {
				fprintf(stderr, "ERROR: Unable to get raw input device name.\n");
				return 0;
			} 

			bool is_rdp = is_rm_rdp_mouse(psName);
			if ((!is_rdp) || IncludeRomoteDesktopMouse) {
				m_Devices.emplace_back();
				Device& device = m_Devices.back();
				device.hDevice = pRawInputDeviceList[i].hDevice;
				device.Name = psName;
				device.Priority = 2;
				device.EventFlag = 0;
				device.State = 0;
				device.WheelDelta = 0;
				m_DeviceToIndexMap[device.hDevice] = m_Devices.size()-1;
			}
		}
	}

	// free the RAWINPUTDEVICELIST
	free(pRawInputDeviceList);

	// finally, register to recieve raw input WM_INPUT messages
	RAWINPUTDEVICE Rid = { 0x01, 0x02 , 0 , hWnd}; // Register only for mouse messages from wm_input.  

	// Register to receive the WM_INPUT message for any change in mouse (buttons, wheel, and movement will all generate the same message)
	if (!RegisterRawInputDevices(&Rid, 1, sizeof (Rid)))
	{
		fprintf(stderr, "ERROR: Unable to register raw input (2).\n");
		return false;
	}

	m_Initialized = true;
	return true;  
}

MultiMouseControler::~MultiMouseControler(void)
{
}

unsigned int MultiMouseControler::GetDeviceIndexByName(const std::string & deviceName) const
{
	for (unsigned int i = 0; i < m_Devices.size(); i++)
	{
		if (m_Devices[i].Name.find(deviceName) != std::string::npos)
			return i;
	}
	return -1;
}



int MultiMouseControler::MouseInputDispicher(HWND hWnd,HRAWINPUT input)
{
	LPBYTE lpb;
	unsigned int dwSize = 0;

	if (GetRawInputData(input, RID_INPUT, nullptr, &dwSize, sizeof(RAWINPUTHEADER)) == -1) {
		fprintf(stderr, "ERROR: Unable to add to get size of raw input header.\n");
		return -2;
	}
	lpb = (LPBYTE)malloc(sizeof(LPBYTE) * dwSize);
	if (lpb == nullptr) {
		fprintf(stderr, "ERROR: Unable to allocate memory for raw input header.\n");
		return -2;
	} 
	if (GetRawInputData(input, RID_INPUT, lpb, &dwSize, sizeof(RAWINPUTHEADER)) != dwSize ) {
		fprintf(stderr, "ERROR: Unable to add to get raw input header.\n");
		return -2;
	} 

	if (!((RAWINPUT*)lpb)->data.mouse.usButtonFlags) 
		return -1;


	auto flag = TranslateWINPINT((RAWINPUT*)lpb);

	m_Syncronizer.Reset();

	free(lpb); 

	return flag.first;
}

std::pair<short,short> MultiMouseControler::TranslateWINPINT(PRAWINPUT raw)
{
	HANDLE handle = raw->header.hDevice;
	auto flag = raw->data.mouse.usButtonFlags;
	auto index = m_DeviceToIndexMap[handle];
	auto &device = m_Devices[index];

	if (flag)
		device.SteadyTimer.Reset();

	if (flag & RI_MOUSE_WHEEL)			
	{
		device.EventFlag |= WheelEvent;
		device.WheelDelta += static_cast<short>(raw->data.mouse.usButtonData) / WHEEL_DELTA;
	}

	// 0x38 == mask for all the button flag
	if (flag & 0x38) device.EventFlag |= ButtonEvent;

	if (flag & RI_MOUSE_BUTTON_1_DOWN)	device.State |= 0x1;
	if (flag & RI_MOUSE_BUTTON_1_UP)	device.State &= ~0x1;
	if (flag & RI_MOUSE_BUTTON_2_DOWN)	device.State |= 0x2;
	if (flag & RI_MOUSE_BUTTON_2_UP)	device.State &= ~0x2;
	if (flag & RI_MOUSE_BUTTON_3_DOWN)	device.State |= 0x4;
	if (flag & RI_MOUSE_BUTTON_3_UP)	device.State &= ~0x4;


	return std::make_pair(index,flag);
}
