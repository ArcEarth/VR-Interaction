#include "NativeWindow.h"

namespace Platform
{
	
	std::map<HWND, std::weak_ptr<NativeWindow>> Application::WindowsLookup;

	Window::Window()
	{
	}

	Window::~Window()
	{
	}

	NativeWindow::NativeWindow()
	{
	}

	void NativeWindow::Hide()
	{}

	void NativeWindow::Minimize()	{}
	void NativeWindow::Maximize()	{}

	bool NativeWindow::IsFullScreen() const	{
		return m_FullScreen;
	}

	void NativeWindow::EnterFullScreen() {}
	void NativeWindow::ExitFullScreen()	{}

	NativeWindow::~NativeWindow()
	{
		Close();
	}

	// Methods for Initialize APP Window
	LPARAM CALLBACK NativeWindow::MessageHandler(UINT umsg, WPARAM wparam, LPARAM lparam)
	{
		switch (umsg)
		{
			// Check if a key has been pressed on the keyboard.
			//case WM_KEYDOWN:
			//{
			//	// If a key is pressed send it to the input object so it can record that state.
			//	m_pInput->KeyDown((unsigned int) wparam);
			//	return 0;
			//}

			//	// Check if a key has been released on the keyboard.
			//case WM_KEYUP:
			//{
			//	// If a key is released then send it to the input object so it can unset the state for that key.
			//	m_pInput->KeyUp((unsigned int) wparam);
			//	return 0;
			//}

			//	//Handel the mouse(hand gesture glove) input
			//case WM_INPUT:
			//{
			//	m_pInput->ReadMouse(hwnd, (HRAWINPUT) lparam);
			//	return 0;
			//}
			// Any other messages send to the default message handler as our application won't make use of them.
		}
		return 0;
	}


	void NativeWindow::Initialize(Platform::String^ title, unsigned int screenWidth, unsigned int screenHeight, bool fullScreen)
	{
		WNDCLASSEX wc;
		DEVMODE dmScreenSettings;
		int posX, posY;

		// Get the instance of this application.
		m_hInstance = GetModuleHandle(NULL);

		// Give the application a name.
		m_Title = title;

		// Setup the windows class with default settings.
		wc.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;
		wc.lpfnWndProc = Application::WndProc;
		wc.cbClsExtra = 0;
		wc.cbWndExtra = 0;
		wc.hInstance = m_hInstance;
		wc.hIcon = LoadIcon(NULL, IDI_WINLOGO);
		wc.hIconSm = wc.hIcon;
		wc.hCursor = LoadCursor(NULL, IDC_ARROW);
		wc.hbrBackground = (HBRUSH) GetStockObject(WHITE_BRUSH);
		wc.lpszMenuName = NULL;
		wc.lpszClassName = m_Title->Data();
		wc.cbSize = sizeof(WNDCLASSEX);

		// Register the window class.
		RegisterClassEx(&wc);

		// Setup the screen settings depending on whether it is running in full screen or in windowed mode.
		if (fullScreen)
		{
			// If full screen set the screen to maximum size of the users desktop and 32bit.
			// Determine the resolution of the clients desktop screen.
			screenHeight = GetSystemMetrics(SM_CYSCREEN);
			screenWidth = GetSystemMetrics(SM_CXSCREEN);

			memset(&dmScreenSettings, 0, sizeof(dmScreenSettings));
			dmScreenSettings.dmSize = sizeof(dmScreenSettings);
			dmScreenSettings.dmPelsWidth = (unsigned long) screenWidth;
			dmScreenSettings.dmPelsHeight = (unsigned long) screenHeight;
			dmScreenSettings.dmBitsPerPel = 32;
			dmScreenSettings.dmFields = DM_BITSPERPEL | DM_PELSWIDTH | DM_PELSHEIGHT;

			// Change the display settings to full screen.
			ChangeDisplaySettings(&dmScreenSettings, CDS_FULLSCREEN);

			// Set the position of the window to the top left corner.
			posX = posY = 0;
		}
		else
		{
			posX = (GetSystemMetrics(SM_CXSCREEN) - screenWidth) / 2;
			posY = (GetSystemMetrics(SM_CYSCREEN) - screenHeight) / 2;
		}

		// Create the window with the screen settings and get the handle to it.
		m_hWnd = CreateWindowEx(WS_EX_APPWINDOW, wc.lpszClassName, wc.lpszClassName,
			WS_CLIPSIBLINGS | WS_CLIPCHILDREN | WS_POPUP,
			posX, posY, screenWidth, screenHeight, NULL, NULL, m_hInstance, NULL);

		// Bring the window up on the screen and set it as main focus.
		ShowWindow(m_hWnd, SW_SHOW);
		SetForegroundWindow(m_hWnd);
		SetFocus(m_hWnd);

		return;
	}

	void NativeWindow::Show()
	{
		ShowWindow(m_hWnd, SW_SHOW);
	}

	void NativeWindow::Focus()
	{
		SetForegroundWindow(m_hWnd);
		SetFocus(m_hWnd);
	}

	void NativeWindow::Close()
	{
		//// Show the mouse cursor.
		//ShowCursor(true);

		// Fix the display settings if leaving full screen mode.
		if (m_FullScreen)
		{
			ChangeDisplaySettings(NULL, 0);
		}

		// Remove the window.
		DestroyWindow(m_hWnd);
		m_hWnd = NULL;

		// Remove the application instance.
		UnregisterClass(m_Title->Data(), m_hInstance);
		m_hInstance = NULL;

		return;
	}


	LRESULT CALLBACK Application::WndProc(HWND hwnd, UINT umessage, WPARAM wparam, LPARAM lparam)
	{
		//return Application::CoreWindow->MessageHandler(umessage, wparam, lparam);
		std::shared_ptr<IWindow> window = nullptr;
		auto itr = WindowsLookup.find(hwnd);
		if (itr != WindowsLookup.end())
			window = itr->second.lock();
		switch (umessage)
		{
			// Check if the window is being destroyed.
		case WM_DESTROY:
		{
			PostQuitMessage(0);
			return 0;
		}

			// Check if the window is being closed.
		case WM_CLOSE:
		{
			PostQuitMessage(0);
			return 0;
		}
		//Check if a key has been pressed on the keyboard.
		case WM_KEYDOWN:
		{
			// If a key is pressed send it to the input object so it can record that state.
			if (window) window->OnKeyDown((unsigned char) wparam);
			//m_pInput->KeyDown((unsigned int) wparam);
			return 0;
		}

		// Check if a key has been released on the keyboard.
		case WM_KEYUP:
		{
			// If a key is released then send it to the input object so it can unset the state for that key.
			if (window) window->OnKeyUp((unsigned char) wparam);
			//m_pInput->KeyUp((unsigned int) wparam);
			return 0;
		}

		//Handel the mouse(hand gesture glove) input
		case WM_MOUSEMOVE:
		{
			if (window) window->OnMouseMove();
			return 0;
		}
		//Any other messages send to the default message handler as our application won't make use of them.
		// All other messages pass to the message handler in the system class.
		default:
		{
			return DefWindowProc(hwnd, umessage, wparam, lparam);
		}

		}
	}
}