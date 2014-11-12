#pragma once
#include <Windows.h>
#include "pch.h"

namespace Platform
{
	ref class Window;
	class NativeWindow;

	class Application
	{
	public:
		virtual void Initialize();
		virtual void SetWindow(const std::shared_ptr<NativeWindow>& window);
		virtual void Load(Platform::String^ entryPoint);
		virtual void Run();
		virtual void Uninitialize();

		// Application lifecycle event handlers.
		virtual void OnActivated(Windows::ApplicationModel::Activation::IActivatedEventArgs^ args);
		virtual void OnSuspending(Platform::Object^ sender, Windows::ApplicationModel::SuspendingEventArgs^ args);
		virtual void OnResuming(Platform::Object^ sender, Platform::Object^ args);

	public:
		static std::weak_ptr<Application> Current;
		static HINSTANCE	Instance();
		static std::map<HWND, std::weak_ptr<NativeWindow>> WindowsLookup;
		static LRESULT CALLBACK WndProc(HWND, UINT, WPARAM, LPARAM);
	protected:
		static HINSTANCE	s_hInstance;

	};

	class IWindow abstract
	{
	public:
		virtual void Initialize(Platform::String^ title, unsigned int width, unsigned int height, bool fullScreen = false) = 0;
		virtual void Show() = 0;
		virtual void Hide() = 0;
		virtual void Focus() = 0;
		virtual void Close() = 0;
		virtual void Minimize() = 0;
		virtual void Maximize() = 0;
		virtual bool IsFullScreen() const = 0;
		virtual void EnterFullScreen() = 0;
		virtual void ExitFullScreen() = 0;
		virtual void OnMouseMove() = 0;
		virtual void OnKeyDown() = 0;
		virtual void OnKeyUp() = 0;
		virtual void OnMouseButtonDown() = 0;
		virtual void OnMouseButtonUp() = 0;
	};

	class NativeWindow : IWindow
	{
	public:
		NativeWindow();
		~NativeWindow();
		void Initialize(Platform::String^ title, unsigned int width, unsigned int height, bool fullScreen = false);

		void Show();
		void Hide();
		void Focus();

		void Close();

		void Minimize();
		void Maximize();

		bool IsFullScreen() const;

		void EnterFullScreen();
		void ExitFullScreen();
		void OnMouseMove() {}
		void OnKeyDown() {}
		void OnKeyUp() {}
		void OnMouseButtonDown(){}
		void OnMouseButtonUp(){}

		HWND Handle() const
		{
			return m_hWnd;
		}

		HINSTANCE ApplicationInstance() const
		{
			return m_hInstance;
		}

		RECT Bound() const
		{
			RECT bound;
			GetWindowRect(m_hWnd, &bound);
			return bound;
		}

		void SetWindow(Window^);

		virtual LPARAM CALLBACK MessageHandler(UINT umsg, WPARAM wparam, LPARAM lparam);
		//virtual void OnActive();
		//virtual void OnClosed();
		//virtual void OnResize();
		//virtual void OnDeactive();
		//virtual void OnKeyDown(Windows::System::VirtualKey key);
		//virtual void OnKeyUp(Windows::System::VirtualKey key);
		//virtual void OnMouseMove();

	private:
		Platform::String^	m_Title;
		HWND				m_hWnd;
		HINSTANCE			m_hInstance;
		bool				m_FullScreen;

	};

	ref class Window sealed
	{
	public:
		Window();
		//Window(std::unique_ptr<NativeWindow> &&native);
		virtual ~Window();
		void Initialize(Platform::String^ title, unsigned int screenWidth, unsigned int screenHeight, bool fullScreen = false);
		void Show();
		void Focus();
		void Close();

		event Windows::Foundation::TypedEventHandler<Window^, Windows::UI::Core::WindowActivatedEventArgs^> ^Activated{
			Windows::Foundation::EventRegistrationToken add(Windows::Foundation::TypedEventHandler<Window^, Windows::UI::Core::WindowActivatedEventArgs^>^ value);
			void remove(Windows::Foundation::EventRegistrationToken token);
			void raise(Window^, Windows::UI::Core::WindowActivatedEventArgs^);
		}

	private:
		std::unique_ptr<NativeWindow>	m_Native;
	};

}
