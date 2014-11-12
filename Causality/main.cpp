// Source.cpp : Defines the entry point for the console application.
//
#include "pch.h"
#include <iostream>
#include <Leap.h>
#include "Common\DeviceResources.h"
#include "DXAppMain.h"
#include "NativeWindow.h"
#include "OculusRift.h"
#include "Player.h"

using namespace std;
using namespace Leap;
using namespace Platform;
using namespace Windows::Globalization;
using namespace concurrency;
using namespace Windows::ApplicationModel;
using namespace Windows::ApplicationModel::Core;
using namespace Windows::ApplicationModel::Activation;
using namespace Windows::UI::Core;
using namespace Windows::UI::Input;
using namespace Windows::System;
using namespace Windows::Foundation;
using namespace Windows::Graphics::Display;
using namespace Causality;
using namespace DirectX;

std::shared_ptr<NativeWindow> window;
std::shared_ptr<DirectX::DeviceResources> deviceResources;
std::unique_ptr<Causality::DXAppMain> m_main;

class SampleListener : public Listener {
public:
	virtual void onConnect(const Controller&);
	virtual void onFrame(const Controller&);

	bool isPrevTracked = false;
};

void SampleListener::onConnect(const Controller& controller) {
	std::cout << "Connected" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
	auto frame = controller.frame();
	if (frame.hands().count() > 0)
	{
		if (!isPrevTracked)
		{
			m_main->StartTracking();
			isPrevTracked = true;
		}
		auto hand = frame.hands().frontmost();
		m_main->TrackingUpdate(hand.palmPosition().x);
	}
	else
	{
		if (isPrevTracked)
		{
			m_main->StopTracking();
			isPrevTracked = false;
		}
	}
	//std::cout << "Frame available" << std::endl;
}

[Platform::MTAThread]
int main(Platform::Array<Platform::String^>^ args)
{
	Leap::Controller controller;
	SampleListener listener;
	controller.addListener(listener);

	window = make_shared<Platform::NativeWindow>();
	window->Initialize(ref new String(L"Causality"), 1280U, 720,false);
	deviceResources = make_shared<DirectX::DeviceResources>();
	deviceResources->SetNativeWindow(window->Handle());
	auto pRift = std::make_shared<Platform::Devices::OculusRift>();
	pRift->Initialize(window->Handle(), deviceResources.get());

	auto pPlayer = std::make_unique<Player>(pRift);
	pRift->DissmisHealthWarnning();

	m_main = make_unique<Causality::DXAppMain>(deviceResources);

	pPlayer->SetPosition(Fundation::Vector3(0.0f, 0.7f, 1.5f));
	pPlayer->FocusAt(Fundation::Vector3(0, 0, 0), Fundation::Vector3(0.0f, 1.0f, 0));

	MSG msg;
	bool done = false;
	while (!done)
	{
		// Handle the windows messages.
		if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		// If windows signals to end the application then exit out.
		if (msg.message == WM_QUIT)
		{
			done = true;
		}
		else
		{
			m_main->Update();
			deviceResources->SetCurrentOrientation(pRift->HeadPose().Orientation);

			pRift->BeginFrame();
			// Otherwise do the frame processing.
			for (int eye = 0; eye < 2; eye++)
			{
				pRift->EyeTexture((DirectX::Scene::EyesEnum) eye).SetAsRenderTarget(deviceResources->GetD3DDeviceContext(), pRift->DepthStencilBuffer());

				auto view = XMMatrixTranspose(pPlayer->GetViewMatrix((DirectX::Scene::EyesEnum) eye));
				auto projection = XMMatrixTranspose(pPlayer->GetProjectionMatrix((DirectX::Scene::EyesEnum) eye));
				m_main->m_sceneRenderer->UpdateViewMatrix(view);
				m_main->m_sceneRenderer->UpdateProjectionMatrix(projection);
				m_main->Render();
			}


			pRift->EndFrame();
		}
	}

	//std::cin.get();
	//auto calendar = ref new Calendar;
	//calendar->SetToNow();
	//wcout << "It's now " << calendar->HourAsPaddedString(2)->Data() << L":" <<
	//	calendar->MinuteAsPaddedString(2)->Data() << L":" <<
	//	calendar->SecondAsPaddedString(2)->Data() << endl;
	//Platform::Details::Console::WriteLine("Hello World");

	controller.removeListener(listener);
	//system("Pause");
	return 0;
}

//void OnActivated(Causality::Window ^sender, Windows::UI::Core::WindowActivatedEventArgs ^args)
//{
//	Platform::Details::Console::WriteLine("Lalalaa Demacia!");
//}
