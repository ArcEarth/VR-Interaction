#pragma once
#include <wrl/client.h>
#include <d3d11_2.h>
#include <d2d1_2.h>
#include <d2d1effects_1.h>
#include <dwrite_2.h>
#include <wincodec.h>
#include <DirectXColors.h>
#include <DirectXMath.h>
#include <memory>

#include "NativeWindow.h"
#include <iostream>
#include "Common\DeviceResources.h"
#include "Common\Renderable.h"
#include "NativeWindow.h"
#include "OculusRift.h"
#include "PrimaryCamera.h"
#include "LeapMotion.h"
#include "Kinect.h"
#include <boost\filesystem.hpp>
#include "Scene.h"

//extern std::unique_ptr<Causality::DXAppMain> m_main;

namespace Causality
{

	class App : public Application, public DirectX::IDeviceNotify
	{
	public:
		static App* Current() {
			return static_cast<App*>(Application::Current.get());
		}

		App();
		~App();

		// Inherited via Application
		virtual void OnStartup(const std::vector<std::string>& args) override;
		virtual void OnExit() override;
		virtual void OnIdle() override;

		// Inherited via IDeviceNotify
		virtual void OnDeviceLost() override;
		virtual void OnDeviceRestored() override;

		boost::filesystem::path	GetResourcesDirectory() const;
		void SetResourcesDirectory(const std::wstring& dir);

		void RegisterComponent(ICursorInteractive *pComponent);
		void RegisterComponent(IKeybordInteractive *pComponent);
		void RegisterComponent(IUserHandsInteractive *pComponent);
		void RegisterComponent(IAppComponent *pComponent);
		void UnregisterComponent(IAppComponent *pComponent);
		void XM_CALLCONV RenderToView(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection);
		Event<const DirectX::StepTimer&> TimeElapsed;
		//void NotifyChildrenCursorButtonDown(const CursorButtonEvent&e);
	protected:
		// Devices & Resources
		boost::filesystem::path							ResourceDirectory;

		// System resources
		std::shared_ptr<DebugConsole>					pConsole;
		std::shared_ptr<NativeWindow>					pWindow;
		std::shared_ptr<DirectX::DeviceResources>		pDeviceResources;

		RenderDevice									pDevice;
		RenderContext									pContext;

		// Extern Devices
		std::shared_ptr<Devices::OculusRift>			pRift;
		std::shared_ptr<Devices::KinectSensor>			pKinect;
		std::shared_ptr<Devices::LeapMotion>			pLeap;

		// Application Logic object
		std::vector<std::unique_ptr<IAppComponent>>		Components;
		std::map<IAppComponent*, std::vector<EventConnection>> ComponentsEventRegisterations;


		// Rendering loop timer.
		DirectX::StepTimer m_timer;

		// Should be the first thing to destroy
		std::vector<std::unique_ptr<Scene>>				Scenes;

	};

}
