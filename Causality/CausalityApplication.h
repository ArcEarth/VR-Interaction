#pragma once
#include "pch.h"
#include "NativeWindow.h"
#include <iostream>
#include "Common\DeviceResources.h"
#include "Common\Renderable.h"
#include "NativeWindow.h"
#include "OculusRift.h"
#include "Player.h"
#include "LeapMotion.h"
#include <boost\filesystem.hpp>

//extern std::unique_ptr<Causality::DXAppMain> m_main;

namespace Causality
{
	class App : public Platform::Application, public DirectX::IDeviceNotify
	{
	public:
		static App* Current() {
			return static_cast<App*>(Platform::Application::Current.get());
		}

		App();
		~App();

		// Inherited via Application
		virtual void OnStartup(Platform::Array<Platform::String^>^ args) override;
		virtual void OnExit() override;
		virtual void OnIdle() override;

		// Inherited via IDeviceNotify
		virtual void OnDeviceLost() override;
		virtual void OnDeviceRestored() override;

		boost::filesystem::path	GetResourcesDirectory() const;
		void SetResourcesDirectory(const std::wstring& dir);

		void RegisterScene(DirectX::Scene::IRenderable* pScene);

		void XM_CALLCONV RenderToView(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection);

		Platform::Fundation::Event<const DirectX::StepTimer&> TimeElapsed;
		//void NotifyChildrenCursorButtonDown(const CursorButtonEvent&e);
	protected:
		// Devices & Resources
		boost::filesystem::path							ResourceDirectory;

		std::shared_ptr<Platform::NativeWindow>			pWindow;
		std::shared_ptr<DirectX::DeviceResources>		pDeviceResources;
		std::shared_ptr<Platform::Devices::OculusRift>	pRift;
		std::shared_ptr<Platform::Devices::LeapMotion>	pLeap;

		std::unique_ptr<DirectX::Scene::ICameraBase>	m_pPrimaryCamera;
		std::vector<std::unique_ptr<DirectX::Scene::IRenderable>> Scenes;

		// Rendering loop timer.
		DirectX::StepTimer m_timer;
	};

}
