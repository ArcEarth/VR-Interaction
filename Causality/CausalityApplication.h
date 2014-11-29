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
	class CameraControlLogic : public Platform::IAppComponent , public DirectX::Scene::ITimeAnimatable, public Platform::IKeybordInteractive, public Platform::ICursorInteractive
	{
	public:
		CameraControlLogic(ICameraBase* pCamera)
		{
			m_pCamera = pCamera;
		}
		// Inherited via ITimeAnimatable
		virtual void UpdateAnimation(StepTimer const & timer) override;

		// Inherited via IKeybordInteractive
		virtual void OnKeyDown(const KeyboardEventArgs & e) override;
		virtual void OnKeyUp(const KeyboardEventArgs & e) override;

		// Inherited via ICursorInteractive
		virtual void OnMouseButtonDown(const CursorButtonEvent & e) override;
		virtual void OnMouseButtonUp(const CursorButtonEvent & e) override;
		virtual void OnMouseMove(const CursorMoveEventArgs & e) override;
	public:
		float											Speed = 0.8f;

	private:
		ICameraBase*									m_pCamera= nullptr;
		bool											IsTrackingCursor = false;
		DirectX::Vector3								CameraVeclocity;
		DirectX::Vector3								CameraAngularVeclocity;
	};

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

		DirectX::Scene::ICameraBase *GetPrimaryCamera()
		{
			return m_pPrimaryCamera.get();
		}

		// Inherited via IDeviceNotify
		virtual void OnDeviceLost() override;
		virtual void OnDeviceRestored() override;

		boost::filesystem::path	GetResourcesDirectory() const;
		void SetResourcesDirectory(const std::wstring& dir);

		void RegisterComponent(std::unique_ptr<Platform::IAppComponent> &&pComponent);

		void XM_CALLCONV RenderToView(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection);
		void OnCursorMove_RotateCamera(const Platform::CursorMoveEventArgs&e);
		Platform::Fundation::Event<const DirectX::StepTimer&> TimeElapsed;
		//void NotifyChildrenCursorButtonDown(const CursorButtonEvent&e);
	protected:
		// Devices & Resources
		boost::filesystem::path							ResourceDirectory;

		std::shared_ptr<Platform::DebugConsole>			pConsole;
		std::shared_ptr<Platform::NativeWindow>			pWindow;
		std::shared_ptr<DirectX::DeviceResources>		pDeviceResources;
		std::shared_ptr<Platform::Devices::OculusRift>	pRift;
		std::shared_ptr<Platform::Devices::LeapMotion>	pLeap;

		std::unique_ptr<DirectX::Scene::ICameraBase>	m_pPrimaryCamera;
		std::vector<std::unique_ptr<Platform::IAppComponent>> Components;

		// Rendering loop timer.
		DirectX::StepTimer m_timer;
	};

}
