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
		CameraControlLogic(DirectX::Scene::ICameraBase* pCamera);
		// Inherited via ITimeAnimatable
		virtual void UpdateAnimation(DirectX::StepTimer const & timer) override;

		// Inherited via IKeybordInteractive
		virtual void OnKeyDown(const Platform::KeyboardEventArgs & e) override;
		virtual void OnKeyUp(const Platform::KeyboardEventArgs & e) override;

		// Inherited via ICursorInteractive
		virtual void OnMouseButtonDown(const Platform::CursorButtonEvent & e) override;
		virtual void OnMouseButtonUp(const Platform::CursorButtonEvent & e) override;
		virtual void OnMouseMove(const Platform::CursorMoveEventArgs & e) override;
	public:
		float											Speed;

	private:
		DirectX::Scene::ICameraBase*					m_pCamera= nullptr;
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
		void UnregisterComponent(Platform::IAppComponent *pComponent);
		void XM_CALLCONV RenderToView(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection);
		void OnCursorMove_RotateCamera(const Platform::CursorMoveEventArgs&e);
		Platform::Fundation::Event<const DirectX::StepTimer&> TimeElapsed;
		//void NotifyChildrenCursorButtonDown(const CursorButtonEvent&e);
	protected:
		// Devices & Resources
		boost::filesystem::path							ResourceDirectory;

		// System resources
		std::shared_ptr<Platform::DebugConsole>			pConsole;
		std::shared_ptr<Platform::NativeWindow>			pWindow;
		std::shared_ptr<DirectX::DeviceResources>		pDeviceResources;

		// Extern Devices
		std::shared_ptr<Platform::Devices::OculusRift>	pRift;

		// Primary Camera
		std::unique_ptr<DirectX::Scene::ICameraBase>	m_pPrimaryCamera;

		// Extern Devices depend on Camera
		std::shared_ptr<Platform::Devices::LeapMotion>	pLeap;

		// Application Logic object
		std::vector<std::unique_ptr<Platform::IAppComponent>> Components;
		std::map<Platform::IAppComponent*, std::vector<Platform::Fundation::EventConnection>> ComponentsEventRegisterations;


		// Rendering loop timer.
		DirectX::StepTimer m_timer;
	};

}
