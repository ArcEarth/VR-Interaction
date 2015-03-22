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
	class KeyboardMouseLogic : public IAppComponent , public DirectX::Scene::ITimeAnimatable, public IKeybordInteractive, public ICursorInteractive
	{
	public:
		KeyboardMouseLogic(Camera* pCamera);
		// Inherited via ITimeAnimatable
		virtual void UpdateAnimation(DirectX::StepTimer const & timer) override;

		// Inherited via IKeybordInteractive
		virtual void OnKeyDown(const KeyboardEventArgs & e) override;
		virtual void OnKeyUp(const KeyboardEventArgs & e) override;

		// Inherited via ICursorInteractive
		virtual void OnMouseButtonDown(const CursorButtonEvent & e) override;
		virtual void OnMouseButtonUp(const CursorButtonEvent & e) override;
		virtual void OnMouseMove(const CursorMoveEventArgs & e) override;
	public:
		float											Speed;
		DirectX::Quaternion								InitialOrientation;
		float											CameraYaw = 0;
		float											CameraPitch = 0;
	private:
		Camera*											m_pCamera= nullptr;
		bool											IsTrackingCursor = false;
		DirectX::Vector3								CameraVeclocity;
		DirectX::Vector3								CameraAngularVeclocity;
	};

	class App : public Application, public DirectX::IDeviceNotify
	{
	public:
		static App* Current() {
			return static_cast<App*>(Application::Current.get());
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

		void RegisterComponent(std::unique_ptr<IAppComponent> &&pComponent);
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
		std::shared_ptr<Devices::Kinect>				pKinect;
		std::shared_ptr<Devices::LeapMotion>			pLeap;

		std::vector<std::unique_ptr<Scene>>				Scenes;

		// Application Logic object
		std::vector<std::unique_ptr<IAppComponent>>		Components;
		std::map<IAppComponent*, std::vector<EventConnection>> ComponentsEventRegisterations;


		// Rendering loop timer.
		DirectX::StepTimer m_timer;
	};

}
