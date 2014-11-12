#pragma once
#include "Common\BasicClass.h"
#include <memory>
#include "Common\Textures.h"
#include "Common\DeviceResources.h"
#include "Common\Carmera.h"

namespace Platform
{
	namespace Devices
	{

		class OculusRift //: public IHeadMountedDisplay
		{
		public:

			OculusRift();
			~OculusRift();

			void Initialize(HWND hWnd, DirectX::DeviceResources* pDeviceResource);

			// Rendering Methods and Properties
			void DissmisHealthWarnning();
			void BeginFrame();
			void EndFrame();
			void SetEyeRenderTarget(DirectX::Scene::EyesEnum eye);

			DirectX::RenderTargetTexture2D& EyeTexture(DirectX::Scene::EyesEnum eye);
			DirectX::DepthStencilBuffer& DepthStencilBuffer();

			DirectX::XMMATRIX EyeProjection(DirectX::Scene::EyesEnum eye) const;

			// Tracking States
			const Platform::Fundation::StaticPose& EyePoses(DirectX::Scene::EyesEnum eye) const;
			float UserEyeHeight() const;
			const  Platform::Fundation::DynamicPose& HeadPose() const;

		private:
			class Impl;
			std::unique_ptr<Impl> pImpl;
		};

		enum GenderEnum
		{
			Unspecifed,
			Male,
			Female,
		};

		struct UserProfile
		{
			int UserIdx;
			std::string Name;
			GenderEnum Gender;
			float PlayerHeight;
			float EyeHeight;
			float IPD;
			float NeckEyeDistance;
			float CenteredFromWorld;
		};

		//class IPositionSensor abstract
		//{
		//public:
		//	virtual Math::Vector3 CurrentPosition() = 0;
		//	virtual Math::Vector3 CurrentVelocity() = 0;
		//	event Windows::Foundation::EventHandler<Platform::Object^>^ PositionChanged;
		//};
		//
		//interface class IOrientationSensor 
		//{
		//public:
		//	property Math::Quaternion CurrentOrientation { Math::Quaternion get(); };
		//	event Windows::Foundation::EventHandler<Platform::Object^>^ OrientationChanged;
		//};
		//
		//interface class IHeadMountedDisplay  : public IPositionSensor, public IOrientationSensor
		//{
		//public:
		//	property float LogicalDpi;
		//	property Windows::Foundation::Size DisplaySize { Windows::Foundation::Size get(); }
		//	property Windows::Foundation::Size EyeViewSize { Windows::Foundation::Size get(); }
		//	property bool NeedDisortion { bool get(); };
		//	virtual Windows::Foundation::Rect EyeViewport(int eye) = 0;
		//	virtual Math::Matrix4 EyeView(int eye) const = 0;
		//	virtual EyePose EyePose(int eye) const = 0;
		//	virtual Math::Matrix4 EyeProjection(int eye) = 0;
		//};

	}
}