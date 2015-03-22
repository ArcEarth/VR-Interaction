#pragma once

#include "BCL.h"
#include <Leap.h>
#include <memory>
#include "Interactive.h"
//#include "Common\Locatable.h"

namespace Causality
{
	namespace Devices
	{
		class LeapMotion : public std::enable_shared_from_this<LeapMotion>
		{
		public:
			static std::weak_ptr<LeapMotion> wpCurrentDevice;
			static std::shared_ptr<LeapMotion> GetForCurrentView();

			LeapMotion(bool useEvent = true, bool isFixed = false);
			~LeapMotion();

			Leap::Controller& Controller();
			const Leap::Controller& Controller() const;

			// To use with seqential logic and distribute events
			void PullFrame();

			//void SetMotionProvider(DirectX::ILocatable* pHeadLoc, DirectX::IOriented *pHeadOrient);
			// Assign Leap motion's coordinate in world space
			void SetDeviceWorldCoord(const DirectX::Matrix4x4 &m);
			// The transform matrix convert Leap Coordinate to World coordinate
			DirectX::XMMATRIX ToWorldTransform() const;

			Event<const Leap::Controller &>		FrameArrived;
			Event<const Leap::Controller &>		DeviceConnected;
			Event<const Leap::Controller &>		DeviceDisconnected;
			Event<const UserHandsEventArgs &>	HandsTracked;
			Event<const UserHandsEventArgs &>	HandsLost;
			Event<const UserHandsEventArgs &>	HandsMove;

		private:
			class Listener;
			std::unique_ptr<Listener>	pListener;
			DirectX::Matrix4x4			Coordinate;
			//DirectX::ILocatable			*pHeadLocation;
			//DirectX::IOriented			*pHeadOrientation;
			bool						PrevConnectionStates;
			Leap::Controller			LeapController;
		};
	}
}