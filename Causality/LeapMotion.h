#pragma once

#include "Common\BasicClass.h"
#include <Leap.h>
#include <memory>
#include "Interactive.h"
#include "Common\Locatable.h"

namespace Platform
{
	namespace Devices
	{
		class LeapMotion : public std::enable_shared_from_this<LeapMotion>
		{
		public:

			LeapMotion(bool useEvent = true, bool isFixed = false);
			~LeapMotion();

			Leap::Controller& Controller();
			const Leap::Controller& Controller() const;

			// To use with seqential logic and distribute events
			void PullFrame();

			void SetMotionProvider(DirectX::ILocatable* pHeadLoc, DirectX::IOriented *pHeadOrient);
			// Assign Leap motion's coordinate in world space
			void SetLeapCoord(const DirectX::Matrix4x4 &m);
			// The transform matrix convert Leap Coordinate to World coordinate
			DirectX::XMMATRIX ToWorldTransform() const;

			Platform::Fundation::Event<const Leap::Controller &> FrameArrived;
			Platform::Fundation::Event<const Leap::Controller &> DeviceConnected;
			Platform::Fundation::Event<const Leap::Controller &> DeviceDisconnected;
			Platform::Fundation::Event<const Platform::UserHandsEventArgs &> HandsTracked;
			Platform::Fundation::Event<const Platform::UserHandsEventArgs &> HandsLost;
			Platform::Fundation::Event<const Platform::UserHandsEventArgs &> HandsMove;

		private:
			class Listener;
			std::unique_ptr<Listener>	pListener;
			DirectX::Matrix4x4			Coordinate;
			DirectX::ILocatable			*pHeadLocation;
			DirectX::IOriented			*pHeadOrientation;
			bool						PrevConnectionStates;
			Leap::Controller			LeapController;
		};
	}
}