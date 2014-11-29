#pragma once

#include "Common\BasicClass.h"
#include <Leap.h>
#include <memory>
#include "Interactive.h"

namespace Platform
{
	namespace Devices
	{
		class LeapMotion : public std::enable_shared_from_this<LeapMotion>, public Leap::Listener
		{
		public:
			LeapMotion(bool useEvent = true);
			~LeapMotion();

			Leap::Controller& Controller();
			const Leap::Controller& Controller() const;

			virtual void onConnect(const Leap::Controller & controller) override;
			virtual void onDisconnect(const Leap::Controller &controller) override;
			virtual void onFrame(const Leap::Controller &controller) override;

			Platform::Fundation::Event<const Leap::Controller &> FrameArrived;
			Platform::Fundation::Event<const Leap::Controller &> DeviceConnected;
			Platform::Fundation::Event<const Leap::Controller &> DeviceDisconnected;
			Platform::Fundation::Event<const Platform::UserHandsEventArgs &> HandsTracked;
			Platform::Fundation::Event<const Platform::UserHandsEventArgs &> HandsLost;
			Platform::Fundation::Event<const Platform::UserHandsEventArgs &> HandsMove;

		private:
			int				 PrevHandsCount;
			Leap::Frame		 CurrentFrame;
			Leap::Controller LeapController;
		};
	}
}