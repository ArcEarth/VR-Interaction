#pragma once
#include <OVR_Kernel.h>
#include "Common.hpp"
#include <memory>

namespace Platform
{
	namespace Input
	{
		class IOrientedDevice
		{
		public:
			// Get current orientation relative to default
			virtual OVR::Quatf GetCurrentOrientation() const = 0;
			virtual OVR::Quatf GetOrientationDelta() const = 0;
			// Set current orientation as default
			virtual void ResetOrientation() = 0;
		};

		class OrientedTouchPad : IOrientedDevice
		{
		public:
			OrientedTouchPad();
			virtual ~OrientedTouchPad();

			void Start();
			void Stop();

			void Update();

			Platform::Event<OVR::Quatf> OrientationChanged;

			// Inherited via IOrientedDevice
			virtual OVR::Quatf GetCurrentOrientation() const override;

			virtual void ResetOrientation() override;

			virtual OVR::Quatf GetOrientationDelta() const override;

		private:
			struct OrientatedTablet;
			OVR::Quatf DefaultOrientation;
			OVR::Quatf CurrentOrientation;
			OVR::Quatf CurrentOrientationDelta;
			std::unique_ptr<OrientatedTablet> p_Impl;

			// Inherited via IOrientedDevice
		};
	}

}