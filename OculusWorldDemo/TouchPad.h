#pragma once
#include <map>
#include <memory>
#include "Common.hpp"

namespace Platform
{
	namespace Input
	{
		using namespace Platform;

		struct PressureMapFrame{		// Frame of data, containing the forces, the frame number and time stamp
			int* forces;						// An array of forces
			int numForces;                      // Number of values in the force array
			int rows;                           // Number of rows in the force array
			int cols;                           // Number of columns in the force array
			unsigned long frameNumber;			// The frame number for this data
			double time;						// The frame time since the device was started
		};

		enum class TouchType : unsigned
		{
			TOUCH_DOWN,					// Touch started, put down
			TOUCH_MOVE,					// Touch moved, continued
			TOUCH_UP					// Touch released
		};

		struct TouchPoint{		// A TactonicTouch
			int id;								// An id for an individual touch					
			float x, y, force;					// Position and force on device
			float dx, dy, dForce;				// Changes in position and force from last framev
			TouchType touchtype;				// Type of touch
		};

		struct TouchPointsFrame{	// A frame of TactonicTouches
			TouchPoint *touches;             // The array of touches
			int numTouches;						// The number of active touches in the array
			long frameNumber;					// The frame number of this touch frame
			double time;						// The time stamp for the touch frame
		};

		class TouchPad
		{
		public:
			enum UsageType : unsigned
			{
				Usage_ForceMap = 1,
				Usage_TouchPoints = 2,
			};

			~TouchPad();

			Platform::Event<TouchPointsFrame*> &TouchFrameReady();
			Platform::Event<PressureMapFrame*> &ForceFrameReady();

			PressureMapFrame* CurrentForceFrame();
			TouchPointsFrame* CurrentTouchFrame();

			static void Initialize();

			TouchPad(unsigned deviceIndex, unsigned usageFlag);
			TouchPad(nullptr_t);
			TouchPad();

			inline bool operator==(nullptr_t)
			{
				return pImpl == nullptr;
			}

			inline bool operator!=(nullptr_t)
			{
				return pImpl != nullptr;
			}

			inline bool operator==(const TouchPad& rhs)
			{
				return pImpl == rhs.pImpl;
			}

			inline void operator=(nullptr_t)
			{
				pImpl = nullptr;
			}

			inline void operator=(TouchPad&& rhs)
			{
				pImpl = std::move(rhs.pImpl);
			}

			inline void operator=(const TouchPad& rhs)
			{
				pImpl = rhs.pImpl;
			}

			inline operator bool()
			{
				return pImpl != nullptr;
			}

			unsigned Rows() const;

			unsigned Cols() const;

		private:

			class Impl;
			friend Impl;
			std::shared_ptr<Impl> pImpl;
		};

		class TouchHandler
		{
			Event<TouchPoint*> TouchPiontLanded;
			Event<TouchPoint*> TouchPointDismissed;

			Event<TouchPointsFrame*> TwoPointInteractionLanded;
		};
	}
}