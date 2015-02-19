#ifndef	KINECT_2_USER
#define KINECT_2_USER

#ifdef _MSC_VER
#pragma once
#endif  // _MSC_VER
#include "Common\BasicClass.h"
#include <wrl\client.h>
#include <memory>
#include "Common\DirectXMathExtend.h"
#include "Skeleton.h"



namespace Platform
{
	namespace Devices
	{
#ifndef _HandEnum_
#define _HandEnum_
		enum _HandEnum
		{
			HandEnum_Left = 0,
			HandEnum_Right = 1,
			HandEnum_Count = HandEnum_Right + 1,
		};
		typedef _HandEnum HandEnum;

#endif // _HandEnum_
#ifndef _HandState_
#define _HandState_
		typedef enum _HandState HandState;


		enum _HandState
		{
			HandState_Unknown = 0,
			HandState_NotTracked = 1,
			HandState_Open = 2,
			HandState_Closed = 3,
			HandState_Lasso = 4
		};
#endif // _HandState_

#ifndef _JointType_
#define _JointType_
		typedef enum _JointType JointType;


		enum _JointType
		{
			JointType_SpineBase = 0,
			JointType_SpineMid = 1,
			JointType_Neck = 2,
			JointType_Head = 3,
			JointType_ShoulderLeft = 4,
			JointType_ElbowLeft = 5,
			JointType_WristLeft = 6,
			JointType_HandLeft = 7,
			JointType_ShoulderRight = 8,
			JointType_ElbowRight = 9,
			JointType_WristRight = 10,
			JointType_HandRight = 11,
			JointType_HipLeft = 12,
			JointType_KneeLeft = 13,
			JointType_AnkleLeft = 14,
			JointType_FootLeft = 15,
			JointType_HipRight = 16,
			JointType_KneeRight = 17,
			JointType_AnkleRight = 18,
			JointType_FootRight = 19,
			JointType_SpineShoulder = 20,
			JointType_HandTipLeft = 21,
			JointType_ThumbLeft = 22,
			JointType_HandTipRight = 23,
			JointType_ThumbRight = 24,
			JointType_Count = (JointType_ThumbRight + 1)
		};
#endif // _JointType_

		struct TrackedPlayer
		{
		public:
			// Skeleton Structure and basic body parameter
			Kinematics::SkeletonBase* Skeleton;

			Kinematics::BoneAnimationFrame& RestFrame;
			// Current pose data
			Kinematics::BoneAnimationFrame CurrentFrame;

			// Hand States
			HandState HandStates[HandEnum_Count];

			LONGLONG PlayerID;

			bool IsCurrentTracked;
		};

		// An aggregate of Kinect Resources
		class Kinect
		{
		public:
			HRESULT Initalize();

			bool IsConnected() const;

			// Player Event event interface!
			Platform::Fundation::Event<const TrackedPlayer&> OnPlayerTracked;
			Platform::Fundation::Event<const TrackedPlayer&> OnPlayerLost;
			Platform::Fundation::Event<const TrackedPlayer&> OnPlayerPoseChanged;
			Platform::Fundation::Event<const TrackedPlayer&, HandEnum, HandState> OnPlayerHandStateChanged;

			// Pull Interface!
			bool HasNewFrame() const;
			const std::list<TrackedPlayer*> &GetLatestFrame();

			// Static Constructors!!!
			static std::unique_ptr<Kinect> CreateDefault()
			{
				std::unique_ptr<Kinect> pKinect(new Kinect);
				if (SUCCEEDED(pKinect->Initalize()))
					return pKinect;
				else
					return nullptr;
			}
		protected:

			Kinect();

			static JointType JointsParent[JointType_Count];

		private:

			class Impl;
			std::unique_ptr<Impl> pImpl;

		};
		
	}
}


#endif


