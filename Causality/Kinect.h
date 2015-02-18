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
#ifndef _Expression_
#define _Expression_
		typedef enum _Expression Expression;


		enum _Expression
		{
			Expression_Neutral = 0,
			Expression_Happy = 1,
			Expression_Count = (Expression_Happy + 1)
		};
#endif // _Expression_
#ifndef _DetectionResult_
#define _DetectionResult_
		typedef enum _DetectionResult DetectionResult;


		enum _DetectionResult
		{
			DetectionResult_Unknown = 0,
			DetectionResult_No = 1,
			DetectionResult_Maybe = 2,
			DetectionResult_Yes = 3
		};
#endif // _DetectionResult_
#ifndef _TrackingConfidence_
#define _TrackingConfidence_
		typedef enum _TrackingConfidence TrackingConfidence;


		enum _TrackingConfidence
		{
			TrackingConfidence_Low = 0,
			TrackingConfidence_High = 1
		};
#endif // _TrackingConfidence_
#ifndef _Activity_
#define _Activity_
		typedef enum _Activity Activity;


		enum _Activity
		{
			Activity_EyeLeftClosed = 0,
			Activity_EyeRightClosed = 1,
			Activity_MouthOpen = 2,
			Activity_MouthMoved = 3,
			Activity_LookingAway = 4,
			Activity_Count = (Activity_LookingAway + 1)
		};
#endif // _Activity_
#ifndef _Appearance_
#define _Appearance_
		typedef enum _Appearance Appearance;


		enum _Appearance
		{
			Appearance_WearingGlasses = 0,
			Appearance_Count = (Appearance_WearingGlasses + 1)
		};
#endif // _Appearance_
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

		class TrackedPlayer
		{

		};
		// An aggregate of Kinect Resources
		class Kinect
		{
		public:
			static std::unique_ptr<Kinect> CreateDefault()
			{
				std::unique_ptr<Kinect> pKinect(new Kinect);
				if (SUCCEEDED(pKinect->Initalize()))
					return pKinect;					
				else
					return nullptr;
			}

			bool IsActive() const
			{
				return m_pKinectSensor;
			}


			Platform::Fundation::Event<const TrackedPlayer&> OnPlayerTracked;
			Platform::Fundation::Event<const TrackedPlayer&> OnPlayerLost;
			Platform::Fundation::Event<const TrackedPlayer&> OnPlayerAction;
			Platform::Fundation::Event<const TrackedPlayer&, Expression> OnPlayerExpressionChanged;
			Platform::Fundation::Event<const TrackedPlayer&, HandEnum, HandState> OnPlayerHandStateChanged;

			IBodyFrameReader* BodyFrameReader() const
			{
				return m_pBodyFrameReader.Get();
			}

			ICoordinateMapper* CoordinateMapper() const
			{
				return m_pCoordinateMapper.Get();
			}

			IKinectGestureRecognizer* GestureRecongnizer() const
			{
				return nullptr;
			}

		protected:

			//static void FillFrameWithIBody(Kinematics::BoneAnimationFrame& frame, IBody* pBody)
			//{
			//	Joint joints[JointType_Count];
			//	JointOrientation oris[JointType_Count];
			//	pBody->GetJoints(ARRAYSIZE(joints), joints);
			//	pBody->GetJointOrientations(ARRAYSIZE(oris), oris);
			//	oris[0].Orientation
			//};

			Kinect()
			{}

			HRESULT Initalize()
			{
				using namespace Microsoft::WRL;
				using namespace std;
				HRESULT hr;
				// Current Kinect
				ComPtr<IKinectSensor>          pKinectSensor;
				ComPtr<ICoordinateMapper>      pCoordinateMapper;
				ComPtr<IBodyFrameReader>       pBodyFrameReader;
				ComPtr<IBodyFrameSource>	   pBodyFrameSource = NULL;

				hr = GetDefaultKinectSensor(&pKinectSensor);
				if (FAILED(hr))
				{
					return hr;
				}

				if (m_pKinectSensor)
				{
					// Initialize the Kinect and get coordinate mapper and the body reader

					hr = m_pKinectSensor->Open();

					if (SUCCEEDED(hr))
					{
						hr = m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
					}

					if (SUCCEEDED(hr))
					{
						hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
					}

					if (SUCCEEDED(hr))
					{
						hr = pBodyFrameSource->OpenReader(&pBodyFrameReader);
					}
				}

				if (!m_pKinectSensor || FAILED(hr))
				{
					//SetStatusMessage(L"No ready Kinect found!", 10000, true);
					return E_FAIL;
				}

				m_pKinectSensor = move(pKinectSensor);
				m_pCoordinateMapper = move(pCoordinateMapper);
				m_pBodyFrameReader = move(pBodyFrameReader);
				return hr;
			}

			static JointType JointsParent[JointType_Count];

		private:

			class Impl;
			std::unique_ptr<Impl> pImpl;

			// Current Kinect
			Microsoft::WRL::ComPtr<IKinectSensor>          m_pKinectSensor;
			Microsoft::WRL::ComPtr<ICoordinateMapper>      m_pCoordinateMapper;
			// Body reader
			Microsoft::WRL::ComPtr<IBodyFrameReader>       m_pBodyFrameReader;
		};
		
	}
}


#endif


