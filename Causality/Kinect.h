#ifndef	KINECT_2_USER
#define KINECT_2_USER

#ifdef _MSC_VER
#pragma once
#endif  // _MSC_VER
#include <Kinect.h>
#include <wrl\client.h>
#include <memory>
#include "Common\DirectXMathExtend.h"

namespace Platform
{
	namespace Devices
	{
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


			// Current Kinect
			Microsoft::WRL::ComPtr<IKinectSensor>          m_pKinectSensor;
			Microsoft::WRL::ComPtr<ICoordinateMapper>      m_pCoordinateMapper;
			// Body reader
			Microsoft::WRL::ComPtr<IBodyFrameReader>       m_pBodyFrameReader;
		};
		
	}
}


#endif


