//*********************************************************
//
// Copyright (c) Microsoft. All rights reserved.
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
//*********************************************************

//
// WASAPICapture.h
//
#pragma once
#include <string>
#include <memory>
#include <vector>
#include <thread>
#include <atomic>

#include <Windows.h>
#include <mfapi.h>
#include <AudioClient.h>
#include <mmdeviceapi.h>
#include <wrl\client.h>

#include "DeviceState.h"
#include "AudioCommon.h"

namespace Platform
{

	namespace Audio
	{
		class IAudioSink
			: public Microsoft::WRL::RuntimeClass < Microsoft::WRL::RuntimeClassFlags< Microsoft::WRL::ClassicCom >, IUnknown>
		{
		public:
			typedef const WAVEFORMATEX *LPCWAVEFORMATEX;

			virtual HRESULT OnCaptureStarted();
			virtual HRESULT OnCaptureStopped();
			// OnFormatChange
			virtual HRESULT OnFormatChange(LPCWAVEFORMATEX pwfx) = 0;
			// OnSample
			virtual HRESULT OnSamples(BYTE* pData, UINT32 numFramesAvailable) = 0;
		};


		// Primary WASAPI Capture Class
		class AudioCaptureDevice :
			public Microsoft::WRL::RuntimeClass < Microsoft::WRL::RuntimeClassFlags< Microsoft::WRL::ClassicCom >, Microsoft::WRL::FtmBase, IActivateAudioInterfaceCompletionHandler >
			, public AudioDeviceStateChangedEvent
		{
		public:
			AudioCaptureDevice();
			~AudioCaptureDevice();

			HRESULT InitializeAudioDevice(UINT reflexIntervalMilliSec);
			HRESULT StartCaptureAsync();
			HRESULT StopCaptureAsync();
			HRESULT StopCapture();
			//HRESULT FinishCaptureAsync();

			inline const IAudioSink* AudioSink() const
			{
				return m_pAudioSink.Get();
			}

			HRESULT SetAudioSink(IAudioSink* pAudioSink);

			// IActivateAudioInterfaceCompletionHandler
			STDMETHOD(ActivateCompleted)(IActivateAudioInterfaceAsyncOperation *operation);

		private:

			HRESULT OnStartCapture(IMFAsyncResult* pResult);
			HRESULT OnStopCapture(IMFAsyncResult* pResult);
			HRESULT OnSampleReady(IMFAsyncResult* pResult);

			HRESULT OnAudioSampleRequested(bool IsSilence = false);

		public:
			MFAsyncCallback<AudioCaptureDevice, &OnStartCapture>  cb_StartCapture;
			MFAsyncCallback<AudioCaptureDevice, &OnStopCapture>   cb_StopCapture;
			MFAsyncCallback<AudioCaptureDevice, &OnSampleReady>   cb_SampleReady;

		private:
			//std::string			m_DeviceIdString;
			UINT32              m_BufferFrames;
			HANDLE              m_SampleReadyEvent;
			MFWORKITEM_KEY      m_SampleReadyKey;
			CRITICAL_SECTION    m_CritSec;
			DWORD               m_dwQueueID;


			WAVEFORMATEX								*m_MixFormat;
			Microsoft::WRL::ComPtr<IAudioClient>		m_AudioClient;
			Microsoft::WRL::ComPtr<IAudioCaptureClient>	m_AudioCaptureClient;
			Microsoft::WRL::ComPtr<IMFAsyncResult>		m_SampleReadyAsyncResult;
			Microsoft::WRL::ComPtr<IAudioSink>			m_pAudioSink;

			//std::unique_ptr<AudioDeviceStateChangedEvent>	m_pDeviceStateChanged;


		};

#pragma region SynchronizeCaptureDevice


#pragma endregion

	}
}