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
#include "Common.h"

//namespace Platform
//{
//	//using String = OVR::String;
//
//	using Boolean = bool;
//}

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

		//AudioDeviceStateChangedEvent* GetDeviceStateEvent() { return m_pDeviceStateChanged.get(); };


		// IActivateAudioInterfaceCompletionHandler
		STDMETHOD(ActivateCompleted)(IActivateAudioInterfaceAsyncOperation *operation);

	private:

		HRESULT OnStartCapture        (IMFAsyncResult* pResult);
		HRESULT OnStopCapture         (IMFAsyncResult* pResult);
		HRESULT OnSampleReady         (IMFAsyncResult* pResult);

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


	//template<class T> using ComPtr<T> = Microsoft::WRL::ComPtr < T > ;
	//template<class T> using unique_ptr<T> = std::unique_ptr < T > ;
	//template<class T> using shared_ptr<T> = std::shared_ptr < T > ;



	// Primary WASAPI Capture Class
	//-----------------------------------------------------------
	// Record an audio stream from the default audio capture
	// device. The RecordAudioStream function allocates a shared
	// buffer big enough to hold one second of PCM audio data.
	// The function uses this buffer to stream data from the
	// capture device. The main loop runs every 1/2 second.
	//-----------------------------------------------------------

	// REFERENCE_TIME time units per second and per millisecond

#pragma region SynchronizeCaptureDevice
	//class AudioRecorder
	//{
	//public:
	//	enum States
	//	{
	//		Uninit,
	//		Ready,
	//		Recording,
	//		Stopped = Ready,
	//		Error,
	//	};

	//	const REFERENCE_TIME REFTIMES_PER_SEC = 10000000;
	//	const REFERENCE_TIME REFTIMES_PER_MILLISEC = 10000;

	//	AudioRecorder();
	//	~AudioRecorder();
	//	HRESULT Initialize(const std::shared_ptr<IAudioSink> &pSink, double requestSampleIntervalSec);
	//	HRESULT Release();

	//	HRESULT StartCaptureAsync();
	//	HRESULT StopCapture();

	//	States State() const;
	//	bool IsRecording() const { return state == Recording; }

	//	static void ConvertMixFormat(WAVEFORMATEX *pWfx);

	//protected:
	//	HRESULT StartCapturing();

	//private:
	//	std::shared_ptr<IAudioSink> m_pSink;
	//	std::unique_ptr<std::thread> m_pCaptureThread;

	//	HRESULT hr;
	//	WAVEFORMATEX *pwfx = NULL;
	//	UINT32 packetLength = 0;
	//	States state;
	//	std::atomic_bool bDone;
	//	BYTE *pData;
	//	DWORD flags;
	//	REFERENCE_TIME hnsRequestedDuration;
	//	REFERENCE_TIME hnsActualDuration;
	//	UINT32 bufferFrameCount;
	//	UINT32 numFramesAvailable;

	//	Microsoft::WRL::ComPtr<IAudioClient> m_pAudioClient = NULL;
	//	Microsoft::WRL::ComPtr<IAudioCaptureClient> m_pCaptureClient = NULL;

	//};

#pragma endregion

}
