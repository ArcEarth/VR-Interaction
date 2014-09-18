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

//#include "pch.h"
#include "Common.h"
#include "AudioCaptureDevice.h"
//#include <windows.media.devices.h>
#include <ppl.h>
#include <ppltasks.h>

//#define ACTIVE_AUDIO_ASYNC

const CLSID CLSID_MMDeviceEnumerator = __uuidof(MMDeviceEnumerator);
const IID IID_IMMDeviceEnumerator = __uuidof(IMMDeviceEnumerator);
const IID IID_IAudioClient = __uuidof(IAudioClient);
const IID IID_IAudioCaptureClient = __uuidof(IAudioCaptureClient);

HRESULT Audio::IAudioSink::OnCaptureStarted()
{
	return S_OK;
}

HRESULT Audio::IAudioSink::OnCaptureStopped()
{
	return S_OK;
}

using namespace Microsoft::WRL;
using namespace Audio;
using namespace std;

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

//#include "pch.h"
#include "AudioCaptureDevice.h"

//
//  WASAPICapture()
//
AudioCaptureDevice::AudioCaptureDevice() :
m_BufferFrames(0),
m_dwQueueID(0),
m_SampleReadyAsyncResult(nullptr),
m_pAudioSink(nullptr),
m_AudioClient(nullptr),
m_AudioCaptureClient(nullptr)
{
	// Create events for sample ready or user stop
	m_SampleReadyEvent = CreateEventEx(nullptr, nullptr, 0, EVENT_ALL_ACCESS);
	if (nullptr == m_SampleReadyEvent)
	{
		ThrowIfFailed(HRESULT_FROM_WIN32(GetLastError()));
	}

	if (!InitializeCriticalSectionEx(&m_CritSec, 0, 0))
	{
		ThrowIfFailed(HRESULT_FROM_WIN32(GetLastError()));
	}

	// Register MMCSS work queue
	HRESULT hr = S_OK;
	DWORD dwTaskID = 0;

	hr = MFStartup(MF_VERSION);
	if (FAILED(hr))
	{
		ThrowIfFailed(hr);
	}

	hr = MFLockSharedWorkQueue(L"Capture", 0, &dwTaskID, &m_dwQueueID);
	if (FAILED(hr))
	{
		ThrowIfFailed(hr);
	}

	// Set the capture event work queue to use the MMCSS queue
	m_xSampleReady.SetQueueID(m_dwQueueID);
}

//
//  ~WASAPICapture()
//
AudioCaptureDevice::~AudioCaptureDevice()
{
	if (INVALID_HANDLE_VALUE != m_SampleReadyEvent)
	{
		CloseHandle(m_SampleReadyEvent);
		m_SampleReadyEvent = INVALID_HANDLE_VALUE;
	}

	MFUnlockWorkQueue(m_dwQueueID);

	DeleteCriticalSection(&m_CritSec);

	MFShutdown();
}

//
//  InitializeAudioDeviceAsync()
//
//  Activates the default audio capture on a asynchronous callback thread.  This needs
//  to be called from the main UI thread.
//
HRESULT AudioCaptureDevice::InitializeAudioDeviceAsync(IAudioSink* pAudioSink)
{
	HRESULT hr = S_OK;
	ComPtr<IActivateAudioInterfaceAsyncOperation> pAsyncOp = nullptr;
	ComPtr<IMMDeviceEnumerator> pEnumerator = nullptr;
	ComPtr<IMMDevice> pDevice = nullptr;

	if (pAudioSink == nullptr)
		return E_INVALIDARG;

	m_pAudioSink = pAudioSink;

	// Get a string representing the Default Audio Capture Device


	hr = CoCreateInstance(
		CLSID_MMDeviceEnumerator, NULL,
		CLSCTX_ALL, IID_IMMDeviceEnumerator,
		(void**) &pEnumerator);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}

	hr = pEnumerator->GetDefaultAudioEndpoint(
		eCapture, eConsole, &pDevice);
	
	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}


#ifdef ACTIVE_AUDIO_ASYNC


	LPWSTR endPointId = L"\\?\SWD#MMDEVAPI#{0.0.1.00000000}.{963a2c89-65fe-46f1-850d-4eea82af65b2}#{2eef81be-33fa-4800-9670-1cd474972c3f}";

	hr = pDevice->GetId(&endPointId);

	// This call must be made on the main UI thread.  Async operation will call back to 
	// IActivateAudioInterfaceCompletionHandler::ActivateCompleted, which must be an agile interface implementation
	hr = ActivateAudioInterfaceAsync(endPointId, __uuidof(IAudioClient), nullptr, this, &pAsyncOp);
	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
	}
#else // ACTIVE_AUDIO_ASYNC
	hr = pDevice->Activate(IID_IAudioClient, CLSCTX_ALL, NULL, (void**) &m_AudioClient);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}

	hr = m_AudioClient->GetMixFormat(&m_MixFormat);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}
	// convert from Float to PCM and from WAVEFORMATEXTENSIBLE to WAVEFORMATEX
	if ((m_MixFormat->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) ||
		((m_MixFormat->wFormatTag == WAVE_FORMAT_EXTENSIBLE) &&
		(reinterpret_cast<WAVEFORMATEXTENSIBLE *>(m_MixFormat)->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT)))
	{
		m_MixFormat->wFormatTag = WAVE_FORMAT_PCM;
		m_MixFormat->wBitsPerSample = 16;
		m_MixFormat->nBlockAlign = m_MixFormat->nChannels * 2;    // (nChannels * wBitsPerSample) / 8
		m_MixFormat->nAvgBytesPerSec = m_MixFormat->nSamplesPerSec * m_MixFormat->nBlockAlign;
		m_MixFormat->cbSize = 0;
	}

	// Initialize the AudioClient in Shared Mode with the user specified buffer
	hr = m_AudioClient->Initialize(AUDCLNT_SHAREMODE_SHARED,
		AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
		200000,
		0,
		m_MixFormat,
		nullptr);

	hr = m_pAudioSink->OnFormatChange(m_MixFormat);

	// Get the maximum size of the AudioClient Buffer
	hr = m_AudioClient->GetBufferSize(&m_BufferFrames);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}

	// Get the capture client
	hr = m_AudioClient->GetService(__uuidof(IAudioCaptureClient), (void**) &m_AudioCaptureClient);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}

	// Create Async callback for sample events
	hr = MFCreateAsyncResult(nullptr, &m_xSampleReady, nullptr, &m_SampleReadyAsyncResult);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}

	// Sets the event handle that the system signals when an audio buffer is ready to be processed by the client
	hr = m_AudioClient->SetEventHandle(m_SampleReadyEvent);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		return hr;
	}

#endif // ACTIVE_AUDIO_ASYNC

	SetState(DeviceState::DeviceStateInitialized,hr,true);
	return hr;
}

//
//  ActivateCompleted()
//
//  Callback implementation of ActivateAudioInterfaceAsync function.  This will be called on MTA thread
//  when results of the activation are available.
//
HRESULT AudioCaptureDevice::ActivateCompleted(IActivateAudioInterfaceAsyncOperation *operation)
{
	HRESULT hr = S_OK;
	HRESULT hrActivateResult = S_OK;
	IUnknown *punkAudioInterface = nullptr;

	// Check for a successful activation result
	hr = operation->GetActivateResult(&hrActivateResult, &punkAudioInterface);
	if (SUCCEEDED(hr) && SUCCEEDED(hrActivateResult))
	{
		// Get the pointer for the Audio Client
		punkAudioInterface->QueryInterface(IID_PPV_ARGS(&m_AudioClient));
		if (nullptr == m_AudioClient)
		{
			hr = E_FAIL;
			goto exit;
		}

		hr = m_AudioClient->GetMixFormat(&m_MixFormat);
		if (FAILED(hr))
		{
			goto exit;
		}

		// convert from Float to PCM and from WAVEFORMATEXTENSIBLE to WAVEFORMATEX
		if ((m_MixFormat->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) ||
			((m_MixFormat->wFormatTag == WAVE_FORMAT_EXTENSIBLE) &&
			(reinterpret_cast<WAVEFORMATEXTENSIBLE *>(m_MixFormat)->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT)))
		{
			m_MixFormat->wFormatTag = WAVE_FORMAT_PCM;
			m_MixFormat->wBitsPerSample = 16;
			m_MixFormat->nBlockAlign = m_MixFormat->nChannels * 2;    // (nChannels * wBitsPerSample) / 8
			m_MixFormat->nAvgBytesPerSec = m_MixFormat->nSamplesPerSec * m_MixFormat->nBlockAlign;
			m_MixFormat->cbSize = 0;
		}

		// Initialize the AudioClient in Shared Mode with the user specified buffer
		hr = m_AudioClient->Initialize(AUDCLNT_SHAREMODE_SHARED,
			AUDCLNT_STREAMFLAGS_EVENTCALLBACK,
			5000,
			0,
			m_MixFormat,
			nullptr);

		m_pAudioSink->OnFormatChange(m_MixFormat);

		if (FAILED(hr))
		{
			goto exit;
		}

		// Get the maximum size of the AudioClient Buffer
		hr = m_AudioClient->GetBufferSize(&m_BufferFrames);
		if (FAILED(hr))
		{
			goto exit;
		}

		// Get the capture client
		hr = m_AudioClient->GetService(__uuidof(IAudioCaptureClient), (void**) &m_AudioCaptureClient);
		if (FAILED(hr))
		{
			goto exit;
		}

		// Create Async callback for sample events
		hr = MFCreateAsyncResult(nullptr, &m_xSampleReady, nullptr, &m_SampleReadyAsyncResult);
		if (FAILED(hr))
		{
			goto exit;
		}

		// Sets the event handle that the system signals when an audio buffer is ready to be processed by the client
		hr = m_AudioClient->SetEventHandle(m_SampleReadyEvent);
		if (FAILED(hr))
		{
			goto exit;
		}

		SetState(DeviceState::DeviceStateInitialized, hr, true);

		// Creates the WAV file.  If successful, will set the Initialized event
		//hr = CreateWAVFile();
		//if (FAILED( hr ))
		//{
		//	goto exit;
		//}
	}

exit:
	SAFE_RELEASE(punkAudioInterface);

	if (FAILED(hr))
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
		m_AudioClient.Reset();
		m_AudioCaptureClient.Reset();
		m_SampleReadyAsyncResult.Reset();
	}

	// Need to return S_OK
	return S_OK;
}

//
//  StartCaptureAsync()
//
//  Starts asynchronous capture on a separate thread via MF Work Item
//
HRESULT AudioCaptureDevice::StartCaptureAsync()
{
	// We should be in the initialized state if this is the first time through getting ready to capture.
	if (GetState() == DeviceState::DeviceStateInitialized)
	{
		SetState(DeviceState::DeviceStateStarting, S_OK, true);
		return MFPutWorkItem2(MFASYNC_CALLBACK_QUEUE_MULTITHREADED, 0, &m_xStartCapture, nullptr);
	}

	// We are in the wrong state
	return E_NOT_VALID_STATE;
}

//
//  OnStartCapture()
//
//  Callback method to start capture
//
HRESULT AudioCaptureDevice::OnStartCapture(IMFAsyncResult* pResult)
{
	HRESULT hr = S_OK;

	// Start the capture
	hr = m_AudioClient->Start();
	if (SUCCEEDED(hr))
	{
		SetState(DeviceState::DeviceStateCapturing, S_OK, true);
		hr = MFPutWaitingWorkItem(m_SampleReadyEvent, 0, m_SampleReadyAsyncResult.Get(), &m_SampleReadyKey);
		if (SUCCEEDED(hr))
			m_pAudioSink->OnCaptureStarted();
		else
			SetState(DeviceState::DeviceStateCapturing, S_OK, true);
	}
	else
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
	}

	return hr;
}

//
//  StopCaptureAsync()
//
//  Stop capture asynchronously via MF Work Item
//
HRESULT AudioCaptureDevice::StopCaptureAsync()
{
	if ((GetState() != DeviceState::DeviceStateCapturing) &&
		(GetState() != DeviceState::DeviceStateInError))
	{
		return E_NOT_VALID_STATE;
	}

	SetState(DeviceState::DeviceStateStopping, S_OK, true);

	return MFPutWorkItem2(MFASYNC_CALLBACK_QUEUE_MULTITHREADED, 0, &m_xStopCapture, nullptr);
}

//
//  OnStopCapture()
//
//  Callback method to stop capture
//
HRESULT AudioCaptureDevice::OnStopCapture(IMFAsyncResult* pResult)
{
	// Stop capture by canceling Work Item
	// Cancel the queued work item (if any)
	if (0 != m_SampleReadyKey)
	{
		MFCancelWorkItem(m_SampleReadyKey);
		m_SampleReadyKey = 0;
	}

	m_AudioClient->Stop();

	m_SampleReadyAsyncResult.Reset();

	m_pAudioSink->OnCaptureStopped();
	// If this is set, it means we writing from the memory buffer to the actual file asynchronously
	// Since a second call to StoreAsync() can cause an exception, don't queue this now, but rather
	// let the async operation completion handle the call.
	//if (!m_fWriting)
	//{
	//	m_DeviceStateChanged->SetState( DeviceState::DeviceStateFlushing, S_OK, true );

	//	concurrency::task<unsigned int>( m_WAVDataWriter->StoreAsync()).then(
	//		[this]( unsigned int BytesWritten )
	//	{
	//		FinishCaptureAsync();
	//	});
	//}

	return S_OK;
}

//
//  FinishCaptureAsync()
//
//  Finalizes WAV file on a separate thread via MF Work Item
//
HRESULT AudioCaptureDevice::FinishCaptureAsync()
{
	// We should be flushing when this is called
	if (GetState() == DeviceState::DeviceStateFlushing)
	{
		return MFPutWorkItem2(MFASYNC_CALLBACK_QUEUE_MULTITHREADED, 0, &m_xFinishCapture, nullptr);
	}

	// We are in the wrong state
	return E_NOT_VALID_STATE;
}

//
//  OnFinishCapture()
//
//  Because of the asynchronous nature of the MF Work Queues and the DataWriter, there could still be
//  a sample processing.  So this will get called to finalize the WAV header.
//
HRESULT AudioCaptureDevice::OnFinishCapture(IMFAsyncResult* pResult)
{
	// FixWAVHeader will set the DeviceStateStopped when all async tasks are complete
	//return FixWAVHeader();
	return S_OK;
}

//
//  OnSampleReady()
//
//  Callback method when ready to fill sample buffer
//
HRESULT AudioCaptureDevice::OnSampleReady(IMFAsyncResult* pResult)
{
	HRESULT hr = S_OK;

	hr = OnAudioSampleRequested(false);

	if (SUCCEEDED(hr))
	{
		// Re-queue work item for next sample
		if (GetState() == DeviceState::DeviceStateCapturing)
		{
			hr = MFPutWaitingWorkItem(m_SampleReadyEvent, 0, m_SampleReadyAsyncResult.Get(), &m_SampleReadyKey);
		}
	}
	else
	{
		SetState(DeviceState::DeviceStateInError, hr, true);
	}

	return hr;
}

#define FLUSH_INTERVAL_SEC 3
//
//  OnAudioSampleRequested()
//
//  Called when audio device fires m_SampleReadyEvent
//
HRESULT AudioCaptureDevice::OnAudioSampleRequested(bool IsSilence)
{
	HRESULT hr = S_OK;
	UINT32 FramesAvailable = 0;
	BYTE *Data = nullptr;
	DWORD dwCaptureFlags;
	UINT64 u64DevicePosition = 0;
	UINT64 u64QPCPosition = 0;
	DWORD cbBytesToCapture = 0;

	EnterCriticalSection(&m_CritSec);

	// If this flag is set, we have already queued up the async call to finalize the WAV header
	// So we don't want to grab or write any more data that would possibly give us an invalid size
	if ((GetState() == DeviceState::DeviceStateStopping) ||
		(GetState() == DeviceState::DeviceStateFlushing))
	{
		goto exit;
	}

	// This should equal the buffer size when GetBuffer() is called
	hr = m_AudioCaptureClient->GetNextPacketSize(&FramesAvailable);
	if (FAILED(hr))
	{
		goto exit;
	}

	if (FramesAvailable > 0)
	{
		cbBytesToCapture = FramesAvailable * m_MixFormat->nBlockAlign;

		// Get sample buffer
		hr = m_AudioCaptureClient->GetBuffer(&Data, &FramesAvailable, &dwCaptureFlags, &u64DevicePosition, &u64QPCPosition);
		if (FAILED(hr))
		{
			goto exit;
		}

		if (dwCaptureFlags & AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY)
		{
			// Pass down a discontinuity flag in case the app is interested and reset back to capturing
			SetState(DeviceState::DeviceStateDiscontinuity, S_OK, true);
			SetState(DeviceState::DeviceStateCapturing, S_OK, false);
		}

		// Zero out sample if silence
		if ((dwCaptureFlags & AUDCLNT_BUFFERFLAGS_SILENT) || IsSilence)
		{
			memset(Data, 0, FramesAvailable * m_MixFormat->nBlockAlign);
		}

		// Store data in array
		std::vector<byte> dataByte(Data, Data+cbBytesToCapture);

		// Release buffer back
		m_AudioCaptureClient->ReleaseBuffer(FramesAvailable);

		// Update plotter data
		m_pAudioSink->OnSamples(dataByte.data(), cbBytesToCapture);
		//ProcessScopeData(dataByte.data(), cbBytesToCapture);

		// Write File and async store
		//m_WAVDataWriter->WriteBytes( dataByte );

	}

exit:
	LeaveCriticalSection(&m_CritSec);

	return hr;
}

HRESULT Audio::AudioCaptureDevice::SetAudioSink(IAudioSink* pAudioSink)
{
	if (GetState() == DeviceState::DeviceStateInitialized || GetState() == DeviceState::DeviceStateUnInitialized)
	{
		m_pAudioSink = pAudioSink;
		m_pAudioSink->OnFormatChange(m_MixFormat);
		return S_OK;
	}
	return E_NOT_VALID_STATE;
}

#pragma region SynchronizeCaptureDevice
//#define EXIT_ON_ERROR(hres)  \
//			  if (FAILED(hres)) return hres
//
//HRESULT Audio::AudioRecorder::Initialize(const shared_ptr<IAudioSink> &pSink, double requestSampleIntervalSec )
//{
//	if (state != Uninit)
//		return E_NOT_VALID_STATE;
//	ComPtr<IMMDeviceEnumerator> pEnumerator = NULL;
//	ComPtr<IMMDevice> pDevice = NULL;
//	ComPtr<IAudioClient> pAudioClient = NULL;
//	ComPtr<IAudioCaptureClient> pCaptureClient = NULL;
//
//	hnsRequestedDuration = static_cast<REFERENCE_TIME>(requestSampleIntervalSec * REFTIMES_PER_SEC);
//	hr = CoCreateInstance(
//		CLSID_MMDeviceEnumerator, NULL,
//		CLSCTX_ALL, IID_IMMDeviceEnumerator,
//		(void**) &pEnumerator);
//	EXIT_ON_ERROR(hr);
//
//	hr = pEnumerator->GetDefaultAudioEndpoint(
//		eCapture, eConsole, &pDevice);
//	EXIT_ON_ERROR(hr);
//
//	hr = pDevice->Activate(
//		IID_IAudioClient, CLSCTX_ALL,
//		NULL, (void**) &pAudioClient);
//	EXIT_ON_ERROR(hr);
//
//	hr = pAudioClient->GetMixFormat(&pwfx);
//	EXIT_ON_ERROR(hr);
//
//	ConvertMixFormat(pwfx);
//
//	hr = pAudioClient->Initialize(
//		AUDCLNT_SHAREMODE_SHARED,
//		0,
//		hnsRequestedDuration,
//		0,
//		pwfx,
//		NULL);
//	EXIT_ON_ERROR(hr);
//
//	// Get the size of the allocated buffer.
//	hr = pAudioClient->GetBufferSize(&bufferFrameCount);
//	EXIT_ON_ERROR(hr);
//
//	hr = pAudioClient->GetService(
//		IID_IAudioCaptureClient,
//		(void**) &pCaptureClient);
//	EXIT_ON_ERROR(hr);
//
//	// Notify the audio sink which format to use.
//	hr = pSink->OnFormatChange(pwfx);
//	EXIT_ON_ERROR(hr);
//
//	// Calculate the actual duration of the allocated buffer.
//	hnsActualDuration = static_cast<REFERENCE_TIME>((double) REFTIMES_PER_SEC *
//		bufferFrameCount / pwfx->nSamplesPerSec);
//
//	m_pCaptureClient = std::move(pCaptureClient);
//	m_pAudioClient = std::move(pAudioClient);
//
//	state = Ready;
//
//	return S_OK;
//}
//
//HRESULT Audio::AudioRecorder::StartCapturing()
//{
//	if (state != Ready)
//		return E_NOT_VALID_STATE;
//	if (m_pAudioClient && m_pCaptureClient && m_pSink)
//		return E_NOT_VALID_STATE;
//
//	hr = m_pAudioClient->Start();  // Start recording.
//	EXIT_ON_ERROR(hr);
//
//	hr = m_pSink->OnCaptureStarted();
//
//	bDone = false;
//	state = Recording;
//
//	// Each loop fills about half of the shared buffer.
//	while (!bDone)
//	{
//		// Sleep for half the buffer duration.
//		Sleep(static_cast<DWORD>(hnsActualDuration / 2 / REFTIMES_PER_MILLISEC));
//
//		hr = m_pCaptureClient->GetNextPacketSize(&packetLength);
//		EXIT_ON_ERROR(hr);
//
//		while (packetLength != 0)
//		{
//			// Get the available data in the shared buffer.
//			hr = m_pCaptureClient->GetBuffer(
//				&pData,
//				&numFramesAvailable,
//				&flags, NULL, NULL);
//			EXIT_ON_ERROR(hr);
//
//			if (flags & AUDCLNT_BUFFERFLAGS_SILENT)
//			{
//				pData = NULL;  // Tell OnSamples to write silence.
//			}
//
//			bool bClientDone = FALSE;
//			// Copy the available capture data to the audio sink.
//			hr = m_pSink->OnSamples(
//				pData, numFramesAvailable);
//			if (bClientDone)
//				bDone = true;
//			EXIT_ON_ERROR(hr);
//
//			hr = m_pCaptureClient->ReleaseBuffer(numFramesAvailable);
//			EXIT_ON_ERROR(hr);
//
//			hr = m_pCaptureClient->GetNextPacketSize(&packetLength);
//			EXIT_ON_ERROR(hr);
//		}
//	}
//
//	hr = m_pAudioClient->Stop();  // Stop recording.
//	EXIT_ON_ERROR(hr);
//	hr = m_pSink->OnCaptureStopped();
//	EXIT_ON_ERROR(hr);
//
//	return S_OK;
//}
//
//HRESULT Audio::AudioRecorder::StartCaptureAsync()
//{
//	if (state != Ready)
//		return E_NOT_VALID_STATE;
//	m_pCaptureThread = make_unique<std::thread>([this](){
//		HRESULT hr = StartCapturing();
//		if (FAILED(hr))
//			state = Error;
//		else
//			state = Stopped;
//	});
//	return S_OK;
//}
//
//HRESULT Audio::AudioRecorder::StopCapture()
//{
//	bDone = true;
//	if (!m_pCaptureThread)
//		return E_NOT_VALID_STATE;
//	m_pCaptureThread->join();
//	m_pCaptureThread = nullptr;
//	state = Stopped;
//
//	return S_OK;
//}
//
//Audio::AudioRecorder::AudioRecorder()
//{
//	state = Uninit;
//}
//
//Audio::AudioRecorder::States Audio::AudioRecorder::State() const
//{
//	return state;
//}
//
//HRESULT Audio::AudioRecorder::Release()
//{
//	StopCapture();
//
//	state = Uninit;
//	m_pSink = nullptr;
//	m_pAudioClient.Reset();
//	m_pCaptureClient.Reset();
//	return S_OK;
//}
//
//void Audio::AudioRecorder::ConvertMixFormat(WAVEFORMATEX *pWfx)
//{
//	// convert from Float to PCM and from WAVEFORMATEXTENSIBLE to WAVEFORMATEX
//	if ((pWfx->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) ||
//		((pWfx->wFormatTag == WAVE_FORMAT_EXTENSIBLE) &&
//		(reinterpret_cast<WAVEFORMATEXTENSIBLE *>(pWfx)->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT)))
//	{
//		pWfx->wFormatTag = WAVE_FORMAT_PCM;
//		pWfx->wBitsPerSample = 16;
//		pWfx->nBlockAlign = pWfx->nChannels * 2;    // (nChannels * wBitsPerSample) / 8
//		pWfx->nAvgBytesPerSec = pWfx->nSamplesPerSec * pWfx->nBlockAlign;
//		pWfx->cbSize = 0;
//	}
//}
//
//Audio::AudioRecorder::~AudioRecorder()
//{
//
//}

#pragma endregion

