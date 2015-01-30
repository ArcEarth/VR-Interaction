#include "stdafx.h"
#include "resource.h"
#include <ole2.h>
#include <Shlobj.h>
#include <NuiApi.h>
#include "SpeechCommandStream.h"
#include "NuiUtility.h"
#include <sphelper.h>

INuiSpStream::INuiSpStream(INuiSensor* pNuiSensor)
{
	using namespace Microsoft::WRL;
	ComPtr<INuiAudioBeam>      pNuiAudioSource = NULL;
	ComPtr<IMediaObject>       pDMO = NULL;
	ComPtr<IPropertyStore>     pPropertyStore = NULL;
	ComPtr<IStream>            pStream = NULL;

	// Get the audio source
	HRESULT hr = pNuiSensor->NuiGetAudioSource(&pNuiAudioSource);
	if (SUCCEEDED(hr))
	{
		hr = pNuiAudioSource.As<IMediaObject>(&pDMO);
		//hr = pNuiAudioSource->QueryInterface(IID_IMediaObject, (void**)&pDMO);

		if (SUCCEEDED(hr))
		{
			hr = pNuiAudioSource.As(&pPropertyStore);
			//hr = pNuiAudioSource->QueryInterface(IID_IPropertyStore, (void**)&pPropertyStore);
	
			// Set AEC-MicArray DMO system mode. This must be set for the DMO to work properly.
			// Possible values are:
			//   SINGLE_CHANNEL_AEC = 0
			//   OPTIBEAM_ARRAY_ONLY = 2
			//   OPTIBEAM_ARRAY_AND_AEC = 4
			//   SINGLE_CHANNEL_NSAGC = 5
			PROPVARIANT pvSysMode;
			PropVariantInit(&pvSysMode);
			pvSysMode.vt = VT_I4;
			pvSysMode.lVal = (LONG)(2); // Use OPTIBEAM_ARRAY_ONLY setting. Set OPTIBEAM_ARRAY_AND_AEC instead if you expect to have sound playing from speakers.
			pPropertyStore->SetValue(MFPKEY_WMAAECMA_SYSTEM_MODE, pvSysMode);
			PropVariantClear(&pvSysMode);

			// Set DMO output format
			WAVEFORMATEX wfxOut = {AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0};
			DMO_MEDIA_TYPE mt = {0};
			MoInitMediaType(&mt, sizeof(WAVEFORMATEX));
	
			mt.majortype = MEDIATYPE_Audio;
			mt.subtype = MEDIASUBTYPE_PCM;
			mt.lSampleSize = 0;
			mt.bFixedSizeSamples = TRUE;
			mt.bTemporalCompression = FALSE;
			mt.formattype = FORMAT_WaveFormatEx;	
			memcpy(mt.pbFormat, &wfxOut, sizeof(WAVEFORMATEX));
	
			hr = pDMO->SetOutputType(0, &mt, 0);

			if (SUCCEEDED(hr))
			{
				m_pAudioStream = new KinectAudioStream(pDMO.Get());

				//hr = m_pAudioStream->QueryInterface(IID_IStream, (void**)&pStream);
				hr = m_pAudioStream.As<IStream>(&pStream);
				if (SUCCEEDED(hr))
				{
					hr = CoCreateInstance(CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpStream), (void**)&m_pSpStream);

					if (SUCCEEDED(hr))
					{
						hr = m_pSpStream->SetBaseStream(pStream.Get(), SPDFID_WaveFormatEx, &wfxOut);
					}
				}
			}

			MoFreeMediaType(&mt);
		}
	}
}

SpeechCommandStream::SpeechCommandStream(INuiSensor* pNuiSensor , LPCWSTR sGrammerFileName)
{
	m_GrammarFileName = sGrammerFileName;
	m_pNuiSensor = pNuiSensor;
	m_pKinectAudioStream = nullptr;
	m_pSpeechStream = nullptr;
	m_pSpeechRecognizer = nullptr;
	m_pSpeechContext = nullptr;
	m_pSpeechGrammar = nullptr;
	m_hSpeechEvent = nullptr;
	m_hSpeechProcess = nullptr;
	m_paused = true;

	m_CommandBuffer = ActionNone;
	if (m_pNuiSensor) 
	{
		//m_pNuiSensor->AddRef();
		Initialize();
	}
}

HRESULT SpeechCommandStream::Initialize()
{
	HRESULT hr;
	//printf("Initializing Speech Command Stream...\n");
	//HRESULT hr = InitializeAudioStream();
	//if (FAILED(hr))
	//{
	//	printf("Could not initialize audio stream.");
	//	return hr;
	//}
	hr = CreateSpeechRecognizer();
	if (FAILED(hr))
	{
		printf("Could not create speech recognizer. Please ensure that Microsoft Speech SDK and other sample requirements are installed.");
		return hr;
	}

	hr = LoadSpeechGrammar();
	if (FAILED(hr))
	{
		printf("Could not load speech grammar. Please ensure that grammar configuration file was properly deployed.");
		return hr;
	}
	printf("Speech Command Stream Initialized.\n");
	return hr;
}

HRESULT SpeechCommandStream::StartStream()
{
	if (!m_paused) return E_FAIL;

	printf("Starting Speech Command Stream...\n");

	HRESULT hr = StartSpeechRecognition();
	if (FAILED(hr))
	{
		printf("Could not start recognizing speech.");
		return hr;
	}
	m_paused = false;
	m_hSpeechProcess = CreateThread(NULL, 0, SpeechProcessThread, this, 0, NULL );
	printf("Speech Command Stream Started.\n");
	return hr;
}

void SpeechCommandStream::PauseStream(bool pause)
{
	if (m_paused == pause)
		return;
	m_paused = pause;
	HRESULT hr;
	if (!m_paused)
	{
		//hr = m_pKinectAudioStream->StartCapture();
		//if (FAILED(hr)) return;
		// Specify that engine should always be reading audio
		hr = m_pSpeechRecognizer->SetRecoState(SPRST_ACTIVE_ALWAYS);
		if (FAILED(hr)) return;
		// Ensure that engine is recognizing speech and not in paused state
		hr = m_pSpeechContext->Resume(0);
		if (FAILED(hr)) return;
		m_hSpeechProcess = CreateThread(NULL, 0, SpeechProcessThread, this, 0, NULL );
		printf("Speech Command Stream Started.\n");
	} else
	{
		//HRESULT hr = m_pKinectAudioStream->StopCapture();
		//if (FAILED(hr)) return;
		// Specify that engine should always be reading audio
		hr = m_pSpeechRecognizer->SetRecoState(SPRST_INACTIVE);
		if (FAILED(hr)) return;
		// Ensure that engine is recognizing speech and not in paused state
		hr = m_pSpeechContext->Pause(0);
		if (FAILED(hr)) return;
		//WaitForSingleObject(m_hSpeechProcess,INFINITE);
		CloseHandle(m_hSpeechProcess);
		m_hSpeechProcess = nullptr;
	}
}

SpeechCommandStream::FrameType SpeechCommandStream::GetStreamFrame()
{
	auto value = m_CommandBuffer;
	m_CommandBuffer = ActionNone;
	return value;
}

SpeechCommandStream::~SpeechCommandStream(void)
{
	PauseStream(true);
	SafeRelease(m_pKinectAudioStream);
	SafeRelease(m_pSpeechStream);
	//SafeRelease(m_pSpeechGrammar);
	//SafeRelease(m_pSpeechContext);
	//SafeRelease(m_pSpeechRecognizer);
	//SafeRelease(m_cpAudio);
	//SafeRelease(m_pNuiSensor);
}

DWORD WINAPI SpeechCommandStream::SpeechProcessThread(LPVOID pParam)
{
	SpeechCommandStream *pthis = (SpeechCommandStream *) pParam;
	return pthis->SpeechProcessThread();
}

//-------------------------------------------------------------------
// Nui_ProcessThread
//
// Thread to handle Kinect processing
//-------------------------------------------------------------------
DWORD WINAPI SpeechCommandStream::SpeechProcessThread()
{
	const int speecheventCount = 1;
	HANDLE speechhEvents[speecheventCount];

	// Main thread loop
	while ( !m_paused )
	{
		//speech
		speechhEvents[0] = m_hSpeechEvent;

		// Check to see if we have either a message (by passing in QS_ALLINPUT)
		// Or a speech event (hEvents)
		DWORD dwEvent = MsgWaitForMultipleObjectsEx(speecheventCount, speechhEvents, INFINITE, QS_ALLINPUT, MWMO_INPUTAVAILABLE);

		// Check if this is an event we're waiting on and not a timeout or message
		if (WAIT_OBJECT_0 == dwEvent)
		{
			ProcessSpeech();
		}
	}

	return 0;
}

/// <summary>
/// Initialize Kinect audio stream object.
/// </summary>
/// <returns>
/// <para>S_OK on success, otherwise failure code.</para>
/// </returns>
HRESULT SpeechCommandStream::InitializeAudioStream()
{
	INuiAudioBeam*      pNuiAudioSource = NULL;
	IMediaObject*       pDMO = NULL;
	IPropertyStore*     pPropertyStore = NULL;
	IStream*            pStream = NULL;

	// Get the audio source
	HRESULT hr = m_pNuiSensor->NuiGetAudioSource(&pNuiAudioSource);
	if (SUCCEEDED(hr))
	{
		hr = pNuiAudioSource->QueryInterface(IID_IMediaObject, (void**)&pDMO);

		if (SUCCEEDED(hr))
		{
			hr = pNuiAudioSource->QueryInterface(IID_IPropertyStore, (void**)&pPropertyStore);
	
			// Set AEC-MicArray DMO system mode. This must be set for the DMO to work properly.
			// Possible values are:
			//   SINGLE_CHANNEL_AEC = 0
			//   OPTIBEAM_ARRAY_ONLY = 2
			//   OPTIBEAM_ARRAY_AND_AEC = 4
			//   SINGLE_CHANNEL_NSAGC = 5
			PROPVARIANT pvSysMode;
			PropVariantInit(&pvSysMode);
			pvSysMode.vt = VT_I4;
			pvSysMode.lVal = (LONG)(2); // Use OPTIBEAM_ARRAY_ONLY setting. Set OPTIBEAM_ARRAY_AND_AEC instead if you expect to have sound playing from speakers.
			pPropertyStore->SetValue(MFPKEY_WMAAECMA_SYSTEM_MODE, pvSysMode);
			PropVariantClear(&pvSysMode);

			// Set DMO output format
			WAVEFORMATEX wfxOut = {AudioFormat, AudioChannels, AudioSamplesPerSecond, AudioAverageBytesPerSecond, AudioBlockAlign, AudioBitsPerSample, 0};
			DMO_MEDIA_TYPE mt = {0};
			MoInitMediaType(&mt, sizeof(WAVEFORMATEX));
	
			mt.majortype = MEDIATYPE_Audio;
			mt.subtype = MEDIASUBTYPE_PCM;
			mt.lSampleSize = 0;
			mt.bFixedSizeSamples = TRUE;
			mt.bTemporalCompression = FALSE;
			mt.formattype = FORMAT_WaveFormatEx;	
			memcpy(mt.pbFormat, &wfxOut, sizeof(WAVEFORMATEX));
	
			hr = pDMO->SetOutputType(0, &mt, 0);

			if (SUCCEEDED(hr))
			{
				m_pKinectAudioStream = new KinectAudioStream(pDMO);

				hr = m_pKinectAudioStream->QueryInterface(IID_IStream, (void**)&pStream);

				if (SUCCEEDED(hr))
				{
					hr = CoCreateInstance(CLSID_SpStream, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpStream), (void**)&m_pSpeechStream);

					if (SUCCEEDED(hr))
					{
						hr = m_pSpeechStream->SetBaseStream(pStream, SPDFID_WaveFormatEx, &wfxOut);
					}
				}
			}

			MoFreeMediaType(&mt);
		}
	}

	SafeRelease(pStream);
	SafeRelease(pPropertyStore);
	SafeRelease(pDMO);
	SafeRelease(pNuiAudioSource);

	return hr;
}

/// <summary>
/// Create speech recognizer that will read Kinect audio stream data.
/// </summary>
/// <returns>
/// <para>S_OK on success, otherwise failure code.</para>
/// </returns>
HRESULT SpeechCommandStream::CreateSpeechRecognizer()
{
	using namespace Microsoft::WRL;
	ComPtr<ISpObjectToken> cpEngineToken = NULL;
	//ComPtr<ISpObjectToken> cpAudio = NULL;

	HRESULT hr = CoCreateInstance(CLSID_SpInprocRecognizer, NULL, CLSCTX_INPROC_SERVER, __uuidof(ISpRecognizer), (void**)&m_pSpeechRecognizer);
	//m_pSpeechRecognizer->AddRef();
	if (FAILED(hr)) return hr;

	hr = SpGetDefaultTokenFromCategoryId(SPCAT_AUDIOIN, &m_cpAudio);

	hr = m_pSpeechRecognizer->SetInput(m_cpAudio.Get(), TRUE);
	//hr = m_pSpeechRecognizer->SetInput(m_pSpeechStream, FALSE);
	if (FAILED(hr)) return hr;

	//hr = SpFindBestToken(SPCAT_RECOGNIZERS,L"Language=409;Kinect=True",NULL,&cpEngineToken);
	//if (FAILED(hr)) return hr;

	//hr = m_pSpeechRecognizer->SetRecognizer(cpEngineToken.Get());
	//if (FAILED(hr)) return hr;

	hr = m_pSpeechRecognizer->CreateRecoContext(&m_pSpeechContext);
	if (FAILED(hr)) return hr;

	return hr;
}

/// <summary>
/// Load speech recognition grammar into recognizer.
/// </summary>
/// <returns>
/// <para>S_OK on success, otherwise failure code.</para>
/// </returns>
HRESULT SpeechCommandStream::LoadSpeechGrammar()
{
	HRESULT hr = m_pSpeechContext->CreateGrammar(1, &m_pSpeechGrammar);
	if (FAILED(hr)) return hr;
	// Populate recognition grammar from file
	hr = m_pSpeechGrammar->LoadCmdFromFile(m_GrammarFileName.c_str(), SPLO_STATIC);
	if (FAILED(hr)) return hr;

	return hr;
}

/// <summary>
/// Start recognizing speech asynchronously.
/// </summary>
/// <returns>
/// <para>S_OK on success, otherwise failure code.</para>
/// </returns>
HRESULT SpeechCommandStream::StartSpeechRecognition()
{
	HRESULT hr;
	//HRESULT hr = m_pKinectAudioStream->StartCapture();
	//if (FAILED(hr)) return hr;
	// Specify that all top level rules in grammar are now active
	hr = m_pSpeechGrammar->SetRuleState(NULL, NULL, SPRS_ACTIVE);
	if (FAILED(hr)) return hr;
	// Specify that engine should always be reading audio
	hr = m_pSpeechRecognizer->SetRecoState(SPRST_ACTIVE_ALWAYS);
	if (FAILED(hr)) return hr;
	// Specify that we're only interested in receiving recognition events
	hr = m_pSpeechContext->SetInterest(SPFEI(SPEI_RECOGNITION), SPFEI(SPEI_RECOGNITION));
	if (FAILED(hr)) return hr;
	// Ensure that engine is recognizing speech and not in paused state
	hr = m_pSpeechContext->Resume(0);
	if (FAILED(hr)) return hr;

	m_hSpeechEvent = m_pSpeechContext->GetNotifyEventHandle();

	return hr;
}

/// <summary>
/// Process recently triggered speech recognition events.
/// </summary>
void SpeechCommandStream::ProcessSpeech()
{
	const float ConfidenceThreshold = 0.3f;

	SPEVENT curEvent;
	ULONG fetched = 0;
	HRESULT hr = S_OK;

	m_pSpeechContext->GetEvents(1, &curEvent, &fetched);

	while (fetched > 0)
	{
		switch (curEvent.eEventId)
		{
			case SPEI_RECOGNITION:
				if (SPET_LPARAM_IS_OBJECT == curEvent.elParamType)
				{
					// this is an ISpRecoResult
					ISpRecoResult* result = reinterpret_cast<ISpRecoResult*>(curEvent.lParam);
					SPPHRASE* pPhrase = NULL;
					
					hr = result->GetPhrase(&pPhrase);
					if (SUCCEEDED(hr))
					{
						if ((pPhrase->pProperties != NULL) && (pPhrase->pProperties->pFirstChild != NULL))
						{
							const SPPHRASEPROPERTY* pSemanticTag = pPhrase->pProperties->pFirstChild;
							if (pSemanticTag->SREngineConfidence > ConfidenceThreshold)
							{
								SpeechCommands action = MapSpeechTagToAction(pSemanticTag->pszValue);
								m_CommandBuffer = action;
							}
						}
						::CoTaskMemFree(pPhrase);
					}
				}
				break;
		}

		m_pSpeechContext->GetEvents(1, &curEvent, &fetched);
	}

	return;
}

SpeechCommands SpeechCommandStream::MapSpeechTagToAction(LPCWSTR pszSpeechTag)
{
	struct SpeechTagToAction
	{
		std::wstring pszSpeechTag;
		SpeechCommands action;
	};
	const SpeechTagToAction Map[] =
	{
		{L"SPATIAL", Spatial},
		{L"REFERENCE", Reference},
		{L"SMALLER", Smaller},
		{L"BIGGER", Bigger},
		{L"EDIT", Edit},
		{L"PAINT", Paint},
		{L"SCAN", Scan},
		{L"DONE", Done},
		{L"LONGER", Longer},
		{L"SHORTER", Shorter},
		{L"HIGHER", Higher},
		{L"LOWER", Lower},
		{L"STRONGER", Stronger},
		{L"FATTER", Fatter},
		{L"THINNER", Thinner},
		{L"CANCEL", Cancel},
		{L"ATTACH", Attach},
		{L"DETACH", Detach},
		{L"RESET", Reset},
		{L"LOAD", Load},
		{L"SAVE", Save},
	};

	SpeechCommands action = ActionNone;

	for (int i = 0; i < _countof(Map); ++i)
	{
		if (0 == wcscmp(Map[i].pszSpeechTag.c_str(), pszSpeechTag))
		{
			action = Map[i].action;
			break;
		}
	}

	return action;
}
