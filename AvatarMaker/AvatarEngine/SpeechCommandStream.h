#pragma once


#include "KinectAudioStream.h"
// For configuring DMO properties
#include <wmcodecdsp.h>
// For FORMAT_WaveFormatEx and such
#include <uuids.h>
// For speech APIs
#include <sapi.h>

#include <wrl\client.h>

#define __MACRO_COMMAND(C) C,
enum SpeechCommands
{
#include "SpeechCommands.h"
};
#undef __MACRO_COMMAND

class INuiSpStream

{
	INuiSpStream(INuiSensor* pNuiSensor);
	~INuiSpStream()
	{}
	operator ISpStream*()
	{
		return m_pSpStream.Get();
	}

private:
	Microsoft::WRL::ComPtr<KinectAudioStream>	m_pAudioStream;
	Microsoft::WRL::ComPtr<ISpStream>			m_pSpStream;
};


class SpeechCommandStream
{
public:
	typedef SpeechCommands FrameType;
public:
	SpeechCommandStream(INuiSensor* pNuiSensor , LPCWSTR sGrammerFileName);
	~SpeechCommandStream(void);

	HRESULT	Initialize();
	HRESULT StartStream();
	void PauseStream(bool pause);
	FrameType GetStreamFrame();

protected:
	/// <summary>
	/// Initialize Kinect audio stream object.
	/// </summary>
	/// <returns>S_OK on success, otherwise failure code.</returns>
	HRESULT                 InitializeAudioStream();

	/// <summary>
	/// Create speech recognizer that will read Kinect audio stream data.
	/// </summary>
	/// <returns>
	/// <para>S_OK on success, otherwise failure code.</para>
	/// </returns>
	HRESULT                 CreateSpeechRecognizer();

	/// <summary>
	/// Load speech recognition grammar into recognizer.
	/// </summary>
	/// <returns>
	/// <para>S_OK on success, otherwise failure code.</para>
	/// </returns>
	HRESULT                 LoadSpeechGrammar();

	/// <summary>
	/// Start recognizing speech asynchronously.
	/// </summary>
	/// <returns>
	/// <para>S_OK on success, otherwise failure code.</para>
	/// </returns>
	HRESULT                 StartSpeechRecognition();

	/// <summary>
	/// Process recently triggered speech recognition events.
	/// </summary>
	void                    ProcessSpeech();

	SpeechCommands MapSpeechTagToAction(LPCWSTR pszSpeechTag);

private:
	static DWORD WINAPI     SpeechProcessThread(LPVOID pParam);
	DWORD WINAPI            SpeechProcessThread();
private:
	INuiSensor*				m_pNuiSensor;

	std::wstring			m_GrammarFileName;
	// Audio stream captured from Kinect.
	KinectAudioStream*      m_pKinectAudioStream;
	// Stream given to speech recognition engine
	ISpStream*              m_pSpeechStream;
	// Audio stream which will be set as input of recongizer
	Microsoft::WRL::ComPtr<ISpObjectToken>			m_cpAudio;
	// Speech recognizer
	Microsoft::WRL::ComPtr<ISpRecognizer>          m_pSpeechRecognizer;
	// Speech recognizer context
	Microsoft::WRL::ComPtr<ISpRecoContext>         m_pSpeechContext;
	// Speech grammar
	Microsoft::WRL::ComPtr<ISpRecoGrammar>         m_pSpeechGrammar;
	// Event triggered when we detect speech recognition
	HANDLE                  m_hSpeechEvent;
	// Thread that we use to process the speech
	HANDLE					m_hSpeechProcess;

	SpeechCommands			m_CommandBuffer;
	bool					m_paused;
};

