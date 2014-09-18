#pragma once
#include "AudioCaptureDevice.h"

namespace Audio
{
	enum ButtonState
	{
		Unknown,
		Released,
		Pressed,
	};

	struct ButtonStateChangedEventArgs
	{
	public:
		ButtonState OldState;
		ButtonState NewState;
	};

	class MicrophoneSimulateButton :
		public Audio::IAudioSink
	{
		typedef void(ButtonStateChangedHandlerType)(MicrophoneSimulateButton* sender, const std::shared_ptr<ButtonStateChangedEventArgs> &e);
	public:
		MicrophoneSimulateButton();
		~MicrophoneSimulateButton();

		virtual HRESULT OnFormatChange(LPCWAVEFORMATEX pwfx);

		virtual HRESULT OnSamples(BYTE* pData, UINT32 numFramesAvailable);

		METHODASYNCCALLBACK(MicrophoneSimulateButton, SendScopeData, OnSendScopeData);

		typedef std::function<ButtonStateChangedHandlerType> ButtonStateChangedEventHandler;

		boost::signals2::signal<ButtonStateChangedHandlerType> ButtonStateChanged;

		inline ButtonState CurrentState() const{
			return m_CurrentState;
		}
	private:

		HRESULT OnSendScopeData(IMFAsyncResult* pResult);

		RenderSampleType sampleType;

		ButtonState							 m_CurrentState;
		WAVEFORMATEX						 m_MixFormat;
		std::shared_ptr<std::vector<int>>	 m_pScopedWaveData;
		UINT32                               m_WaveDataMax;
		UINT32                               m_WaveDataFilled;
	};

}