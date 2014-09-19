#pragma once
#include "AudioCaptureDevice.h"

namespace Audio
{
	using boost::signals2::signal;
	using std::shared_ptr;
	using std::vector;
	using std::function;

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

	class AudioButton :
		public Audio::IAudioSink
	{
		typedef void(ButtonStateChangedHandlerType)(AudioButton* sender, const shared_ptr<ButtonStateChangedEventArgs> &e);
	private:
		HRESULT OnSendScopeData(IMFAsyncResult* pResult);
	public:
		AudioButton();
		~AudioButton();

		virtual HRESULT OnFormatChange(LPCWAVEFORMATEX pwfx);

		virtual HRESULT OnSamples(BYTE* pData, UINT32 numFramesAvailable);

		MFAsyncCallback<AudioButton, &OnSendScopeData> cb_SendScopeData;

		typedef function<ButtonStateChangedHandlerType> ButtonStateChangedEventHandler;

		signal<ButtonStateChangedHandlerType> ButtonStateChanged;

		inline ButtonState CurrentState() const{
			return m_CurrentState;
		}
	private:


		RenderSampleType sampleType;

		ButtonState							 m_CurrentState;
		WAVEFORMATEX						 m_MixFormat;
		shared_ptr<vector<int>>	 m_pScopedWaveData;
		UINT32                               m_WaveDataMax;
		UINT32                               m_WaveDataFilled;
	};

}