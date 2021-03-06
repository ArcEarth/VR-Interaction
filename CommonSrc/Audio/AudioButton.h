#pragma once
#include "AudioCaptureDevice.h"

namespace Platform
{
	namespace Audio
	{
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
			public IAudioSink
		{
			typedef void(ButtonStateChangedHandlerType)(AudioButton* sender, const ButtonStateChangedEventArgs *e);
		private:
			HRESULT OnSendScopeData(IMFAsyncResult* pResult);
		public:
			AudioButton();
			~AudioButton();

			virtual HRESULT OnFormatChange(LPCWAVEFORMATEX pwfx);

			virtual HRESULT OnSamples(BYTE* pData, UINT32 numFramesAvailable);

			MFAsyncCallback<AudioButton, &OnSendScopeData> cb_SendScopeData;

			typedef function<ButtonStateChangedHandlerType> ButtonStateChangedEventHandler;

			Platform::Event<const ButtonStateChangedEventArgs *> ButtonStateChanged;

			inline ButtonState CurrentState() const{
				return m_CurrentState;
			}
		private:


			RenderSampleType sampleType;

			ButtonState							 m_CurrentState;
			WAVEFORMATEX						 m_MixFormat;
			vector<int>							 m_ScopedWaveData;
			UINT32                               m_WaveDataMax;
			UINT32                               m_WaveDataFilled;
		};

	}
}