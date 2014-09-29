#include "AudioButton.h"

#include <DirectXMath.h>

using namespace Audio;
using namespace Microsoft::WRL;

const IID IID_IMFAsyncCallback = __uuidof(IMFAsyncCallback);

#define MILLISECONDS_TO_VISUALIZE 200
AudioButton::AudioButton()
	: cb_SendScopeData(this)
{
}


AudioButton::~AudioButton()
{
}

HRESULT AudioButton::OnFormatChange(LPCWAVEFORMATEX pwfx)
{
	sampleType = CalculateMixFormatType(pwfx);
	HRESULT hr = S_OK;

	m_MixFormat = *pwfx;

	m_WaveDataFilled = 0;
	m_WaveDataMax = (MILLISECONDS_TO_VISUALIZE * m_MixFormat.nSamplesPerSec) / 1000;

	m_ScopedWaveData.resize(m_WaveDataMax + 1);
	if (m_ScopedWaveData.size() < m_WaveDataMax + 1)
	{
		return E_OUTOFMEMORY;
	}

	// Only Support 16 bit Audio for now
	if (m_MixFormat.wBitsPerSample == 16)
	{
		m_ScopedWaveData[m_WaveDataMax] = -32768;  // INT16_MAX
	}
	else
	{
		hr = S_FALSE;
	}

	return hr;
}

HRESULT AudioButton::OnSamples(BYTE* pData, UINT32 cbBytes)
{
	HRESULT hr = S_OK;

	// We don't have a valid pointer array, so return.  This could be the case if we aren't
	// dealing with 16-bit audio
	//if (m_pScopedWaveData == nullptr)
	//{
	//	return S_FALSE;
	//}

	DWORD dwNumPoints = cbBytes / m_MixFormat.nChannels / (m_MixFormat.wBitsPerSample / 8);

	// Read the 16-bit samples from channel 0
	INT16 *pi16 = (INT16*) pData;

	for (DWORD i = 0; m_WaveDataFilled < m_WaveDataMax && i < dwNumPoints; i++)
	{
		m_ScopedWaveData[m_WaveDataFilled] = *pi16;
		pi16 += m_MixFormat.nChannels;

		m_WaveDataFilled++;
	}

	// Send off the event and get ready for the next set of samples
	if (m_WaveDataFilled == m_WaveDataMax)
	{
		ComPtr<CAsyncState> spState = Make<CAsyncState>(&m_ScopedWaveData, m_WaveDataMax + 1);

		if (SUCCEEDED(hr))
		{
			MFPutWorkItem2(MFASYNC_CALLBACK_QUEUE_MULTITHREADED, 0, &cb_SendScopeData, spState.Get());
		}

		m_WaveDataFilled = 0;
	}

	return hr;
}

HRESULT AudioButton::OnSendScopeData(IMFAsyncResult* pResult)
{
	HRESULT hr = S_OK;
	ComPtr<CAsyncState> pState = nullptr;

	hr = pResult->GetState(&pState);
	if (SUCCEEDED(hr))
	{
		const auto& data = *pState->Data;

		double ymax = (double)abs(data.back());

		// Average Amplitude of the wave in this time-window
		double A = 0;
		for (auto frame : data)
		{
			A += (double) std::abs(frame);
		}

		A /= (double) pState->Size * DirectX::XM_PIDIV2; // Convert the integral to amplitude

		pState.Reset();

		ButtonState state = (A > ymax / 64) ? Pressed : Released;
		if (state != m_CurrentState)
		{
			ButtonStateChangedEventArgs e;
			e.NewState = state;
			e.OldState = m_CurrentState;
			m_CurrentState = state;
			ButtonStateChanged(this, &e);
		}
	}

	return S_OK;
}
