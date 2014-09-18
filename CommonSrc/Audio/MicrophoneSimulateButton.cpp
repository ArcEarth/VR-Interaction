#include "MicrophoneSimulateButton.h"

#include <wrl\async.h>
using namespace Audio;
using namespace Microsoft::WRL;

const IID IID_IMFAsyncCallback = __uuidof(IMFAsyncCallback);

#define MILLISECONDS_TO_VISUALIZE 500
MicrophoneSimulateButton::MicrophoneSimulateButton()
{
}


MicrophoneSimulateButton::~MicrophoneSimulateButton()
{
}

HRESULT MicrophoneSimulateButton::OnFormatChange(LPCWAVEFORMATEX pwfx)
{
	sampleType = CalculateMixFormatType(pwfx);
	HRESULT hr = S_OK;

	m_MixFormat = *pwfx;

	m_WaveDataFilled = 0;
	m_WaveDataMax = (MILLISECONDS_TO_VISUALIZE * m_MixFormat.nSamplesPerSec) / 1000;

	m_pScopedWaveData = std::make_shared<std::vector<int>>(m_WaveDataMax + 1);
	if (m_pScopedWaveData->size() < m_WaveDataMax + 1)
	{
		return E_OUTOFMEMORY;
	}

	// Only Support 16 bit Audio for now
	if (m_MixFormat.wBitsPerSample == 16)
	{
		m_pScopedWaveData->at(m_WaveDataMax) = -32768;  // INT16_MAX
	}
	else
	{
		m_pScopedWaveData = nullptr;
		hr = S_FALSE;
	}

	return hr;
}

HRESULT MicrophoneSimulateButton::OnSamples(BYTE* pData, UINT32 cbBytes)
{
	HRESULT hr = S_OK;

	// We don't have a valid pointer array, so return.  This could be the case if we aren't
	// dealing with 16-bit audio
	//if (m_PlotData.empty())
	//{
	//	return S_FALSE;
	//}

	DWORD dwNumPoints = cbBytes / m_MixFormat.nChannels / (m_MixFormat.wBitsPerSample / 8);

	// Read the 16-bit samples from channel 0
	INT16 *pi16 = (INT16*) pData;

	for (DWORD i = 0; m_WaveDataFilled < m_WaveDataMax && i < dwNumPoints; i++)
	{
		m_pScopedWaveData->at(m_WaveDataFilled) = *pi16;
		pi16 += m_MixFormat.nChannels;

		m_WaveDataFilled++;
	}

	// Send off the event and get ready for the next set of samples
	if (m_WaveDataFilled == m_WaveDataMax)
	{
		ComPtr<IUnknown> spUnknown;
		ComPtr<CAsyncState> spState = Make<CAsyncState>(m_pScopedWaveData, m_WaveDataMax + 1);

		hr = spState.As(&spUnknown);
		if (SUCCEEDED(hr))
		{
			MFPutWorkItem2(MFASYNC_CALLBACK_QUEUE_MULTITHREADED, 0, &m_xSendScopeData, spUnknown.Get());
		}

		m_WaveDataFilled = 0;
	}

	return hr;
}

HRESULT MicrophoneSimulateButton::OnSendScopeData(IMFAsyncResult* pResult)
{
	HRESULT hr = S_OK;
	ComPtr<CAsyncState> pState = nullptr;

	hr = pResult->GetState(reinterpret_cast<IUnknown**>(pState.GetAddressOf()));
	if (SUCCEEDED(hr))
	{
		const auto& data = *pState->Data;

		uint32_t sum = 0;
		for (auto frame : data)
		{
			sum += std::abs(frame);
		}

		sum /= pState->Size;

		ButtonState state = (sum > abs(data.back()) / 4) ? Pressed : Released;
		if (state != m_CurrentState)
		{
			auto e = std::make_shared<ButtonStateChangedEventArgs>();
			e->NewState = state;
			e->OldState = m_CurrentState;
			m_CurrentState = state;
			ButtonStateChanged(this, e);
		}
	}

	return S_OK;
}
