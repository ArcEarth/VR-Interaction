#include "../pch.h"
#include "SampleFpsTextRenderer.h"
#include "../Common/DirectXHelper.h"
#include "../Common/Textures.h"

using namespace Causality;
using namespace DirectX;

// Initializes D2D resources used for text rendering.
HUDInterface::HUDInterface(const std::shared_ptr<DirectX::DeviceResources>& deviceResources) : 
	m_text(L""),
	m_deviceResources(deviceResources)
{
	ZeroMemory(&m_textMetrics, sizeof(DWRITE_TEXT_METRICS));

	////DirectX::Texture2D tex(deviceResources->GetD3DDevice(), 800, 600);
	//DirectX::RenderTargetTexture2D tex(deviceResources->GetD3DDevice(), 800, 600,DXGI_FORMAT_R8G8B8A8_UNORM,4,1);
	////DirectX::Texture2D tex(deviceResources->GetD3DDevice(), 800, 600,1,DXGI_FORMAT_R8G8B8A8_UNORM,D3D11_USAGE_DEFAULT,D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE,0,0, 4,1);

	//auto ptex = tex.Resource();
	//ComPtr<IDXGISurface> pDxgiSurface = NULL;
	//ptex->QueryInterface<IDXGISurface>(&pDxgiSurface);
	//D2D1_RENDER_TARGET_PROPERTIES props =
	//	D2D1::RenderTargetProperties(
	//	D2D1_RENDER_TARGET_TYPE_DEFAULT,
	//	D2D1::PixelFormat(DXGI_FORMAT_UNKNOWN, D2D1_ALPHA_MODE_PREMULTIPLIED),
	//	96,
	//	96
	//	);

	//auto pD2DFactory = deviceResources->GetD2DFactory();
	//ComPtr<ID2D1RenderTarget> pRenderTarget;
	//pD2DFactory->CreateDxgiSurfaceRenderTarget(
	//	pDxgiSurface.Get(),
	//	&props,
	//	&pRenderTarget
	//	);

	// Create device independent resources
	DirectX::ThrowIfFailed(
		m_deviceResources->GetDWriteFactory()->CreateTextFormat(
			L"Segoe UI",
			nullptr,
			DWRITE_FONT_WEIGHT_LIGHT,
			DWRITE_FONT_STYLE_NORMAL,
			DWRITE_FONT_STRETCH_NORMAL,
			32.0f,
			L"en-US",
			&m_textFormat
			)
		);

	DirectX::ThrowIfFailed(
		m_textFormat->SetParagraphAlignment(DWRITE_PARAGRAPH_ALIGNMENT_NEAR)
		);
	DirectX::ThrowIfFailed(
		m_deviceResources->GetD2DFactory()->CreateDrawingStateBlock(&m_stateBlock)
		);

	CreateDeviceDependentResources();
}

// Updates the text to be displayed.
void HUDInterface::UpdateAnimation(DirectX::StepTimer const& timer)
{
	// Update display text.
	uint32 fps = timer.GetFramesPerSecond();

	m_text = (fps > 0) ? std::to_wstring(fps) + L" FPS" : L" - FPS";

	DirectX::ThrowIfFailed(
		m_deviceResources->GetDWriteFactory()->CreateTextLayout(
			m_text.c_str(),
			(uint32) m_text.length(),
			m_textFormat.Get(),
			240.0f, // Max width of the input text.
			50.0f, // Max height of the input text.
			&m_textLayout
			)
		);

	DirectX::ThrowIfFailed(
		m_textLayout->GetMetrics(&m_textMetrics)
		);
}

// Renders a frame to the screen.
void HUDInterface::Render(DirectX::DeviceResources *pDeviceResources)
{
	ID2D1DeviceContext* context = pDeviceResources->GetD2DDeviceContext();
	Windows::Foundation::Size logicalSize = pDeviceResources->GetLogicalSize();

	context->SaveDrawingState(m_stateBlock.Get());
	context->BeginDraw();

	// Position on the bottom right corner
	D2D1::Matrix3x2F screenTranslation = D2D1::Matrix3x2F::Translation(
		logicalSize.Width - m_textMetrics.layoutWidth,
		logicalSize.Height - m_textMetrics.height
		);

	context->SetTransform(screenTranslation * m_deviceResources->GetOrientationTransform2D());

	DirectX::ThrowIfFailed(
		m_textFormat->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_TRAILING)
		);

	context->DrawTextLayout(
		D2D1::Point2F(0.f, 0.f),
		m_textLayout.Get(),
		m_whiteBrush.Get()
		);

	// Ignore D2DERR_RECREATE_TARGET here. This error indicates that the device
	// is lost. It will be handled during the next call to Present.
	HRESULT hr = context->EndDraw();
	if (hr != D2DERR_RECREATE_TARGET)
	{
		DirectX::ThrowIfFailed(hr);
	}

	context->RestoreDrawingState(m_stateBlock.Get());
}

void Causality::HUDInterface::Render(ID3D11DeviceContext * pContext)
{
	Render(m_deviceResources.get());
}

void HUDInterface::CreateDeviceDependentResources()
{
	DirectX::ThrowIfFailed(
		m_deviceResources->GetD2DDeviceContext()->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::White), &m_whiteBrush)
		);
}

void HUDInterface::ReleaseDeviceDependentResources()
{
	m_whiteBrush.Reset();
}

void Causality::TextBlock::UpdateLayout()
{
}

void Causality::TextBlock::Render()
{
}
