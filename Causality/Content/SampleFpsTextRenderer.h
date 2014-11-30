#pragma once

#include <string>
#include "..\Common\DeviceResources.h"
#include "..\Common\StepTimer.h"
#include "..\Common\Renderable.h"
#include "..\Interactive.h"

namespace Causality
{
	// Renders the current FPS value in the bottom right corner of the screen using Direct2D and DirectWrite.
	class HUDInterface : public Platform::IAppComponent, public DirectX::Scene::IRenderable, public DirectX::Scene::ITimeAnimatable
	{
	public:
		HUDInterface(const std::shared_ptr<DirectX::DeviceResources>& deviceResources);
		void CreateDeviceDependentResources();
		void ReleaseDeviceDependentResources();
		// ITimeAnimatable
		void UpdateAnimation(DirectX::StepTimer const& timer) override;


		// Inherited via IRenderable
		virtual void Render(ID3D11DeviceContext * pContext) override;
		void Render(DirectX::DeviceResources *pDeviceResources);

	private:
		// Cached pointer to device resources.
		std::shared_ptr<DirectX::DeviceResources> m_deviceResources;

		// Resources related to text rendering.
		std::wstring                                    m_text;
		DWRITE_TEXT_METRICS	                            m_textMetrics;
		Microsoft::WRL::ComPtr<ID2D1SolidColorBrush>    m_whiteBrush;
		Microsoft::WRL::ComPtr<ID2D1DrawingStateBlock>  m_stateBlock;
		Microsoft::WRL::ComPtr<IDWriteTextLayout>       m_textLayout;
		Microsoft::WRL::ComPtr<IDWriteTextFormat>		m_textFormat;


	};
}