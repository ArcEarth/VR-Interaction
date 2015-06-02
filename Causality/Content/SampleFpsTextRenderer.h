#pragma once

#include <string>
#include "..\Common\DeviceResources.h"
#include "..\Common\StepTimer.h"
#include "..\Common\Renderable.h"
#include "..\Interactive.h"

namespace Causality
{
	// Renders the current FPS value in the bottom right corner of the screen using Direct2D and DirectWrite.
	class HUDInterface : public IAppComponent, public DirectX::Scene::IRenderable, public DirectX::Scene::ITimeAnimatable
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
		std::shared_ptr<DirectX::DeviceResources>		m_deviceResources;

		// Resources related to text rendering.
		std::wstring                                    m_text;
		DWRITE_TEXT_METRICS	                            m_textMetrics;
		Microsoft::WRL::ComPtr<ID2D1SolidColorBrush>    m_whiteBrush;
		Microsoft::WRL::ComPtr<ID2D1DrawingStateBlock>  m_stateBlock;
		Microsoft::WRL::ComPtr<IDWriteTextLayout>       m_textLayout;
		Microsoft::WRL::ComPtr<IDWriteTextFormat>		m_textFormat;
	};

	class TextBlock;

	class HUDElement
	{
	public:
		// for composition
		Vector2 Position;
		Vector2 Size;
		float	Opticity;
		
	private:

	};

	class Plot2D : public HUDElement
	{
		template <class Iter, class Iter>
		Plot2D(DirectX::Vector2 position,DirectX::Vector2 size,size_t n, Iter xItr, Iter yItr);

	private:

	};

	class TextBlockFactory
	{
	public:
		std::weak_ptr<TextBlock> CreateTimedTextBlock(const std::wstring& content, float X, float Y, double lifeTime, float fontSize = 48, DirectX::FXMVECTOR fontColor = DirectX::Colors::White, bool Fading = true, bool Shifting = false, const std::wstring fontFamily = L"Arial");
		std::shared_ptr<TextBlock> CreateFixedTextBlock(const std::wstring& content, float X = 0.0f, float Y = 0.0f, float fontSize = 48, DirectX::FXMVECTOR fontColor = DirectX::Colors::White, const std::wstring fontFamily = L"Arial");
		std::shared_ptr<TextBlock> CreateCustomizedTextBlock(const std::wstring& content, float X, float Y, std::function<void(TextBlock*)> CustomlizeFunction, float fontSize = 48, DirectX::FXMVECTOR fontColor = DirectX::Colors::White, const std::wstring fontFamily = L"Arial");

	private:
		std::list<std::weak_ptr<TextBlock>> m_TextBlockContainer;
	};

	class TextBlock : public std::enable_shared_from_this<TextBlock>
	{
		void CreateDeviceDependentResources();
		void ReleaseDeviceDependentResources();

		void UpdateBrushColor();

		const std::wstring& Text() const;
		void SetText(const std::wstring& text);

		const DirectX::Color& Foreground() const;
		void SetForeground(const DirectX::Color& color);

		float FontSize() const;
		void SetFontSize(float fontSize);

		const Vector2& Size() const;
		const Vector2& Position() const;

		void SetSize(const Vector2& size) const;
		void SetPosition(const Vector2& position) const;

		void UpdateLayout();
		void Render();

		virtual void Update(time_seconds const& time_delta);
	private:
		DirectX::Color									m_textColor;
		std::wstring                                    m_text;
		DWRITE_TEXT_METRICS	                            m_textMetrics;
		Microsoft::WRL::ComPtr<ID2D1SolidColorBrush>    m_Brush;
		Microsoft::WRL::ComPtr<ID2D1DrawingStateBlock>  m_stateBlock;
		Microsoft::WRL::ComPtr<IDWriteTextLayout>       m_textLayout;
		Microsoft::WRL::ComPtr<IDWriteTextFormat>		m_textFormat;
	};

	class TimedTextBlock : public TextBlock
	{
	};
}