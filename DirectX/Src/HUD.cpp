#include "pch_directX.h"
#include "HUD.h"
#include "DirectXHelper.h"

using namespace DirectX;
using namespace DirectX::Scene;
using namespace D2D1;

wchar_t g_defaultFontFamilayName[] = L"Segoe UI";
wchar_t g_defaultLocaleName[] = L"en-US";

namespace DirtyFlags
{
	unsigned Position = 0x1;
	unsigned Size = 0x2;
	unsigned Alignment = 0x4;
	unsigned Opticity = 0x8;
	unsigned TextColor = 0x10;
	unsigned TextContent = 0x20;
	unsigned FontSize = 0x40;
}

// Updates the text to be displayed.
//void HUDInterface::UpdateAnimation(DirectX::StepTimer const& timer)
//{
//	m_text = (fps > 0) ? std::to_wstring(fps) + L" FPS" : L" - FPS";
//}

TextBlock::TextBlock()
{
}

TextBlock::TextBlock(const std::wstring & text, ID2D1Factory * pD2dFactory, IDWriteFactory * pDwriteFactory)
{
}

void TextBlock::CreateDeviceResources(ID2D1Factory* pD2dFactory, IDWriteFactory * pFactory)
{
	DirectX::ThrowIfFailed(
		pFactory->CreateTextFormat(
			g_defaultFontFamilayName,
			nullptr,
			DWRITE_FONT_WEIGHT_LIGHT,
			DWRITE_FONT_STYLE_NORMAL,
			DWRITE_FONT_STRETCH_NORMAL,
			m_fontSize,
			g_defaultLocaleName,
			&m_textFormat
			)
		);

	ThrowIfFailed(
		m_textFormat->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_TRAILING)
		);

	DirectX::ThrowIfFailed(
		m_textFormat->SetParagraphAlignment(DWRITE_PARAGRAPH_ALIGNMENT_NEAR)
		);

	DirectX::ThrowIfFailed(
		pD2dFactory->CreateDrawingStateBlock(&m_stateBlock)
		);

	if (!m_text.empty())
		UpdateTextLayout(pFactory);
}

void TextBlock::ReleaseDeviceResources()
{
	m_Brush.Reset();
	m_stateBlock.Reset();
	m_textLayout.Reset();
	m_textFormat.Reset();
}

const std::wstring & TextBlock::Text() const { return m_text; }

void TextBlock::SetText(const std::wstring & text) { m_text = text; m_dirtyFlags |= DirtyFlags::TextContent; }

const Color & TextBlock::Foreground() const { return m_textColor; }

void TextBlock::SetForeground(const Color & color) { m_textColor = color; m_dirtyFlags |= DirtyFlags::TextColor; }

float TextBlock::FontSize() const { return m_fontSize; }

void TextBlock::SetFontSize(float fontSize) { m_fontSize = fontSize; m_dirtyFlags |= DirtyFlags::FontSize; }

void TextBlock::UpdateBrushColor(ID2D1DeviceContext* pContext)
{
	const auto& color =
		reinterpret_cast<const D2D1_COLOR_F &>(m_textColor);
	if (m_Brush)
	{
		if (m_dirtyFlags & DirtyFlags::TextColor)
		{
			m_dirtyFlags &= ~DirtyFlags::TextColor;
			m_Brush->SetColor(color);
		}

		if (m_dirtyFlags & DirtyFlags::Opticity)
		{
			m_dirtyFlags &= ~DirtyFlags::Opticity;
			m_Brush->SetOpacity(m_opticity);
		}
	}
	else
	{
		m_dirtyFlags &= ~DirtyFlags::TextColor;
		m_dirtyFlags &= ~DirtyFlags::Opticity;
		pContext->CreateSolidColorBrush(color, &m_Brush);
		m_Brush->SetOpacity(m_opticity);
	}
}

void TextBlock::UpdateTextLayout(IDWriteFactory* pFactory)
{
	if (pFactory == nullptr)
		pFactory = m_dwFactory.Get();

	if (pFactory == nullptr)
		return;

	if (m_textLayout == nullptr || m_dirtyFlags & DirtyFlags::TextContent)
	{
		m_dirtyFlags &= ~DirtyFlags::TextContent;
		DirectX::ThrowIfFailed(
			pFactory->CreateTextLayout(
				m_text.c_str(),
				(uint32_t)m_text.length(),
				m_textFormat.Get(),
				m_size.x, // Max width of the input text.
				m_size.y, // Max height of the input text.
				&m_textLayout
				)
			);
	}
	else if (m_dirtyFlags & DirtyFlags::Size)
	{
		m_dirtyFlags &= ~DirtyFlags::Size;
		m_textLayout->SetMaxHeight(m_size.y);
		m_textLayout->SetMaxWidth(m_size.x);
	}
	else if (m_dirtyFlags & DirtyFlags::FontSize)
	{
		m_dirtyFlags &= ~DirtyFlags::FontSize;
		m_textLayout->SetFontSize(m_fontSize, DWRITE_TEXT_RANGE{ 0, m_text.size() });
	}

	DirectX::ThrowIfFailed(
		m_textLayout->GetMetrics(&m_textMetrics)
		);
}


void TextBlock::Render(ID2D1DeviceContext* context)
{
	if (m_text.empty()) return;

	UpdateLayout();
	UpdateTextLayout();
	UpdateBrushColor(context);

	if (!m_textLayout)
		throw std::runtime_error("not intialized");

	context->SaveDrawingState(m_stateBlock.Get());
	context->BeginDraw();

	Matrix3x2F parentTransform;
	context->GetTransform(&parentTransform);

	context->SetTransform(m_transform * parentTransform);

	context->DrawTextLayout(
		D2D1::Point2F(0.f, 0.f),
		m_textLayout.Get(),
		m_Brush.Get()
		);

	// Ignore D2DERR_RECREATE_TARGET here. This error indicates that the device
	// is lost. It will be handled during the next call to Present.
	HRESULT hr = context->EndDraw();
	if (hr != D2DERR_RECREATE_TARGET)
	{
		ThrowIfFailed(hr);
	}

	context->RestoreDrawingState(m_stateBlock.Get());
}

Vector2 HUDElement::MeasureOverride(const Vector2 & availableSize)
{
}

Vector2 HUDElement::ArrangeOverride(const Vector2 & actualSize)
{
}

void HUDElement::Render(ID2D1DeviceContext * context)
{
}

HUDElement::HUDElement()
	: m_parent(nullptr),
	m_transform(D2D1::Matrix3x2F::Identity()),
	m_hAlign(Left), m_vAlign(Top),
	m_dirtyFlags(-1),
	m_opticity(1.0f),
	m_wAuto(true), m_hAuto(true),
	m_position(), m_size()
{
}

float HUDElement::Width() const { return m_size.x; }

float HUDElement::Height() const { return m_size.y; }

void HUDElement::SetWidth(float width) {
	m_size.x = width;
	m_dirtyFlags |= DirtyFlags::Size;
}

void HUDElement::SetHeight(float height) {
	m_size.y = height;
	m_dirtyFlags |= DirtyFlags::Size;
}

const Vector2 & HUDElement::Size() const {
	return m_size;
}

const Vector2 & HUDElement::Position() const { return m_position; }

void HUDElement::SetSize(const Vector2 & size) {
	m_size = size;
	m_dirtyFlags |= DirtyFlags::Size;
}

void HUDElement::SetPosition(const Vector2 & position) {
	m_position = position;
	m_dirtyFlags |= DirtyFlags::Position;
}

void HUDElement::UpdateLayout()
{
	auto logicalSize = Parent()->Size();

	float tx = 0, ty = 0, sx = 1, sy = 1;

	switch (m_hAlign)
	{
	case Right:
		tx = logicalSize.x - m_size.x;
		break;
	case Center:
		tx = (logicalSize.x - m_size.x) * 0.5f;
		break;
	case Stretch:
	case Left:
	default:
		break;
	}

	switch (m_vAlign)
	{
	case Bottom:
		tx = logicalSize.y - m_size.y;
		break;
	case Center:
		tx = (logicalSize.y - m_size.y) * 0.5f;
		break;
	case Stretch:
	case Top:
	default:
		break;
	}

	m_transform = Matrix3x2F::Translation(
		tx,
		ty
		);

}

float HUDElement::SetOpticity(float opticity) {
	m_opticity = opticity;
	m_dirtyFlags |= DirtyFlags::Opticity;
}

bool HUDElement::IsLayoutUptoDate() const { return !m_dirtyFlags; }

void HUDElement::InvaliadMeasure()
{
	m_dirtyFlags = DirtyFlags::Size & DirtyFlags::Alignment & DirtyFlags::Position & DirtyFlags::Opticity;
}

Vector2 HUDElement::Measure(const Vector2 & availableSize)
{
	return MeasureOverride(availableSize);
}

Vector2 HUDElement::Arrange(const Vector2 & actualSize)
{
	return ArrangeOverride(actualSize);
}

void HUDCanvas::UpdateLayout()
{
}

void HUDCanvas::Render(ID2D1DeviceContext * context)
{
	context->SetTarget(m_renderTarget.Get());
}

void Panel::AddChild(HUDElement * elem)
{
	m_children.push_back(elem);
	elem->m_parent = this;
}
