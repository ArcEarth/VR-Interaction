#pragma once

#include <string>
#include <memory>
#include <limits>
#include <dwrite_3.h>
#include <d2d1_3.h>

namespace DirectX
{
	namespace Scene
	{
		class TextBlock;

		using Microsoft::WRL::ComPtr;

		enum HorizentalAlignmentEnum
		{
			Left,
			Right,
			Center,
			Stretch,
		};

		enum VerticalAlignmentEnum
		{
			Top,
			Bottom,
			Center,
			Stretch,
		};

		class Panel;

		const float AutoSize = std::numeric_limits<float>::signaling_NaN();

		XM_ALIGNATTR
		class HUDElement : public AlignedNew<HUDElement>
		{
		public:
			HUDElement();

			float Width() const;
			float Height() const;
			void  SetWidth(float width);
			void  SetHeight(float height);

			const Vector2& Size() const;
			const Vector2& Position() const;

			void SetSize(const Vector2& size);
			void SetPosition(const Vector2& position);

			float Opticity() const { return m_opticity; }
			float SetOpticity(float opticity);

			const HUDElement* Parent() const { return m_parent; }
			HUDElement* Parent() { return m_parent; }

			// Layout interface
			bool IsLayoutUptoDate() const;
			void InvaliadMeasure();
			Vector2 Measure(const Vector2& availableSize);
			Vector2 Arrange(const Vector2& actualSize);
			virtual Vector2 MeasureOverride(const Vector2& availableSize);
			virtual Vector2 ArrangeOverride(const Vector2& actualSize);
			virtual void UpdateLayout();

			// Rendering interface
			virtual void Render(ID2D1DeviceContext* context);

			// Allows Panel to change parent property
			friend Panel;
		protected:
			// for composition
			Vector2		            m_position;
			Vector2		            m_size;
			Vector2		            m_maxSize;
			bool					m_wAuto; // signal when width is set to auto
			bool					m_hAuto; // signal when height is set to auto
			float		            m_opticity;
			unsigned	            m_dirtyFlags;
			HorizentalAlignmentEnum m_hAlign;
			VerticalAlignmentEnum	m_vAlign;
			D2D1_MATRIX_3X2_F		m_transform; // relative transform to it's parent
			HUDElement*				m_parent;
		};

		class I2dDeviceResource abstract
		{
		public:
			// Factory resources is actually device free
			virtual void CreateDeviceResources(ID2D1Factory* pD2dFactory, IDWriteFactory* pDwriteFactory) = 0;
			virtual void ReleaseDeviceResources() = 0;
		};

		class Panel : public HUDElement
		{
		public:
			typedef std::vector<HUDElement*> ElementListType;

			const ElementListType& Children() const { return m_children; }

			virtual void AddChild(HUDElement* elem);

		protected:
			ElementListType m_children;
		};

		// Root element of hud tree, nothing but to provide the logical size of the window
		class HUDCanvas : public Panel
		{
		public:
			virtual void UpdateLayout();
			virtual void Render(ID2D1DeviceContext* context);

			void SetTarget(ID2D1Bitmap* bitmap);

		protected:
			ComPtr<ID2D1Bitmap> m_canvasBitmap;
		};

		class StackPanel : public Panel
		{

		};

		class ScatterChart : public HUDElement, public I2dDeviceResource
		{
		public:
			ScatterChart(size_t n, float* xItr, float* yItr);

			enum LineInterpolationTypeEnum
			{
				Point,
				Linear,
				Cubic,
			};

			enum PointLabelTypeEnum
			{
				NoPoint,
				Circle,
			};

			void SetDataPointLabel();
			void SetLineInterpolationType(LineInterpolationTypeEnum type);
			void SetXRange(float min, float max);
			void SetYRange(float min, float max);
			void SetFrameGridType();
			void SetChartTitle(const std::wstring& title);
			void SetSequenceColor(const Color& color);
			void SetLineWidth(float width);
			void SetBackground(const Color& color);
			void SetForeground(const Color& color);

			void CreateDeviceResources(ID2D1Factory* pD2dFactory, IDWriteFactory* pDwriteFactory) override;
			void ReleaseDeviceResources() override;
		private:

			std::vector<Vector2, AlignedAllocator<XMVECTOR>> m_rawData;
		};

		class TextBlock : public HUDElement, public I2dDeviceResource
		{
			TextBlock();
			TextBlock(const std::wstring& text, ID2D1Factory* pD2dFactory = nullptr, IDWriteFactory* pDwriteFactory = nullptr);

			void CreateDeviceResources(ID2D1Factory* pD2dFactory, IDWriteFactory* pDwriteFactory) override;
			void ReleaseDeviceResources() override;

			const std::wstring& Text() const;
			void SetText(const std::wstring& text);

			const Color& Foreground() const;
			void SetForeground(const Color& color);

			float FontSize() const;
			void  SetFontSize(float fontSize);

			virtual void Render(ID2D1DeviceContext* context) override;

		private:
			void UpdateBrushColor(ID2D1DeviceContext* pContext);
			// Create New text layout when text is changed
			void UpdateTextLayout(IDWriteFactory* pFactory = nullptr);

		private:
			std::wstring                    m_text;
			Color							m_textColor;
			float							m_fontSize;

			DWRITE_TEXT_METRICS	            m_textMetrics;

			ComPtr<IDWriteFactory>			m_dwFactory;
			ComPtr<ID2D1SolidColorBrush>    m_Brush;
			ComPtr<ID2D1DrawingStateBlock>  m_stateBlock;
			ComPtr<IDWriteTextLayout>       m_textLayout;
			ComPtr<IDWriteTextFormat>		m_textFormat;
		};

		class TimedTextBlock : public TextBlock
		{
		};
	}
}