#ifndef PAINT_MANAGER_H
#define PAINT_MANAGER_H
#pragma once
#include <vector>
#include "TIMER.h"
#include "BitMap.h"
#include "ColorPalette.h"
#include "EditingToolsInterface.h"
#include "SkinMesh.h"
#include "SpaceWarpperInterface.h"
#include "Lights.h"
#include "TextureBrush.h"
#include "DynamicMetaBallModel.h"

namespace EditingTools
{
	namespace PaintTools 
	{
		class GeometricObject;

		static const unsigned int Special_Pixel_Ratio = 1000;

		//class IBrush{

		//public:

		//	virtual bool Draw(DirectX::Bitmap* Canvas , DirectX::XMFLOAT2 Coordinate , DirectX::FXMVECTOR Color) = 0;
		//	virtual ~IBrush() {}

		//	void SetRadius(unsigned int PixelRadius) { _radius = PixelRadius; }
		//	unsigned int GetRadius() const {return _radius; }

		//	__declspec(property(get = GetRadius , put = SetRadius))
		//		unsigned int Radius;

		//protected:
		//	unsigned int _radius;
		//};

		//class PaintBucket
		//	: public IBrush
		//{
		//public:
		//	PaintBucket() {}
		//	virtual ~PaintBucket(){}
		//	virtual bool Draw(DirectX::Bitmap* Canvas , DirectX::XMFLOAT2 Coordinate , DirectX::FXMVECTOR Color);
		//};

		//class GaussBrush
		//	: public IBrush
		//{
		//public:
		//	GaussBrush(unsigned int Radius = 10);
		//	virtual ~GaussBrush(){}
		//	virtual bool Draw(DirectX::Bitmap* Canvas , DirectX::XMFLOAT2 Coordinate , DirectX::FXMVECTOR Color);
		//};

		class IColorSource
		{
		public:
			virtual DirectX::XMVECTOR GetColor() const = 0;
			virtual void SetColor(DirectX::FXMVECTOR Color) = 0;
		};

		class ColorHoverPicker
			: public IColorSource
		{
		public:
			ColorHoverPicker(DirectX::FXMVECTOR InitialColor = DirectX::Colors::LawnGreen);
			~ColorHoverPicker() {}

			virtual DirectX::XMVECTOR GetColor() const 
			{ return m_PaintColor;}

			virtual void SetColor(DirectX::FXMVECTOR Color);

			// hover-style update color
			bool HoverColor(DirectX::FXMVECTOR Color);
			bool HoverPick(DirectX::FXMVECTOR Position);
			// Force update color
		protected:
			DirectX::Vector4 m_PaintColor;
			// Data for hover selection
			DirectX::Vector4 m_UpdatingColor;
			TIMER m_Timer;

		protected:
			static ColorPalette* s_pPalette;
			static const unsigned int Hover_Select_Time = 500; //million sec
		public:
			static std::unique_ptr<ColorPalette> InitializeColorPalette(ID3D11Device* pDevice,const ICamera* pCamera);
//			static void ReleaseColorPalette();
			static ColorPalette* GetColorPalette();
		};
		
		class PaintToolBase
			: public EditingTools::IEditingTool
		{
		public:
			PaintToolBase(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera , SphereLightData* pCursorLight);
			virtual ~PaintToolBase();
			virtual void Start();
			virtual void Finish();
			virtual void Abort();
			// the data should be HAND , WRIST , ELBOW , SHOULDER since we only have arm to use as a tool
			virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);

			virtual bool IsActive() const;

			// The generic description about this tool's affection
			virtual float Radius() const;
			virtual bool SetRadius(float Radius);
			virtual bool TurnRadius(float deltaRadius);

			// the Interface for rendering
			virtual DirectX::IRenderObject* Visualization();
			//virtual void Edit(const HolographicPoint& Ptr , bool Snaped) = 0;
			virtual void Edit(DirectX::FXMVECTOR vPos , bool Snaped) = 0;
		protected:

			//DirectX::ITextureObject*			m_pTextureMapper;
			std::shared_ptr<IColorSource>						m_pSource;
			std::shared_ptr<Geometrics::DynamicMetaBallModel>	m_pSubject;
			//IWarpper*							m_pWarpper;
			SphereLightData*					m_pCursorLight;
			const ICamera*						m_pCamera;
			std::unique_ptr<PaintTools::GeometricObject> m_pVisual;

			bool m_Snaped;
			float m_radius;
			bool m_working;
			bool MandatorySnaped;

		//public:
		//	static DirectX::XMFLOAT2 UVMapping(const DirectX::Vector3& p);
		};

		class BrushTool
			: public PaintToolBase
		{
		public:
			BrushTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera , SphereLightData* pCursorLight);
			virtual void Start();
			virtual void Finish();
			virtual void Abort();
			
			void SetBrush(ID3D11DeviceContext* pDevice,std::unique_ptr<PaintTools::IBrush> &&pBrush)
			{
				m_pContext = pDevice;
				m_pBrush = std::move(pBrush);
				m_pBrush->Radius  = Radius();
			}

			//virtual void Edit(const HolographicPoint& Ptr , bool Snaped);
			virtual void Edit(DirectX::FXMVECTOR vPos , bool Snaped);

			virtual bool SetRadius(float Radius);

		protected:
			std::unique_ptr<PaintTools::IBrush> m_pBrush;
			ID3D11DeviceContext*				m_pContext;
			//std::unique_ptr<DirectX::Bitmap> m_pTexBackup;
		};

		class EyedropperTool
			: public PaintToolBase
		{
		public:
			EyedropperTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera ,const std::shared_ptr<ColorPalette>& pPalette);
			virtual void Start();
			virtual void Abort();

			//virtual void Edit(const HolographicPoint& Ptr , bool Snaped);
			virtual void Edit(DirectX::FXMVECTOR vPos , bool Snaped);
			//std::unique_ptr<EyeDropper> m_pDropper;
		protected:
			std::shared_ptr<ColorPalette>		m_pColorPallete;
			DirectX::Vector4					m_ColorBackup;
		};

		class HoverCursorTool
			: public PaintToolBase
		{
		public:
			HoverCursorTool(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pSubject , const std::shared_ptr<IColorSource> &pSource , ID3D11DeviceContext* pContext ,const ICamera* pCamera , SphereLightData* pCursorLight,const std::shared_ptr<ColorHoverPicker> &pColorHoverPicker);

			std::shared_ptr<IColorSource> GetColorSource() const;
			//virtual void Edit(const HolographicPoint& Ptr , bool Snaped);
			virtual void Edit(DirectX::FXMVECTOR vPos , bool Snaped);
			
		protected:
			std::shared_ptr<ColorHoverPicker> m_pHoverPicker;
		};
	}

	// This is a ....errr special tool
	//class PaintManager
	//	: public Geometrics::EditingTools::IEditingTool
	//{
	//public:
	//	enum WorkState
	//	{
	//		Pause,
	//		Cursor,
	//		Eyedropper,
	//		PaintBrush,
	//		PaintBucket
	//	};

	//public:
	//	PaintManager(DirectX::TextureSkinMesh* pSubject , ID3D11DeviceContext* pContext ,const ICamera* pCamera);
	//	~PaintManager();
	//	virtual void Start();
	//	virtual void Finish();
	//	virtual void Abort();
	//	// the data should be HAND , WRIST , ELBOW , SHOULDER since we only have arm to use as a tool
	//	virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);

	//	virtual bool IsActive() const;

	//	// The generic description about this tool's affection
	//	virtual float Radius() const;
	//	virtual bool SetRadius(float Radius);
	//	virtual bool TurnRadius(float deltaRadius);

	//	// the Interface for rendering
	//	virtual DirectX::IRenderObject* Visualization();

	//	WorkState GetState() const;
	//	void SetState(WorkState newState);

	//protected:
	//	WorkState State;

	//	//PaintTools::EyeDropper m_dropper;
	//	//std::unique_ptr<PaintTools::IBrush> m_pBrush;
	//	//std::unique_ptr<PaintTools::IBrush> m_pBucket;
	//	DirectX::TextureSkinMesh* m_pSubject;
	//	// for Abort the editing 
	//	std::unique_ptr<DirectX::Bitmap> m_pTexBackup;
	//	IWarpper *m_pWarpper;
	//	std::unique_ptr<PaintTools::GeometricObject> m_pVisual;

	//	float m_radius;
	//	bool m_working;

	//public:
	//	static DirectX::XMFLOAT2 UVMapping(const DirectX::Vector3& p);
	//	static std::unique_ptr<ColorPalette> s_pPalette;
	//};

}




#endif