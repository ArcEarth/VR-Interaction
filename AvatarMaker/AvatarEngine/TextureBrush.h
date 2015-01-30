#pragma once
#include "Textures.h"
#include <memory>
#include "SkinMesh.h"

namespace EditingTools
{
namespace PaintTools 
{
	class IBrush{

	public:

		virtual void Start(){}
		virtual bool Draw(ID3D11DeviceContext* pContext , DirectX::RenderTargetTexture2D* Canvas , DirectX::XMVECTOR PositionWS, DirectX::FXMVECTOR Color) = 0;
		virtual void Finish(){}
		virtual ~IBrush() {}

		void setRadius(float Radius) { _radius = Radius; }
		float getRadius() const {return _radius; }

		__declspec(property(get = getRadius , put = setRadius))
			float Radius;

	protected:
		float _radius;
	};

	class PaintBucket
		: public IBrush
	{
	public:
		PaintBucket() {}
		virtual ~PaintBucket(){}
		virtual bool Draw(ID3D11DeviceContext* pContext , DirectX::RenderTargetTexture2D* Canvas , DirectX::XMVECTOR PositionWS, DirectX::FXMVECTOR Color);
	};

	class GaussBrush
		: public IBrush
	{
	public:
		GaussBrush(ID3D11Device* pDevice , DirectX::SkinMesh* m_pTarget  , unsigned int _Index , float Radius = 10);
		virtual ~GaussBrush();
		virtual void Start();
		virtual void Finish();
		virtual bool Draw(ID3D11DeviceContext* pContext , DirectX::RenderTargetTexture2D* Canvas , DirectX::XMVECTOR PositionWS, DirectX::FXMVECTOR Color);
	protected:
		class Impl;
		std::shared_ptr<Impl> m_pImpl;

		unsigned int Index;
	};

} // End namespace PaintTools
} // End namespace Editing Tools