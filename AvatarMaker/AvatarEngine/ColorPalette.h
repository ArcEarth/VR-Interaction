#ifndef COLOR_PALETTE_H
#define COLOR_PALETTE_H
#pragma once
#include "MathHelper.h"
#include <DirectXCollision.h>
#include <DirectXColors.h>
#include "ModelInterface.h"
#include "Carmera.h"

namespace EditingTools
{
namespace PaintTools 
{
	class ColorPalette
		: public DirectX::IRenderObject
	{
	public:

		ColorPalette(const ICamera* pCamera);
		ColorPalette(ID3D11Device* pDevice , const ICamera* pCamera);
		~ColorPalette(void);

		DirectX::XMVECTOR GetColor(DirectX::FXMVECTOR Point) const;
		bool SetColor(DirectX::FXMVECTOR Point);

		//const DirectX::BoundingBox* PigmentBox(unsigned index) const;
		DirectX::XMVECTOR GetColor(unsigned index) const;
		bool SetColor(unsigned int index);

		virtual void Render(ID3D11DeviceContext *pContext);

	private:
		void Initialize();

		DirectX::BoundingBox	m_PigmentBoxes[6];
		DirectX::Vector4		m_Colors[6];
		const ICamera*			m_pCamera;
	};
}
}

#endif