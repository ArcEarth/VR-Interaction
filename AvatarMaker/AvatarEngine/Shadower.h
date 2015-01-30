#pragma once
#include "stdafx.h"
#include "Textures.h"
#include <Effects.h>
#include "ModelInterface.h"

namespace DirectX
{
	class Shadower
	{
	public:
		Shadower(ID3D11Device* pDevice);
		~Shadower(void);

		void DroppingShadow(IRenderObject *pObject);

	private:
		std::unique_ptr<DirectX::IEffect> m_pEffect;
		std::unique_ptr<DirectX::RenderTargetTexture2D> m_pShadowMap;
	};
}

