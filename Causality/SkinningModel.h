#pragma once
#include "Common\Model.h"

namespace Causality
{
	struct SkinMeshData;

	class SkinAnimateModel :
		public DirectX::Scene::MonolithModel
	{
	public:
		typedef SkinAnimateModel *Ref;
		static Ref CreateFromSkinMeshData(ID3D11Device* pDevice, SkinMeshData* pData);

		SkinAnimateModel();
		~SkinAnimateModel();
	};

}