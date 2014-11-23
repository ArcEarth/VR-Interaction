#pragma once
#include "Common\Model.h"
#include "Common\DeviceResources.h"
#include <Effects.h>
namespace Causality
{
	class Foregrounds : public DirectX::Scene::IRenderable , public DirectX::Scene::IViewable
	{
	public:
		Foregrounds(const std::shared_ptr<DirectX::DeviceResources> &pResouce);
		~Foregrounds();

		void LoadAsync();

		std::shared_ptr<DirectX::BasicEffect> pEffect;
		std::vector<std::shared_ptr<DirectX::Scene::ObjModel>> Objects;

		// Inherited via IRenderable
		virtual void Render(ID3D11DeviceContext * pContext) override;

		// Inherited via IViewable
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view) override;
		virtual void XM_CALLCONV UpdateProjectionMatrix(DirectX::FXMMATRIX projection) override;

		bool m_loadingComplete;
	};
}
