#include "Foregrounds.h"

using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;
using namespace std;


Causality::Foregrounds::Foregrounds(const std::shared_ptr<DirectX::DeviceResources>& pResouce)
{
	pEffect = pResouce->GetBasicEffect();
}

Foregrounds::~Foregrounds()
{
}

void Causality::Foregrounds::LoadAsync()
{
	m_loadingComplete = false;

}

void Causality::Foregrounds::Render(ID3D11DeviceContext * pContext)
{
	for (auto&& pObj : Objects)
	{
		pEffect->SetWorld(pObj->GetModelMatrix());
		pEffect->Apply(pContext);
		pObj->Draw(pContext);
	}

}

void XM_CALLCONV Causality::Foregrounds::UpdateViewMatrix(DirectX::FXMMATRIX view)
{
	pEffect->SetView(view);
}

void XM_CALLCONV Causality::Foregrounds::UpdateProjectionMatrix(DirectX::FXMMATRIX projection)
{
	pEffect->SetProjection(projection);
}
