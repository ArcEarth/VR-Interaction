#include "Foregrounds.h"
#include "CausalityApplication.h"

using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;
using namespace std;


wstring models [] =
{
	L"baseball_bat.obj",
	L"ConfederatePistol.obj",
	L"NBA_BASKETBALL.obj"
};

Causality::Foregrounds::Foregrounds(const std::shared_ptr<DirectX::DeviceResources>& pResouce)
{
	LoadAsync(pResouce->GetD3DDevice());
}

Foregrounds::~Foregrounds()
{
}

void Causality::Foregrounds::LoadAsync(ID3D11Device* pDevice)
{
	auto Directory = App::Current()->GetResourcesDirectory();
	auto ModelDirectory = Directory / "Models";
	auto TextureDirectory = Directory / "Textures";
	auto texDir = TextureDirectory.wstring();

	m_loadingComplete = false;
	concurrency::task<void> load_models([this,pDevice,texDir]() {
		pEffect = std::make_shared<BasicEffect>();
		pEffect->SetVertexColorEnabled(false);
		pEffect->SetTextureEnabled(true);
		pEffect->SetLightingEnabled(true);
		Objects.push_back(make_shared<ObjModel>(pDevice, models[0], texDir));
		Objects.push_back(make_shared<ObjModel>(pDevice, models[1], texDir));
		Objects.push_back(make_shared<ObjModel>(pDevice, models[2], texDir));
		m_loadingComplete = true;
	});
}

void Causality::Foregrounds::Render(ID3D11DeviceContext * pContext)
{
	for (auto&& pObj : Objects)
	{
		pEffect->SetWorld(pObj->GetModelMatrix());
		pEffect->SetTexture(pObj->DisplaceMap.Get());
		pEffect->SetDiffuseColor(pObj->DiffuseColor);
		pEffect->SetSpecularColor(pObj->SpecularColor);
		pEffect->SetAlpha(pObj->Alpha);
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

void Causality::Foregrounds::UpdateAnimation(StepTimer const & timer)
{
}
