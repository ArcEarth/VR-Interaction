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

	m_loadingComplete = false;
	concurrency::task<void> load_models([this, pDevice]() {
		auto Directory = App::Current()->GetResourcesDirectory();
		auto ModelDirectory = Directory / "Models";
		auto TextureDirectory = Directory / "Textures";
		auto texDir = TextureDirectory.wstring();
		pEffect = std::make_shared<BasicEffect>(pDevice);
		pEffect->SetVertexColorEnabled(false);
		pEffect->SetTextureEnabled(true);
		pEffect->SetLightingEnabled(true);
		{
			void const* shaderByteCode;
			size_t byteCodeLength;
			pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
			pInputLayout = CreateInputLayout<VertexPositionNormalTexture>(pDevice, shaderByteCode, byteCodeLength);
		}
		this->Position = { 0,0,-5 };
		this->Scale = { 0.1f,0.1f,0.1f };
		AddModel(GeometryModel::CreateFromObjFile(pDevice, (ModelDirectory / models[0]).wstring(), texDir));
		AddModel(GeometryModel::CreateFromObjFile(pDevice, (ModelDirectory / models[1]).wstring(), texDir));
		AddModel(GeometryModel::CreateFromObjFile(pDevice, (ModelDirectory / models[2]).wstring(), texDir));
		m_loadingComplete = true;
	});
}

void Causality::Foregrounds::Render(ID3D11DeviceContext * pContext)
{
	pContext->IASetInputLayout(pInputLayout.Get());
	ModelCollection::Render(pContext, pEffect.get());
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
