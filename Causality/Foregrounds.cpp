#include "Foregrounds.h"
#include "CausalityApplication.h"
#include <tinyxml2.h>
#include <sstream>
using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;
using namespace std;


wstring models [] =
{
	//L"baseball_bat.obj",
	L"sibenik_n.obj",
	//L"ConfederatePistol.obj",
	//L"NBA_BASKETBALL.obj"
};

Causality::Foregrounds::Foregrounds(const std::shared_ptr<DirectX::DeviceResources>& pResouce)
	: States(pResouce->GetD3DDevice())
{
	LoadAsync(pResouce->GetD3DDevice());
}

Foregrounds::~Foregrounds()
{
}

void Causality::Foregrounds::LoadAsync(ID3D11Device* pDevice)
{

	m_loadingComplete = false;
	//CD3D11_DEFAULT d;
	//CD3D11_RASTERIZER_DESC Desc(d);
	//Desc.MultisampleEnable = TRUE;
	//ThrowIfFailed(pDevice->CreateRasterizerState(&Desc, &pRSState));

	concurrency::task<void> load_models([this, pDevice]() {
		auto Directory = App::Current()->GetResourcesDirectory();
		auto ModelDirectory = Directory / "Models";
		auto TextureDirectory = Directory / "Textures";
		auto texDir = TextureDirectory.wstring();
		pEffect = std::make_shared<BasicEffect>(pDevice);
		pEffect->SetVertexColorEnabled(false);
		pEffect->SetTextureEnabled(true);
		//pEffect->SetLightingEnabled(true);
		pEffect->EnableDefaultLighting();
		{
			void const* shaderByteCode;
			size_t byteCodeLength;
			pEffect->GetVertexShaderBytecode(&shaderByteCode, &byteCodeLength);
			pInputLayout = CreateInputLayout<VertexPositionNormalTexture>(pDevice, shaderByteCode, byteCodeLength);
		}
		this->Position = { 0,0,-5 };
		this->Scale = { 0.1f,0.1f,0.1f };

		auto sceneFile = Directory / "Foregrounds.xml";
		tinyxml2::XMLDocument sceneDoc;
		sceneDoc.LoadFile(sceneFile.string().c_str());
		auto scene = sceneDoc.FirstChild();
		auto obj = scene->FirstChildElement("obj");
		while (obj)
		{
			auto path = obj->Attribute("src");
			if (path != nullptr && strlen(path) != 0)
			{
				auto model = GeometryModel::CreateFromObjFile(pDevice, (ModelDirectory / path).wstring(), texDir);
				auto attr = obj->Attribute("position");
				if (attr != nullptr)
				{
					stringstream ss(attr);
					Vector3 vec;
					char ch;
					ss >> vec.x >> ch >> vec.y >> ch >> vec.z;
					model->SetPosition(vec);
				}
				attr = obj->Attribute("scale");
				if (attr != nullptr)
				{
					stringstream ss(attr);
					float scale;
					ss >> scale;
					model->SetScale(XMVectorReplicate(scale));
				}

				AddModel(model);
			}
			obj = obj->NextSiblingElement("obj");
		}

		//AddModel(GeometryModel::CreateFromObjFile(pDevice, (ModelDirectory / models[2]).wstring(), texDir));

		for (auto& obj : Children)
		{
			//auto& ext = obj->BoundBox.Extents;
			//auto s = max(max(ext.x, ext.y), ext.z);
			////s = 5.0f / s;
			//obj->SetScale({ s ,s ,s });
		}
		m_loadingComplete = true;
	});
}

void Causality::Foregrounds::Render(ID3D11DeviceContext * pContext)
{
	pContext->IASetInputLayout(pInputLayout.Get());
	auto pAWrap = States.AnisotropicWrap();
	pContext->PSSetSamplers(0, 1, &pAWrap);
	pContext->RSSetState(pRSState.Get());
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

	//for (auto& obj : Children)
	//{
	//	auto s = (rand() % 1000) / 1000.0;
	//	obj->Rotate(XMQuaternionRotationRollPitchYaw(0, 0.5f * timer.GetElapsedSeconds(), 0));
	//}
}
