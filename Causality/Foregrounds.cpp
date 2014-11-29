#include "Foregrounds.h"
#include "CausalityApplication.h"
#include <tinyxml2.h>
#include <sstream>
#include <mutex>
#include <thread>
#include "Common\DebugVisualizer.h"

using namespace Causality;
using namespace DirectX;
using namespace DirectX::Scene;
using namespace std;
using namespace Platform;
using namespace Platform::Fundation;
extern wstring ResourcesDirectory;

const static wstring SkyBoxTextures[6] = {
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Right.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Left.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Top.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Bottom.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Front.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Back.dds"),
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
	pBackground = nullptr;
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

		pBackground = std::make_unique<SkyDome>(pDevice, SkyBoxTextures);

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
	if (pBackground)
		pBackground->Render(pContext);

	pContext->IASetInputLayout(pInputLayout.Get());
	auto pAWrap = States.AnisotropicWrap();
	pContext->PSSetSamplers(0, 1, &pAWrap);
	pContext->RSSetState(pRSState.Get());
	ModelCollection::Render(pContext, pEffect.get());

	dxout.Begin();
	{
		lock_guard<mutex> guard(m_HandFrameMutex);
		for (const auto& hand : m_Frame.hands())
		{
			for (const auto& finger : hand.fingers())
			{
				for (size_t i = 0; i < 4; i++)
				{
					const auto & bone = finger.bone((Leap::Bone::Type)i);
					if (i == 0)
						dxout.DrawSphere(bone.prevJoint().toVector3<Vector3>(), 0.1f, DirectX::Colors::Lime);

					dxout.DrawSphere(bone.nextJoint().toVector3<Vector3>(), 0.1f, DirectX::Colors::Lime);
					dxout.DrawLine(bone.prevJoint().toVector3<Vector3>(), bone.nextJoint().toVector3<Vector3>(), DirectX::Colors::White);
				}
			}
		}
	}
	dxout.End();
}

void XM_CALLCONV Causality::Foregrounds::UpdateViewMatrix(DirectX::FXMMATRIX view)
{
	if (pEffect)
		pEffect->SetView(view);
	if (pBackground)
		pBackground->UpdateViewMatrix(view);
	dxout.SetView(view);
}

void XM_CALLCONV Causality::Foregrounds::UpdateProjectionMatrix(DirectX::FXMMATRIX projection)
{
	if (pEffect)
		pEffect->SetProjection(projection);
	if (pBackground)
		pBackground->UpdateProjectionMatrix(projection);
	dxout.SetProjection(projection);
}

void Causality::Foregrounds::UpdateAnimation(StepTimer const & timer)
{

	//for (auto& obj : Children)
	//{
	//	auto s = (rand() % 1000) / 1000.0;
	//	obj->Rotate(XMQuaternionRotationRollPitchYaw(0, 0.5f * timer.GetElapsedSeconds(), 0));
	//}
}

void Causality::Foregrounds::OnHandsTracked(const UserHandsEventArgs & e)
{
}

void Causality::Foregrounds::OnHandsTrackLost(const UserHandsEventArgs & e)
{
}

void Causality::Foregrounds::OnHandsMove(const UserHandsEventArgs & e)
{
	std::lock_guard<mutex> guard(m_HandFrameMutex);
	m_Frame = e.sender.frame();
}
