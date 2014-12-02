#include <iostream>
#include "Foregrounds.h"
#include "CausalityApplication.h"
#include <tinyxml2.h>
#include <sstream>
#include <mutex>
#include <thread>
#include "Common\DebugVisualizer.h"
#include <random>

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

Causality::WorldScene::WorldScene(const std::shared_ptr<DirectX::DeviceResources>& pResouce)
	: States(pResouce->GetD3DDevice())
{
	m_HaveHands = false;
	LoadAsync(pResouce->GetD3DDevice());
}

WorldScene::~WorldScene()
{
}

void Causality::WorldScene::LoadAsync(ID3D11Device* pDevice)
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

		//pBackground = std::make_unique<SkyDome>(pDevice, SkyBoxTextures);

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
				XMFLOAT3 v = model->BoundOrientedBox.Extents;
				std::sort(&v.x, &v.z + 1);
				v.x /= v.z;
				v.y /= v.z;
				m_ModelsFeature[model->Name] = { v.x, v.y };
				std::cout << "[Model] f(" << model->Name << ") = " << m_ModelsFeature[model->Name] << std::endl;

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

void Causality::WorldScene::Render(ID3D11DeviceContext * pContext)
{
	if (pBackground)
		pBackground->Render(pContext);

	pContext->IASetInputLayout(pInputLayout.Get());
	auto pAWrap = States.AnisotropicWrap();
	pContext->PSSetSamplers(0, 1, &pAWrap);
	pContext->RSSetState(pRSState.Get());
	ModelCollection::Render(pContext, pEffect.get());
	Vector3 conners[8];
	BoundingOrientedBox obox;
	BoundingBox box;
	dxout.Begin();
	{
		//Draw axias
		dxout.DrawSphere({ 0,0,0,0.02 }, Colors::Red);
		dxout.DrawLine({ -5,0,0 }, { 5,0,0 }, Colors::Red);
		dxout.DrawLine({ 0,-5,0 }, { 0,5,0 }, Colors::Green);
		dxout.DrawLine({ 0,0,-5 }, { 0,0,5 }, Colors::Blue);
		dxout.DrawTriangle({ 5.05f,0,0 }, { 4.95,0.05,0 }, { 4.95,-0.05,0 }, Colors::Red);
		dxout.DrawTriangle({ 0,5.05f,0 }, { -0.05,4.95,0 }, { 0.05,4.95,0 }, Colors::Green);
		dxout.DrawTriangle({ 0,0,5.05f }, { 0.05,0,4.95 }, { -0.05,0,4.95 }, Colors::Blue);
		dxout.DrawTriangle({ 5.05f,0,0 }, { 4.95,-0.05,0 }, { 4.95,0.05,0 }, Colors::Red);
		dxout.DrawTriangle({ 0,5.05f,0 }, { 0.05,4.95,0 }, { -0.05,4.95,0 }, Colors::Green);
		dxout.DrawTriangle({ 0,0,5.05f }, { -0.05,0,4.95 }, { 0.05,0,4.95 }, Colors::Blue);


		auto fh = m_HandDescriptionFeature;

		auto s = Children.size();
		for (size_t i = 0; i < s; i++)
		{
			const auto& model = Children[i];
			XMMATRIX transform = model->GetModelMatrix();
			model->GetOrientedBoundingBox().GetCorners(conners);
			DrawBox(conners, DirectX::Colors::DarkGreen);
			if (m_HaveHands)
			{
				const auto& fm = m_ModelsFeature[model->Name];
				auto dis = Vector2::Distance(fm, fh);
				std::cout << dis << ' ';
				dis = 1.414 - dis;
				Color c = Color::Lerp({ 1,0,0 }, { 0,1,0 }, dis);
				for (size_t i = 0; i < 8; i++)
				{
					dxout.DrawSphere(conners[i], 0.1 * dis, c);
				}
			}
			auto pModel = dynamic_cast<Model*>(model.get());
			if (pModel)
			{
				for (const auto& part : pModel->Parts)
				{
					part.BoundOrientedBox.Transform(obox, transform);
					obox.GetCorners(conners);
					DrawBox(conners, DirectX::Colors::Orange);
				}
			}
		}
		if (m_HaveHands)
			std::cout << std::endl;
	}
	if (m_HaveHands)
	{
		auto pCamera = App::Current()->GetPrimaryCamera();

		XMMATRIX leap2world = m_FrameTransform;// XMMatrixScalingFromVector(XMVectorReplicate(0.001f)) * XMMatrixTranslation(0,0,-1.0) * XMMatrixRotationQuaternion(pCamera->GetOrientation()) * XMMatrixTranslationFromVector((XMVECTOR)pCamera->GetPosition());
		std::lock_guard<mutex> guard(m_HandFrameMutex);
		for (const auto& hand : m_Frame.hands())
		{
			for (const auto& finger : hand.fingers())
			{
				for (size_t i = 0; i < 4; i++)
				{
					const auto & bone = finger.bone((Leap::Bone::Type)i);
					XMVECTOR bJ = XMVector3Transform(bone.prevJoint().toVector3<Vector3>(), leap2world);
					XMVECTOR eJ = XMVector3Transform(bone.nextJoint().toVector3<Vector3>(), leap2world);
					if (i == 0)
						dxout.DrawSphere(bJ, 0.01f, DirectX::Colors::Lime);

					// The unit of leap is millimeter
					dxout.DrawSphere(eJ, 0.01f, DirectX::Colors::Lime);
					dxout.DrawLine(bJ, eJ, DirectX::Colors::White);
				}
			}
		}

		// NOT VALIAD!~!!!!!
		if (m_HandTrace.size() > 0)
		{
			//auto pJoints = m_HandTrace.linearize();
			m_CurrentHandBoundingBox.GetCorners(conners);
			DrawBox(conners, Colors::LimeGreen);
			m_HandTraceBoundingBox.GetCorners(conners);
			DrawBox(conners, Colors::YellowGreen);
			for (int i = m_HandTrace.size() - 1; i >= std::max(0, (int) m_HandTrace.size() - TraceLength); i--)
			{
				const auto& h = m_HandTrace[i];
				float radius = (i + 1 - std::max(0U, m_HandTrace.size() - TraceLength)) / (std::min<float>(m_HandTrace.size(), TraceLength));
				for (size_t j = 0; j < h.size(); j++)
				{
					dxout.DrawSphere(h[j], 0.005 * radius, Colors::LimeGreen);
				}
			}

		}
	}
	dxout.End();

}

void Causality::WorldScene::DrawBox(DirectX::SimpleMath::Vector3  conners [], DirectX::CXMVECTOR color)
{
	dxout.DrawLine(conners[0], conners[1], color);
	dxout.DrawLine(conners[1], conners[2], color);
	dxout.DrawLine(conners[2], conners[3], color);
	dxout.DrawLine(conners[3], conners[0], color);

	dxout.DrawLine(conners[3], conners[7], color);
	dxout.DrawLine(conners[2], conners[6], color);
	dxout.DrawLine(conners[1], conners[5], color);
	dxout.DrawLine(conners[0], conners[4], color);

	dxout.DrawLine(conners[4], conners[5], color);
	dxout.DrawLine(conners[5], conners[6], color);
	dxout.DrawLine(conners[6], conners[7], color);
	dxout.DrawLine(conners[7], conners[4], color);

}

void XM_CALLCONV Causality::WorldScene::UpdateViewMatrix(DirectX::FXMMATRIX view)
{
	if (pEffect)
		pEffect->SetView(view);
	if (pBackground)
		pBackground->UpdateViewMatrix(view);
	dxout.SetView(view);
}

void XM_CALLCONV Causality::WorldScene::UpdateProjectionMatrix(DirectX::FXMMATRIX projection)
{
	if (pEffect)
		pEffect->SetProjection(projection);
	if (pBackground)
		pBackground->UpdateProjectionMatrix(projection);
	dxout.SetProjection(projection);
}

void Causality::WorldScene::UpdateAnimation(StepTimer const & timer)
{
	if (m_HandTrace.size() > 0)
	{
		{ // Critia section
			std::lock_guard<mutex> guard(m_HandFrameMutex);
			const int plotSize = 45;
			BoundingOrientedBox::CreateFromPoints(m_CurrentHandBoundingBox, m_HandTrace.back().size(), m_HandTrace.back().data(), sizeof(Vector3));
			m_TracePoints.clear();
			Color color = Colors::LimeGreen;
			for (int i = m_HandTrace.size() - 1; i >= std::max(0, (int) m_HandTrace.size() - plotSize); i--)
			{
				const auto& h = m_HandTrace[i];
				//float radius = (i + 1 - std::max(0U, m_HandTrace.size() - plotSize)) / (std::min<float>(m_HandTrace.size(), plotSize));
				for (size_t j = 0; j < h.size(); j++)
				{
					//dxout.DrawSphere(h[j], 0.005 * radius, color);
					m_TracePoints.push_back(h[j]);
				}
			}
		}
		BoundingOrientedBox::CreateFromPoints(m_HandTraceBoundingBox, m_TracePoints.size(), m_TracePoints.data(), sizeof(Vector3));
		XMFLOAT3 v = m_HandTraceBoundingBox.Extents;
		std::sort(&v.x, &v.z + 1);
		m_HandDescriptionFeature = { v.x / v.z, v.y / v.z };

		// ASSUMPTION: Extends is sorted from!


		//XMMATRIX invTrans = XMMatrixAffineTransformation(g_XMOne / XMLoadFloat3(&m_HandTraceBoundingBox.Extents), XMVectorZero(), XMQuaternionInverse(XMLoadFloat4(&m_HandTraceBoundingBox.Orientation)), -XMLoadFloat3(&m_HandTraceBoundingBox.Center));
		//int sampleCount = std::min<int>(m_TraceSamples.size(), TraceLength)*m_TraceSamples[0].size();
		//for (const auto& model : Children)
		//{
		//	auto pModel = dynamic_cast<Model*>(model.get());
		//	auto inCount = 0;
		//	if (pModel)
		//	{
		//		auto obox = model->GetOrientedBoundingBox();
		//		XMMATRIX fowTrans = XMMatrixAffineTransformation(XMLoadFloat3(&obox.Extents), XMVectorZero(), XMQuaternionIdentity(), XMVectorZero());
		//		fowTrans = invTrans * fowTrans;
		//		auto pSample = m_TraceSamples.front().data();
		//		for (size_t i = 0; i < sampleCount; i++)
		//		{
		//			const auto& point = pSample[-i];
		//			XMVECTOR p = XMVector3Transform(point, fowTrans);
		//			int j;
		//			for ( j = 0; j < pModel->Parts.size(); j++)
		//			{
		//				if (pModel->Parts[j].BoundOrientedBox.Contains(p))
		//					break;
		//			}
		//			if (j >= pModel->Parts.size())
		//				inCount++;
		//		}
		//	}
		//	m_ModelDetailSimilarity[model->Name] = (float) inCount / (float)sampleCount;
		//}



	}
	//for (auto& obj : Children)
	//{
	//	auto s = (rand() % 1000) / 1000.0;
	//	obj->Rotate(XMQuaternionRotationRollPitchYaw(0, 0.5f * timer.GetElapsedSeconds(), 0));
	//}
}

void Causality::WorldScene::OnHandsTracked(const UserHandsEventArgs & e)
{
	m_HaveHands = true;
}

void Causality::WorldScene::OnHandsTrackLost(const UserHandsEventArgs & e)
{
	if (e.sender.frame().hands().count() == 0)
	{
		m_HaveHands = false;
		std::lock_guard<mutex> guard(m_HandFrameMutex);
		m_HandTrace.clear();
		m_TraceSamples.clear();
	}
}

void Causality::WorldScene::OnHandsMove(const UserHandsEventArgs & e)
{
	std::lock_guard<mutex> guard(m_HandFrameMutex);
	m_Frame = e.sender.frame();
	m_FrameTransform = e.toWorldTransform;
	XMMATRIX leap2world = m_FrameTransform;
	//std::array<DirectX::Vector3, 25> joints;
	std::vector<DirectX::BoundingOrientedBox> handBoxes;
	float fingerStdev = 0.02;
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<float> normalDist(0, fingerStdev);
	std::uniform_real<float> uniformDist;

	int handIdx = 0;
	for (const auto& hand : m_Frame.hands())
	{
		int fingerIdx = 0; // hand idx
		m_HandTrace.emplace_back();
		//m_TraceSamples.emplace_back();
		//auto& samples = m_TraceSamples.back();
		auto& joints = m_HandTrace.back();
		for (const auto& finger : hand.fingers())
		{
			XMVECTOR bJ = XMVector3Transform(finger.bone((Leap::Bone::Type)0).prevJoint().toVector3<Vector3>(), leap2world);
			joints[fingerIdx * 5] = bJ;
			for (size_t boneIdx = 0; boneIdx < 4; boneIdx++) // bone idx
			{
				const auto & bone = finger.bone((Leap::Bone::Type)boneIdx);
				XMVECTOR eJ = XMVector3Transform(bone.nextJoint().toVector3<Vector3>(), leap2world);
				joints[fingerIdx * 5 + boneIdx + 1] = eJ;


				//auto dir = eJ - bJ;
				//float dis = XMVectorGetX(XMVector3Length(dir));
				//if (abs(dis) < 0.001)
				//	continue;
				//XMVECTOR rot = XMQuaternionRotationVectorToVector(g_XMIdentityR1, dir);
				//bJ = eJ;
				//for (size_t k = 0; k < 100; k++) // bone idx
				//{
				//	float x = normalDist(gen);
				//	float h = uniformDist(gen);
				//	float z = normalDist(gen);
				//	XMVECTOR disp = XMVectorSet(x, h*dis, z, 1);
				//	disp = XMVector3Rotate(disp, rot);
				//	disp += bJ;
				//	samples[(fingerIdx * 4 + boneIdx) * 100 + k] = disp;
				//}
			}
			fingerIdx++;
		}
		handIdx++;
		while (m_HandTrace.size() > 60)
		{
			m_HandTrace.pop_front();
			//m_TraceSamples.pop_front();
		}
	}


	std::cout << "[Leap] Hands Move." << std::endl;
}
