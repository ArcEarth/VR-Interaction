#include "pch_bcl.h"
#include "Scene.h"
#include <ShaderEffect.h>

using namespace Causality;
using namespace std;

Scene::Scene()
{
	is_paused = false;
	is_loaded = false;
	primary_cameral = nullptr;
}

Scene::~Scene()
{
	int* p = nullptr;
}


Camera * Scene::PrimaryCamera()
{
	return primary_cameral;
}

bool Scene::SetAsPrimaryCamera(Camera * camera)
{
	if (camera->Scene != this) return false;
	primary_cameral = camera;
	camera->SetRenderTarget(Canvas());
	return true;
}

void Scene::SetRenderDeviceAndContext(RenderDevice & device, RenderContext & context)
{
	assets.SetRenderDevice(device);
	render_device = device;
	render_context = context;
}

void Scene::UpdateRenderViewCache()
{
	if (!camera_dirty) return;
	cameras.clear();
	renderables.clear();
	lights.clear();
	effects.clear();

	auto& aseffects = assets.GetEffects();
	for (auto pe : aseffects)
		effects.push_back(pe);

	for (auto& obj : content->nodes())
	{
		auto pCamera = obj.As<ICamera>();
		if (pCamera != nullptr)
			cameras.push_back(pCamera);

		auto pLight = obj.As<ILight>();
		if (pLight != nullptr)
			lights.push_back(pLight);

		auto pRenderable = obj.As<IRenderable>();
		if (pRenderable != nullptr)
			renderables.push_back(pRenderable);
	}
	camera_dirty = false;
}


void Scene::Update()
{
	if (is_paused) return;
	lock_guard<mutex> guard(content_mutex);
	step_timer.Tick([this]() {
		time_seconds deltaTime(step_timer.GetElapsedSeconds());
		for (auto& pObj : content->nodes())
		{
			if (pObj.IsEnabled())
				pObj.Update(deltaTime);
		}
	});
	if (camera_dirty)
		UpdateRenderViewCache();
}

void Scene::SignalCameraCache() { camera_dirty = true; }

std::mutex & Scene::ContentMutex() { return content_mutex; }


void Scene::Render(RenderContext & context)
{
	// if (!is_loaded) return;
	lock_guard<mutex> guard(content_mutex);

	SetupEffectsLights(nullptr);

	for (auto pCamera : cameras)
	{
		pCamera->BeginFrame(context);
		auto viewCount = pCamera->ViewCount();
		auto pCameraEffect = pCamera->GetRenderEffect();

		for (size_t view = 0; view < viewCount; view++)
		{
			pCamera->SetView(view);
			auto v = pCamera->GetViewMatrix(view);
			auto p = pCamera->GetProjectionMatrix(view);
			auto& viewFrustum = pCamera->GetViewFrustum(view);

			SetupEffectsViewProject(pCameraEffect, v, p);

			for (auto& pRenderable : renderables)
			{
				if (pCamera->AcceptRenderFlags(pRenderable->GetRenderFlags())
					&& pRenderable->IsVisible(viewFrustum))
				{
					pRenderable->UpdateViewMatrix(v, p);
					pRenderable->Render(context, pCameraEffect);
				}
			}
		}
		pCamera->EndFrame();
	}
}

vector<DirectX::IEffect*>& Causality::Scene::GetEffects() {
	return effects;
}

void Scene::SetupEffectsViewProject(DirectX::IEffect* pEffect, const DirectX::XMMATRIX &v, const DirectX::XMMATRIX &p)
{
	auto pME = dynamic_cast<DirectX::IEffectMatrices*>(pEffect);
	if (pME)
	{
		pME->SetView(v);
		pME->SetProjection(p);
	}

	for (auto pEff : effects)
	{
		pME = dynamic_cast<DirectX::IEffectMatrices*>(pEff);
		if (pME)
		{
			pME->SetView(v);
			pME->SetProjection(p);
		}
	}
}

void Causality::Scene::SetupEffectsLights(DirectX::IEffect * pEffect)
{
	using namespace DirectX;

	XM_ALIGNATTR
	struct LightParam
	{
		XMVECTOR color;
		XMVECTOR direction;
		XMMATRIX view;
		XMMATRIX proj;
		ID3D11ShaderResourceView* shadow;
	};

	XMVECTOR ambient = XMVectorSet(0.2, 0.2, 0.2, 1.0f);

	static const int MaxLights = IEffectLights::MaxDirectionalLights;
	LightParam Lps[MaxLights];

	for (int i = 0; i < std::min((int)lights.size(), MaxLights); i++)
	{
		auto pLight = lights[i];
		Lps[i].color = pLight->GetColor();
		Lps[i].direction = pLight->GetFocusDirection();
		Lps[i].shadow = pLight->GetShadowMap();
		Lps[i].view = pLight->GetViewMatrix();
		Lps[i].proj = pLight->GetProjectionMatrix();
	}

	for (auto pEff : effects)
	{
		auto pELS = dynamic_cast<DirectX::IEffectLightsShadow*>(pEff);
		auto pEL = dynamic_cast<DirectX::IEffectLights*>(pEff);

		if (pEL)
		{
			pEL->SetAmbientLightColor(ambient);

			for (int i = 0; i < std::min((int)lights.size(), MaxLights); i++)
			{
				auto pLight = lights[i];
				pEL->SetLightEnabled(i, true);
				pEL->SetLightDiffuseColor(i, Lps[i].color);
				pEL->SetLightSpecularColor(i, Lps[i].color);
				pEL->SetLightDirection(i, Lps[i].direction);

				if (pELS != nullptr)
				{
					pELS->SetLightShadowMap(i, Lps[i].shadow);
					pELS->SetLightView(i, Lps[i].view);
					pELS->SetLightProjection(i, Lps[i].proj);
				}

			}
		}
	}
}

void Scene::Load()
{
}

void Scene::Release()
{
}

bool Scene::IsLoading() const
{
	return false;
}

bool Scene::IsReleasing() const
{
	return false;
}

bool Scene::IsLoaded() const
{
	return false;
}

void Scene::OnNavigatedTo()
{
}

void Scene::OnNavigatedFrom()
{
}
