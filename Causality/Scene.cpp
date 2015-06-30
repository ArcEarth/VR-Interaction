#include "pch_bcl.h"
#include "Scene.h"

using namespace Causality;
using namespace std;

Causality::Scene::Scene()
{
	is_paused = false;
	is_loaded = false;
	primary_cameral = nullptr;
}

Causality::Scene::~Scene()
{
	int* p = nullptr;
}


void Causality::Scene::Update()
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

void Causality::Scene::SignalCameraCache() { camera_dirty = true; }

std::mutex & Causality::Scene::ContentMutex() { return content_mutex; }


void Causality::Scene::Render(RenderContext & context)
{
	// if (!is_loaded) return;
	lock_guard<mutex> guard(content_mutex);

	for (auto pCamera : cameras)
	{
		pCamera->BeginFrame();
		for (size_t view = 0; view < pCamera->ViewCount(); view++)
		{
			pCamera->SetView(view);
			auto v = pCamera->GetViewMatrix(view);
			auto p = pCamera->GetProjectionMatrix(view);
			auto& viewFrustum = pCamera->GetViewFrustum(view);

			SetupEffectsViewProject(v, p);

			for (auto& pRenderable : renderables)
			{
				if (pRenderable->IsVisible(viewFrustum))
				{
					pRenderable->UpdateViewMatrix(v, p);
					pRenderable->Render(context);
				}
			}
		}
		pCamera->EndFrame();
	}
}

void Causality::Scene::SetupEffectsViewProject(const DirectX::XMMATRIX &v, const DirectX::XMMATRIX &p)
{
	auto& effects = assets.GetEffects();
	for (auto& pEff : effects)
	{
		auto pME = dynamic_cast<DirectX::IEffectMatrices*>(pEff);
		if (pME)
		{
			pME->SetView(v);
			pME->SetProjection(p);
		}
	}
}

void Causality::Scene::Load()
{
}

void Causality::Scene::Release()
{
}

bool Causality::Scene::IsLoading() const
{
	return false;
}

bool Causality::Scene::IsReleasing() const
{
	return false;
}

bool Causality::Scene::IsLoaded() const
{
	return false;
}

void Causality::Scene::OnNavigatedTo()
{
}

void Causality::Scene::OnNavigatedFrom()
{
}
