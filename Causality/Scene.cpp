#include "pch_bcl.h"
#include "Scene.h"

using namespace Causality;

Causality::Scene::Scene()
{
	is_paused = false;
	is_loaded = false;
}

Causality::Scene::~Scene()
{
	int* p = nullptr;
}


void Causality::Scene::Update()
{
	if (is_paused || !is_loaded) return;
	step_timer.Tick([this]() {
		time_seconds deltaTime(step_timer.GetElapsedSeconds());
		for (auto& pObj : content->nodes())
		{
			if (pObj.IsEnabled())
				pObj.Update(deltaTime);
		}
	});
}

void Causality::Scene::Render(RenderContext & context)
{
	if (!is_loaded) return;

	for (auto& pCamera : cameras)
	{
		pCamera->BeginFrame();
		for (size_t view = 0; view < pCamera->ViewCount(); view++)
		{
			pCamera->SetView(view);
			auto v = pCamera->GetViewMatrix(view);
			auto p = pCamera->GetProjectionMatrix(view);
			auto& viewFrustum = pCamera->GetViewFrustum(view);

			auto default_effect = assets.GetEffect("");
			auto pEm = dynamic_cast<DirectX::IEffectMatrices*>(default_effect);
			if (pEm)
			{
				pEm->SetView(v);
				pEm->SetProjection(p);
			}

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
