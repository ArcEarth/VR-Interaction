#include "pch_bcl.h"
#include "Scene.h"

using namespace Causality;

void Causality::Scene::Update()
{
	time_seconds deltaTime(step_timer.GetElapsedSeconds());
	for (auto& pObj : content->nodes())
	{
		auto pTimeable = pObj.As<ITimeAnimatable>();
		if (pTimeable != nullptr)
		{ 
			pTimeable->UpdateAnimation(deltaTime);
		}
	}
}

void Causality::Scene::Render(RenderContext & context)
{
	for (auto& pCamera : cameras)
	{
		pCamera->BeginFrame();
		for (size_t view = 0; view < pCamera->ViewCount(); view++)
		{
			pCamera->SetView(view);
			auto v = pCamera->GetViewMatrix(view);
			auto p = pCamera->GetProjectionMatrix(view);
			auto& viewFrustum = pCamera->GetViewFrustum(view);
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
