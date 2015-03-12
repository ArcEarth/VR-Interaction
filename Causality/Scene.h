#pragma once
#include "Common\Renderable.h"
#include "Interactive.h"
#include "SceneObject.h"
#include "Common\Carmera.h"
#include "Common\StepTimer.h"

namespace Causality
{
	// Controls the asset name resolve and loading
	class ResourceDictionary
	{
	};

	interface IScene abstract: virtual public IRenderable
	{
	public:
		virtual ~IScene();

		virtual bool IsLoading() const = 0;
		virtual bool IsReleasing() const = 0;

		virtual bool IsLoaded() const = 0;
		virtual void Load() = 0;
		virtual void Release() = 0;

		virtual void OnNavigatedTo() = 0;
		virtual void OnNavigatedFrom() = 0;
	};

	// A Scene is a collection of it's contents, interactive logic, 
	class Scene : public IScene, virtual public ITimeAnimatable
	{
		static Scene& GetSceneFroCurrentView();

		bool	IsPaused() const;
		void	Pause();
		void	Resume();

		time_seconds	GetLocalTime() const
		{
		}

		double	GetTimeScale() const;
		void	SetTimeScale(double time_scale);

	private:
		DirectX::StepTimer			step_timer;
		ResourceDictionary			resouces;
		std::vector<SceneObject>	children;
		DirectX::RenderTarget		scene_canvas;
	};
}
