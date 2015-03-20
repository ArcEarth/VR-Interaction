#pragma once
#include "Common\Renderable.h"
#include "Interactive.h"
#include "SceneObject.h"
#include "Common\Carmera.h"
#include "Common\StepTimer.h"
#include "RenderContext.h"
#include "Common\Carmera.h"
#include "AssetDictionary.h"
#include "PrimaryCamera.h"

namespace Causality
{
	interface IScene abstract
	{
	public:
		virtual ~IScene();

		virtual void Update() = 0;
		virtual void Render(RenderContext& context) = 0;
		virtual void Load() = 0;
		virtual void Release() = 0;

		virtual bool IsLoading() const = 0;
		virtual bool IsReleasing() const = 0;

		virtual bool IsLoaded() const = 0;

		virtual void OnNavigatedTo() = 0;
		virtual void OnNavigatedFrom() = 0;
	};

	// A Scene is a collection of it's contents, interactive logic, 
	class Scene : public IScene
	{
	public:
		static std::unique_ptr<Scene> LoadSceneFromXML(const string& xml_file);

		static Scene& GetSceneFroCurrentView();

		// Inherit from IScene
		virtual void Update() override;
		virtual void Render(RenderContext& context) override;
		virtual void Load() override;
		virtual void Release() override;

		virtual bool IsLoading() const override;
		virtual bool IsReleasing() const override;

		virtual bool IsLoaded() const override;

		virtual void OnNavigatedTo() override;
		virtual void OnNavigatedFrom() override;

		// Timeline operations
		bool	IsPaused() const;
		void	Pause();
		void	Resume();

		time_seconds	GetLocalTime() const
		{
			return time_seconds(step_timer.GetElapsedSeconds());
		}

		double	GetTimeScale() const;
		void	SetTimeScale(double time_scale);

		// Contens operations
		SceneObject*	Content();

		AssetDictionary& Assets();
		const AssetDictionary& Assets() const;

		DirectX::RenderTarget&			RenderTarget();
		const DirectX::RenderTarget&	RenderTarget() const;

	private:
		double						time_scale;
		DirectX::StepTimer			step_timer;
		AssetDictionary				assets;
		uptr<SceneObject>			content;
		DirectX::RenderTarget		scene_canvas;

		bool						is_paused;
		bool						is_loaded;
		int							loading_count;
	};
}
