#pragma once
#include "Renderable.h"
#include "Interactive.h"
#include "SceneObject.h"
#include "CameraObject.h"
#include "LightObject.h"
#include "StepTimer.h"
#include "RenderContext.h"
#include "AssetDictionary.h"
#include <mutex>

namespace Causality
{
	class Frame;

	interface IScene abstract
	{
	public:
		virtual ~IScene() {};

		virtual void Update() = 0;
		virtual void Render(RenderContext& context) = 0;
		virtual void Load() = 0;
		virtual void Release() = 0;

		virtual bool IsLoading() const = 0;
		virtual bool IsReleasing() const = 0;

		virtual bool IsLoaded() const = 0;

		virtual void OnNavigatedTo() = 0;
		virtual void OnNavigatedFrom() = 0;

		virtual SceneObject* Content() = 0;
	};

	enum SceneTimeLineType
	{
		PhysicalTimeInterval, // The time of scene is binded to physical time
		FixedTimeInterval,	  // The time of scene is binded to frame count, every two adjacant frame have same time delta
	};

	// A Scene is a collection of it's contents, interactive logic, 
	class Scene : public IScene
	{
	public:
		Scene();
		~Scene();

		static std::unique_ptr<Scene> LoadSceneFromXML(const string& xml_file);

		void LoadFromXML(const string& xml_file);

		static Scene& GetSceneForCurrentView();

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
		bool	IsPaused() const { return is_paused; }
		void	Pause() { is_paused = true; }
		void	Resume() { is_paused = false; }

		time_seconds	GetLocalTime() const
		{
			return time_seconds(step_timer.GetTotalSeconds());
		}

		double	GetTimeScale() const { return time_scale; }
		void	SetTimeScale(double time_scale) { this->time_scale = time_scale; }

		// Contens operations
		SceneObject*	Content() override;
		SceneObject*	SetContent(SceneObject* sceneRoot);

		AssetDictionary& Assets() { return assets; }
		const AssetDictionary& Assets() const { return assets; }

		ICamera *PrimaryCamera();

		bool SetAsPrimaryCamera(ICamera* camera);

		RenderDevice&			GetRenderDevice() { return render_device; }
		const RenderDevice&		GetRenderDevice() const { return render_device; }
		RenderContext&			GetRenderContext() { return render_context; }
		const RenderContext&	GetRenderContext() const { return render_context; }

		void SetRenderDeviceAndContext(RenderDevice& device, RenderContext& context);

		DirectX::RenderTarget&			Canvas() { return scene_canvas; }
		const DirectX::RenderTarget&	Canvas() const { return scene_canvas; }
		void							SetCanvas(DirectX::RenderTarget& canvas);

		vector<ICamera*>&				GetCameras() { return cameras; }
		vector<ILight*>&				GetLights() { return lights; }
		const vector<const ICamera*>&	GetCameras() const { return reinterpret_cast<const vector<const ICamera*>&>(cameras); }
		const vector<const ILight*>&	GetLights() const { return reinterpret_cast<const vector<const ILight*>&>(lights); }
		vector<DirectX::IEffect*>&		GetEffects();

		void SetupEffectsViewProject(DirectX::IEffect* pEffect, const DirectX::XMMATRIX &v, const DirectX::XMMATRIX &p);
		void SetupEffectsLights(DirectX::IEffect* pEffect);

		void UpdateRenderViewCache();

		void SignalCameraCache();

		std::mutex&	ContentMutex();

	private:
		SceneTimeLineType			timeline_type;
		double						time_scale;
		DirectX::StepTimer			step_timer;
		RenderDevice				render_device;
		RenderContext				render_context;
		AssetDictionary				assets;
		uptr<SceneObject>			content;
		DirectX::RenderTarget		scene_canvas;
		DirectX::RenderableTexture2D back_buffer;
		ICamera						*primary_cameral;

		vector<ICamera*>			cameras;
		vector<ILight*>				lights;
		vector<IRenderable*>		renderables;
		vector<DirectX::IEffect*>	effects;

		std::mutex					content_mutex;

		bool						camera_dirty;
		bool						object_dirty;

		bool						is_paused;
		bool						is_loaded;
		int							loading_count;
	};
}
