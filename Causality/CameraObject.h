#pragma once
#include "OculusRift.h"
#include "RenderContext.h"
#include "SceneObject.h"
#include <PostProcessingEffect.h>
#include <ShadowMapEffect.h>
#include <PostProcessingEffect.h>

namespace Causality
{
	struct CameraBuffer
	{
		DirectX::XMFLOAT4X4	ViewProjectionMatrix;
		DirectX::XMFLOAT4	CameraPosition;
		DirectX::XMFLOAT4	CameraFocus;
		DirectX::XMFLOAT4X4 ViewMatrix;
		DirectX::XMFLOAT4X4 ProjectionMatrix;
	};

	// The basic functions for camera, to provide View/Projection Matrix, Setup Position and focus
	class IViewControl abstract
	{
	public:
		virtual DirectX::XMMATRIX				GetViewMatrix() const = 0;
		virtual DirectX::XMMATRIX				GetProjectionMatrix() const = 0;
		virtual void							FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) = 0;
		virtual const BoundingGeometry&			GetViewFrustum() const = 0;

		inline DirectX::XMMATRIX				GetViewProjectionMatrix() const
		{
			DirectX::XMMATRIX mat = GetViewMatrix();
			mat *= GetProjectionMatrix();
			return mat;
		}
	};

	// Control the logic of render target setup and post-processing needed for current camera
	class IRenderControl abstract
	{
	public:
		// Called in the beginning of per-frame
		virtual void Begin(RenderContext& context) = 0;
		// Called in in the end of per frame, should call Prenset inside
		virtual void End() = 0;

		virtual DirectX::IEffect* GetRenderEffect() = 0;
		virtual DirectX::IPostEffect* GetPostEffect() = 0;
		virtual DirectX::RenderTarget& GetRenderTarget() = 0;
		virtual DirectX::Texture2D& GetOutput() = 0;

		virtual bool AcceptRenderFlags(RenderFlags flags) = 0;
	};

	class ICamera
	{
	public:
		virtual ~ICamera()
		{}

		virtual size_t ViewCount() const = 0;
		virtual IViewControl* GetView(int view = 0) = 0;
		virtual size_t ViewRendererCount(int view = 0) const = 0;
		virtual IRenderControl* GetViewRenderer(int view = 0, int renderer = 0) = 0;

		inline  const IViewControl* GetView(int view = 0) const { return const_cast<ICamera*>(this)->GetView(view); }
	};

	class CameraViewControl : virtual public IViewControl
	{
	public:
		CameraViewControl();
		virtual DirectX::XMMATRIX		GetViewMatrix() const override;
		virtual DirectX::XMMATRIX		GetProjectionMatrix() const override;
		virtual void					FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) override;
		virtual const BoundingGeometry&	GetViewFrustum() const override;

		DirectX::IRigid* GetAttachedRigid() const { return m_Parent; }
		void	SetAttachedRigid(DirectX::IRigid* pRigid);

		void	SetPerspective(float fovRadius, float aspectRatioHbyW, float Near = 0.01f, float Far = 100.0f);
		void	SetOrthographic(float viewWidth, float viewHeight, float Near = 0.01f, float Far = 100.0f);
		void	SetHandness(bool rightHand);

		bool	IsPerspective() const;
		float	GetFov() const;
		float	GetAspectRatio() const;
		float	GetNear() const;
		float	GetFar() const;
		void	SetFov(float fov);
		void	SetAspectRatio(float aspectHbyW);
		void	SetNear(float _near);
		void	SetFar(float _far);

		static const DirectX::XMVECTORF32 Foward, Up;
	private:
		DirectX::IRigid*							m_Parent;
		Vector3										m_Displacement; // local transform form view to camera
		Vector3										m_Forward;
		Vector3										m_UpDir;
		bool										m_IsRightHand;
		bool										m_IsPerspective;
		float										m_Near, m_Far;
		float										m_Fov, m_AspectRatio;

		mutable BoundingGeometry					m_ExtrinsicViewFrutum; // the frutum also transformed by View Matrix
		mutable BoundingGeometry					m_ViewFrutum; // the frutum defined by Projection Matrix
		mutable DirectX::Matrix4x4					m_ViewCache; // extrinsic matrix
		mutable DirectX::Matrix4x4					m_ProjectionCache; // instrinsic matrix
		mutable bool								m_ViewDirty;
		mutable bool								m_ProjectDirty;

		DirectX::XMMATRIX UpdateProjectionCache() const;
	};

	// A Effect camera always follow it's owner's view and projection, but have different render control
	class EffectRenderControl : virtual public IRenderControl
	{
	public:
		EffectRenderControl();

		~EffectRenderControl();

		// Inherited via IRenderControl
		virtual void Begin(RenderContext& context) override;
		virtual void End() override;
		virtual bool AcceptRenderFlags(RenderFlags flags) override;
		// Only need for Stereo or more camera
		void SetView(IViewControl *pViewControl);

		virtual DirectX::IEffect* GetRenderEffect();
		virtual DirectX::IPostEffect* GetPostEffect();

		void SetRequestRenderFlags(RenderFlags flags);

		void SetRenderEffect(const std::shared_ptr<DirectX::IEffect>& pEffect);
		void SetPostEffect(const std::shared_ptr<DirectX::IPostEffect>& pPostEffect);

		void	SetRenderTargetClearence(bool ifClear);
		Color	GetBackground() const;
		void	SetBackground(const Color& color);


		virtual DirectX::RenderTarget&	GetRenderTarget() override;
		virtual DirectX::Texture2D&		GetOutput() override;

		void SetRenderTarget(DirectX::RenderTarget & renderTarget);
		void SetPostEffectOutput(DirectX::RenderableTexture2D& output);

	protected:
		bool										m_IfClearRenderTarget;
		bool										m_HaveItemRendered;
		RenderFlags									m_RequstFlags;
		Color										m_Background;
		RenderContext								m_pRenderContext;
		std::shared_ptr<DirectX::IEffect>			m_pEffect;
		std::shared_ptr<DirectX::IPostEffect>		m_pPostEffect;
		DirectX::RenderTarget						m_RenderTarget;
		DirectX::RenderableTexture2D				m_PostEffectOutput;
	};

	class SingleViewCamera : public SceneObject, public virtual ICamera, public CameraViewControl
	{
	public:
		SingleViewCamera()
		{
			CameraViewControl::SetAttachedRigid(this);
		}

		virtual size_t ViewCount() const override { return 1; };
		virtual IViewControl* GetView(int view = 0) override { return this; };

		// A direct call to monolith camera's focus will change camera's orientation instead of CameraViewControl's local coords
		virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) override;
	};

	// Standard camera with single viewport and single rendering pass
	class Camera : public SingleViewCamera, public EffectRenderControl
	{
	public:
		Camera();

		virtual size_t ViewRendererCount(int view = 0) const override { return 1; }
		virtual IRenderControl* GetViewRenderer(int view = 0, int renderer = 0) override { return this; }

		void SetRenderTarget(DirectX::RenderTarget & renderTarget);
	};

	class MultipassCamera : public SingleViewCamera
	{
	public:
		virtual size_t ViewRendererCount(int view = 0) const { return m_pRenderers.size(); }
		virtual IRenderControl* GetViewRenderer(int view = 0, int renderer = 0) { return m_pRenderers[renderer].get(); }
	protected:
		vector<shared_ptr<EffectRenderControl>> m_pRenderers;
	};

	class MuiltiviewCamera : public SceneObject, public ICamera
	{
	public:
		virtual size_t ViewCount() const { return m_Viewports.size(); };
		virtual IViewControl* GetView(int view = 0) { return m_Viewports[view].get(); };
		virtual size_t ViewRendererCount(int view = 0) const { return m_ViewportRenderers[view].size(); }
		virtual IRenderControl* GetViewRenderer(int view = 0, int renderer = 0) { return m_pRenderers[m_ViewportRenderers[view][renderer]].get(); }

	protected:
		vector<unique_ptr<CameraViewControl>>	m_Viewports;
		vector<vector<unsigned>>				m_ViewportRenderers;
		vector<shared_ptr<EffectRenderControl>>	m_pRenderers;
	};



	class SoftShadowCamera : public MultipassCamera
	{
	public:
		SoftShadowCamera(ID3D11Device* pDevice, DirectX::RenderTarget& canvas);
	};

	class HMDCamera : public MuiltiviewCamera
	{
	private:
		std::shared_ptr<Devices::OculusRift>		m_pRift;
	};
}
