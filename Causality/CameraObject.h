#pragma once
#include "OculusRift.h"
#include "RenderContext.h"
#include "SceneObject.h"

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
	class ICameraViewControl abstract
	{
	public:
		virtual size_t							ViewCount() const = 0;
		virtual DirectX::XMMATRIX				GetViewMatrix(size_t view = 0) const = 0;
		virtual DirectX::XMMATRIX				GetProjectionMatrix(size_t view = 0) const = 0;
		virtual void							FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) = 0;
		virtual bool XM_CALLCONV				IsInView(DirectX::FXMVECTOR pos, size_t view = 0) const = 0;
		virtual const BoundingFrustum&			GetViewFrustum(size_t view = 0) const = 0;

		inline DirectX::XMMATRIX				GetViewProjectionMatrix(size_t view = 0) const
		{
			DirectX::XMMATRIX mat = GetViewMatrix(view);
			mat *= GetProjectionMatrix(view);
			return mat;
		}
	};

	// Control the logic of render target setup and post-processing needed for current camera
	class ICameraRenderControl abstract
	{
	public:
		// Called in the beginning of per-frame
		virtual void BeginFrame(RenderContext& context) = 0;
		// Called in in the end of per frame, should call Prenset inside
		virtual void EndFrame() = 0;
		// Called in advance of per-view rendering
		virtual void SetView(size_t view = 0) = 0;

		virtual DirectX::IEffect* GetRenderEffect();

		virtual bool AcceptRenderFlags(RenderFlags flags) = 0;

		//virtual RenderTarget& GetRenderTarget(size_t view = 0) = 0;
		//virtual RenderTarget& SetRenderTarget(size_t view = 0) = 0;
	};

	class ICamera : public ICameraRenderControl, public ICameraViewControl
	{
	public:
		virtual ~ICamera()
		{}
	};

	// A Camera setup which works both in the case with OculusRift and Normal Monolith Camera
	class Camera : virtual public SceneObject, public ICamera
	{
	public:
		Camera();

		virtual ~Camera() override;

		// Inherited via ICameraRenderControl
		virtual void BeginFrame(RenderContext& context) override;
		virtual void EndFrame() override;
		virtual bool AcceptRenderFlags(RenderFlags flags) override;
		// Only need for Stereo or more camera
		virtual void SetView(size_t view = 0) override;

		// This ignore the effect of Oculus

		// Inherited via ICamera
		virtual size_t					ViewCount() const override;
		virtual DirectX::XMMATRIX		GetViewMatrix(size_t view = 0) const override;
		virtual DirectX::XMMATRIX		GetProjectionMatrix(size_t view = 0) const override;
		virtual void					FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) override;
		virtual const BoundingFrustum&	GetViewFrustum(size_t view = 0) const override;
		virtual bool XM_CALLCONV		IsInView(DirectX::FXMVECTOR pos, size_t view = 0) const override;

		// Inherited via ICameraParameters
		void	SetPerspective(float fovRadius, float aspectRatioHbyW, float Near = 0.01f, float Far = 100.0f);
		void	SetOrthographic(float viewWidth, float viewHeight);

		bool	IsPerspective() const;
		float	GetFov() const;
		float	GetAspectRatio() const;
		float	GetNear() const;
		float	GetFar() const;

		DirectX::RenderTarget&	GetRenderTarget(int view = 0);
		void					SetRenderTarget(DirectX::RenderTarget& renderTarget, int view = 0, bool autoAspect = true);
		void					SetRenderTarget(DirectX::RenderTarget&& renderTarget, int view = 0, bool autoAspect = true);
		void					SetRenderContext(const RenderContext& context);

		// Stereo Settings
		void EnableStereo(const std::shared_ptr<Devices::OculusRift>& pRift);
		void DisableStoreo();
		bool IsStereoEnabled() const;

		void XM_CALLCONV	Move(DirectX::FXMVECTOR p);
		void XM_CALLCONV	Rotate(DirectX::FXMVECTOR q);

		Color	GetBackground() const { return m_Background; }
		void	SetBackground(const Color& color) { m_Background = color; }

		static const DirectX::XMVECTORF32 Foward, Up;

	private:
		bool										m_IsPerspective;
		float										m_Near, m_Far;							
		float										m_Fov,	m_AspectRatio;
		Color										m_Background;

		std::shared_ptr<DirectX::IEffect>			m_pEffect;
		DirectX::RenderTarget						m_RenderTarget;
		RenderContext								m_pRenderContext;
		std::shared_ptr<Devices::OculusRift>		m_pRift;
		mutable BoundingFrustum						m_ExtrinsicViewFrutum; // the frutum also transformed by View Matrix
		mutable BoundingFrustum						m_ViewFrutum; // the frutum defined by Projection Matrix
		mutable DirectX::Matrix4x4					m_ViewCache; // extrinsic matrix
		mutable DirectX::Matrix4x4					m_ProjectionCache; // instrinsic matrix
		mutable bool								m_ViewDirty;
		mutable bool								m_ProjectDirty;

		DirectX::XMMATRIX UpdateProjectionCache() const;
	};

	class HMDCamera : public Camera
	{
		// Stereo Settings
		void EnableStereo(const std::shared_ptr<Devices::OculusRift>& pRift);
		void DisableStoreo();
		bool IsStereoEnabled() const;

		// Inherited via ICameraRenderControl
		virtual void BeginFrame(RenderContext& context) override;
		virtual void EndFrame() override;
		// Only need for Stereo or more camera
		virtual void SetView(size_t view = 0) override;

		// This ignore the effect of Oculus
		virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) override;

		// Inherited via ICamera
		virtual size_t ViewCount() const override;
		virtual DirectX::XMMATRIX		GetViewMatrix(size_t view) const override;
		virtual DirectX::XMMATRIX		GetProjectionMatrix(size_t view) const override;
		const BoundingFrustum&			GetViewFrustum(size_t view) const;
		virtual bool XM_CALLCONV		IsInView(DirectX::FXMVECTOR pos, size_t view = 0) const override;

	private:
		std::shared_ptr<Devices::OculusRift>		m_pRift;
	};
}
