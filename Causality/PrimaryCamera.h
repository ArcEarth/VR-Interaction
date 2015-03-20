#pragma once
#include "OculusRift.h"
#include "Component.h"
#include "RenderContext.h"

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
	class ICameraBase abstract
	{
	public:
		virtual ~ICameraBase()
		{}
		virtual size_t ViewCount() const = 0;
		virtual DirectX::XMMATRIX GetViewMatrix(size_t view = 0) const = 0;
		virtual DirectX::XMMATRIX GetProjectionMatrix(size_t view = 0) const = 0;
		virtual void FocusAt(DirectX::FXMVECTOR focusPoint, DirectX::FXMVECTOR upDir) = 0;
		DirectX::XMMATRIX GetViewProjectionMatrix(size_t view = 0) const
		{
			DirectX::XMMATRIX mat = GetViewMatrix(view);
		}
		virtual bool XM_CALLCONV IsInView(DirectX::FXMVECTOR pos, size_t view = 0) const = 0;
	};

	// Control the logic of render target setup and post-processing needed for current camera
	class ICameraRenderControl abstract
	{
	public:
		// Called in the beginning of per-frame
		virtual void BeginFrame() = 0;
		// Called in in the end of per frame, should call Prenset inside
		virtual void EndFrame() = 0;
		// Called in advance of per-view rendering
		virtual void SetView(size_t view = 0) = 0;

		//virtual RenderTarget& GetRenderTarget(size_t view = 0) = 0;
		//virtual RenderTarget& SetRenderTarget(size_t view = 0) = 0;
	};

	// A Camera setup which works both in the case with OculusRift and Normal Monolith Camera
	class Camera : virtual public SceneObject, public ICameraBase, public ICameraRenderControl
	{
	public:
		Camera(const RenderContext& context = nullptr);

		~Camera();

		DirectX::RenderTarget& GetRenderTarget(int view = 0);
		bool SetRenderTarget(DirectX::RenderTarget& renderTarget, int view = 0);
		bool SetRenderTarget(DirectX::RenderTarget&& renderTarget, int view = 0);

		void SetRenderContext(const RenderContext& context);

		// Stereo Settings
		void EnableStereo(const std::shared_ptr<Devices::OculusRift>& pRift);
		void DisableStoreo();
		bool IsStereoEnabled() const;

		// Inherited via ICameraRenderControl
		virtual void BeginFrame() override;
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

		// Inherited via ICameraParameters
		void SetPerspective(float fovRadius, float aspectRatioHbyW);
		void SetOrthographic(float viewWidth, float viewHeight);

		float	GetFov() const;
		float	GetNear() const;
		float	GetFar() const;
		float	GetAspectRatio() const;
		void	SetNear(float _near);
		void	SetFar(float _far);

		virtual Vector3		GetPosition() const override;
		virtual void		SetPosition(const Vector3& p) override;
		virtual Quaternion	GetOrientation() const override;
		virtual void		SetOrientation(const Quaternion &q) override;

		void XM_CALLCONV	Move(DirectX::FXMVECTOR p);
		void XM_CALLCONV	Rotate(DirectX::FXMVECTOR q);

		Color										Background;

		static const DirectX::XMVECTORF32 Foward, Up;
	private:
		float _near, _far, _left, _right, _up, _down;
		float Fov, AspectRatio;
		bool  dirty;

		BoundingFrustum								m_ViewFrutum;
		DirectX::RenderTarget						m_RenderTarget;
		RenderContext								m_pRenderContext;
		std::shared_ptr<Devices::OculusRift>		m_pRift;
	};
}
