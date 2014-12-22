#pragma once
#include "DeviceResources.h"
#include "StepTimer.h"
#include "DirectXMathExtend.h"
#include "Locatable.h"
#include <DirectXCollision.h>

namespace DirectX{
	namespace Scene
	{
		// Interface for Rendering
		class IRenderable abstract
		{
		public:
			virtual void Render(ID3D11DeviceContext *pContext) = 0;
		};

		// Interface for setting View/Projection Matrix
		// Represent a 3D object that depend on Camera view and 3D location
		class IViewable abstract
		{
		public:
			virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view) = 0;
			virtual void XM_CALLCONV UpdateProjectionMatrix(DirectX::FXMMATRIX projection) = 0;
		};

		// Interface for Time-dependent Animation
		class ITimeAnimatable abstract
		{
		public:
			virtual void UpdateAnimation(StepTimer const& timer) = 0;
		};

		// Interface for object with local coordinate
		class ILocalMatrix abstract
		{
		public:
			virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) = 0;
			virtual XMMATRIX GetModelMatrix() const = 0;

			void XM_CALLCONV TransformLocal(DirectX::FXMMATRIX trans)
			{
				SetModelMatrix(trans * GetModelMatrix());
			}

			void XM_CALLCONV TransformGlobal(DirectX::FXMMATRIX trans)
			{
				SetModelMatrix(GetModelMatrix() * trans);
			}
		};

		// The object Using rigid information to genreate ILocalMatrix interface
		struct IRigidLocalMatrix : virtual public ILocalMatrix, virtual public IRigid
		{
		public:
			virtual XMMATRIX GetModelMatrix() const override
			{
				return XMMatrixAffineTransformation(GetScale(), XMVectorZero(), GetOrientation(), GetPosition());
			}
		protected:
			virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override
			{
				XMVECTOR scale, rotation, translation;
				XMMatrixDecompose(&scale, &rotation, &translation, model);
				SetScale((Vector3)scale);
				SetOrientation(rotation);
				SetPosition(translation);
			}
		};

		// helper struct that implements IViewable interface and stored matrix for future use
		struct ViewMatrixCache : virtual public IViewable
		{
		public:
			// Inherited via IViewable
			virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view) override
			{
				ViewMatrix = view;
			}
			virtual void XM_CALLCONV UpdateProjectionMatrix(DirectX::FXMMATRIX projection) override
			{
				ProjectionMatrix = projection;
			}
			Matrix4x4	ViewMatrix;
			Matrix4x4	ProjectionMatrix;
		};
		
		// An brige class to transfer Rigid data into model matrix
		class RigidLocalMatrix : virtual public ILocalMatrix
		{
		public:
			inline RigidLocalMatrix(IRigid* pRigid)
				: m_pRigid(pRigid)
			{
				assert(pRigid != nullptr);
			}

			virtual XMMATRIX GetModelMatrix() const override
			{
				return XMMatrixAffineTransformation(m_pRigid->GetScale(), XMVectorZero(), m_pRigid->GetOrientation(), m_pRigid->GetPosition());
			}
		protected:
			virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override
			{
				XMVECTOR scale, rotation, translation;
				XMMatrixDecompose(&scale, &rotation, &translation, model);
				m_pRigid->SetScale((Vector3) scale);
				m_pRigid->SetOrientation(rotation);
				m_pRigid->SetPosition(translation);
			}
		private:
			IRigid * m_pRigid;
		};

	}

}