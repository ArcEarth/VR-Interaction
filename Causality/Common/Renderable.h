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
			~IRenderable()
			{}
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

			virtual void XM_CALLCONV TransformLocal(DirectX::FXMMATRIX trans)
			{
				SetModelMatrix(trans * GetModelMatrix());
			}

			virtual void XM_CALLCONV TransformGlobal(DirectX::FXMMATRIX trans)
			{
				SetModelMatrix(GetModelMatrix() * trans);
			}
		};

		// The object Using rigid information to genreate ILocalMatrix interface
		class RigidObject : public RigidBase , public ILocalMatrix
		{
		public:
			virtual XMMATRIX GetModelMatrix() const override
			{
				return XMMatrixAffineTransformation(Scale(), XMVectorZero(), Orientation(), Position());
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
	}

}