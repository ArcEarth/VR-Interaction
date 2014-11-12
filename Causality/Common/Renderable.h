#pragma once
#include "DeviceResources.h"
#include "StepTimer.h"
#include "BasicClass.h"
#include "Locatable.h"

namespace DirectX{
	namespace Scene
	{
		// Interface for Rendering
		class IRenderable abstract
		{
		public:
			virtual void Render(DeviceResources *pDeviceResources) = 0;
		};

		// Interface for setting View/Projection Matrix
		// Represent a 3D object that depend on Camera view and 3D location
		class IViewable abstract
		{
		public:
			virtual void UpdateViewMatrix(DirectX::CXMMATRIX view) = 0;
			virtual void UpdateProjectionMatrix(DirectX::CXMMATRIX projection) = 0;
		};

		// Interface for Time-dependent Animation
		class ITimeAnimatable abstract
		{
		public:
			virtual void UpdateAnimation(StepTimer const& timer) = 0;
		};

		// Interface for object with local coordinate
		class ILocalCoordinate abstract
		{
		public:
			virtual void SetModelMatrix(DirectX::CXMMATRIX model) = 0;
			virtual XMMATRIX GetModelMatrix() const = 0;

			virtual void TransformLocal(DirectX::CXMMATRIX trans)
			{
				SetModelMatrix(trans * GetModelMatrix());
			}

			virtual void TransformGlobal(DirectX::CXMMATRIX trans)
			{
				SetModelMatrix(GetModelMatrix() * trans);
			}
		};

		class IRigidCoordinate abstract : public ILocalCoordinate, public IRigid
		{
		public:
			void Move(FXMVECTOR p)         { SetPosition((XMVECTOR)Position() + p); }
			void Rotate(FXMVECTOR q)    { SetOrientation(XMQuaternionMultiply(q,Orientation())); }

			virtual XMMATRIX GetModelMatrix() const override
			{
				return XMMatrixAffineTransformation(XMVectorReplicate(Scale()), XMVectorZero(), Orientation(), Position());
			}
		protected:
			virtual void SetModelMatrix(DirectX::CXMMATRIX model) override
			{
				XMVECTOR scale, rotation, translation;
				XMMatrixDecompose(&scale, &rotation, &translation, model);
				SetScale(XMVectorGetX(scale));
				SetOrientation(rotation);
				SetPosition(translation);
			}
		};
	}

}