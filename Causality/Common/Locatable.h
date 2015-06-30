#pragma once
#include "DirectXMathExtend.h"

namespace DirectX
{
	// Interface for Object with 3D Position
	class ILocatable abstract
	{
	public:
		virtual Vector3 GetPosition() const = 0;
		virtual void  SetPosition(const Vector3 &p) = 0;
	};

	// Interface for 3D-Oriented-Object
	class IOriented abstract
	{
	public:
		virtual Quaternion GetOrientation() const = 0;
		virtual void  SetOrientation(const Quaternion &q) = 0;
	};

	// Interface for Local-Bounding Box
	// Scale in (x,y,z) is equal to the extends of it's bouding box
	class IScalable abstract
	{
	public:
		virtual Vector3 GetScale() const = 0;
		virtual void SetScale(const Vector3& s) = 0;
	};

	// Interface for collisition detection (Bouding Geometries)
	// Why reference? It's good to also keep the bounding geometries up to date!
	class IBoundable abstract
	{
	public:
		//virtual BoundingSphere		GetBoundingSphere() const = 0;

		// Efficent interface, but less friendly
		//virtual HRESULT GetBoundingBox(BoundingBox& out) const = 0;
		//virtual const Vector3& GetExtent() const = 0;

		virtual BoundingBox			GetBoundingBox() const = 0;
		virtual BoundingOrientedBox GetOrientedBoundingBox() const {
			auto box = GetBoundingBox();
			return BoundingOrientedBox(box.Center, box.Extents, Quaternion::Identity);
		};
	};

	// Interface for object with the ability to Move / Rotate / Isotropic-Scale
	class IRigid abstract : virtual public ILocatable, virtual public IOriented, virtual public IScalable
	{
	public:
		void XM_CALLCONV Move(FXMVECTOR p) { SetPosition(XMVectorAdd((XMVECTOR)GetPosition(), p)); }
		void XM_CALLCONV Rotate(FXMVECTOR q) { SetOrientation(XMQuaternionMultiply(GetOrientation(), q)); }
		XMMATRIX GetRigidTransformMatrix() const
		{
			return XMMatrixAffineTransformation(GetScale(), XMVectorZero(), GetOrientation(), GetPosition());
		}
	};

	class BasicTransform : public AffineTransform, virtual public IRigid
	{
	public:
		// Inherited via IRigid
		virtual Vector3 GetPosition() const
		{
			return AffineTransform::Translation;
		}
		virtual void SetPosition(const Vector3 &p) override
		{
			AffineTransform::Translation = p;
		}
		virtual Quaternion GetOrientation() const override
		{
			return AffineTransform::Rotation;
		}
		virtual void SetOrientation(const Quaternion &q) override
		{
			AffineTransform::Rotation = q;
		}
		virtual Vector3 GetScale() const override
		{
			return AffineTransform::Scale;
		}
		virtual void SetScale(const Vector3 & s) override
		{
			AffineTransform::Scale = s;
		}

		//Vector3 GetForward() const;
		//Vector3 GetUp() const;
		//Vector3 GetRight() const;
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
}