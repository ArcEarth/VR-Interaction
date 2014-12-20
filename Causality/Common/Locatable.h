#pragma once
#include "DirectXMathExtend.h"

namespace DirectX
{

	// Interface for Object with 3D Position
	class ILocatable abstract
	{
	public:
		virtual const Vector3& GetPosition() const = 0;
		virtual void  SetPosition(const Vector3 &p) = 0;
	};

	// Interface for 3D-Oriented-Object
	class IOriented abstract
	{
	public:
		virtual const Quaternion& GetOrientation() const = 0;
		virtual void  SetOrientation(const Quaternion &q) = 0;
	};

	// Interface for Local-Bounding Box
	// Scale in (x,y,z) is equal to the extends of it's bouding box
	class IScalable abstract
	{
	public:
		virtual const Vector3& GetScale() const = 0;
		virtual void SetScale(const Vector3& s) = 0;
	};

	// Interface for collisition detection (Bouding Geometries)
	// Why reference? It's good to also keep the bounding geometries up to date!
	class IBoundable abstract
	{
	public:
		virtual BoundingOrientedBox GetOrientedBoundingBox() const = 0;
		virtual BoundingBox			GetBoundingBox() const = 0;
		virtual BoundingSphere		GetBoundingSphere() const = 0;
	};

	// Interface for object with the ability to Move / Rotate / Isotropic-Scale
	class IRigid abstract : virtual public ILocatable, virtual public IOriented, virtual public IScalable
	{
	public:
		void XM_CALLCONV Move(FXMVECTOR p) { SetPosition((XMVECTOR) GetPosition() + p); }
		void XM_CALLCONV Rotate(FXMVECTOR q) { SetOrientation(XMQuaternionMultiply(GetOrientation(),q)); }
	};

	// Helper struct to implement IRigid Interface
	struct RigidBase : virtual public IRigid
	{
	public:
		RigidBase()
			: Position(),Orientation(),Scale(1.f, 1.f, 1.f)
		{
		}

		// Inherited via IRigid
		virtual const Vector3 & GetPosition() const
		{
			return Position;
		}
		virtual void SetPosition(const Vector3 &p) override
		{
			Position = p;
		}
		virtual const Quaternion & GetOrientation() const override
		{
			return Orientation;
		}
		virtual void SetOrientation(const Quaternion &q) override
		{
			Orientation = q;
		}
		virtual const Vector3 & GetScale() const override
		{
			return static_cast<const Vector3&>(Scale);
		}
		virtual void SetScale(const Vector3 & s) override
		{
			Scale = s;
		}

	public:
		Vector3		Position;
		Quaternion	Orientation;
		Vector3		Scale;
	};

}