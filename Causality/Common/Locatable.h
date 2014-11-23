#pragma once
#include "DirectXMathExtend.h"

namespace DirectX
{

	// Interface for Object with 3D Position
	class ILocatable abstract
	{
	public:
		virtual const Vector3&    Position() const = 0;
		virtual void  SetPosition(const Vector3 &p) = 0;
	};

	// Interface for 3D-Oriented-Object
	class IOriented abstract
	{
	public:
		virtual const Quaternion& Orientation() const = 0;
		virtual void  SetOrientation(const Quaternion &q) = 0;
	};

	// Interface for Local-Bounding Box
	// Scale in (x,y,z) is equal to the extends of it's bouding box
	class IScalable abstract
	{
	public:
		virtual const Vector3& Scale() const = 0;
		virtual void SetScale(const Vector3& s) = 0;
	};

	// Interface for object with the ability to Move / Rotate / Isotropic-Scale / Boundarize
	class IRigid abstract : public ILocatable, public IOriented, public IScalable
	{
	public:
		virtual const BoundingOrientedBox& GetOrientedBoundingBox() = 0;
		virtual BoundingBox GetBoundingBox() = 0;
	};

	struct Rigid : public IRigid
	{
	public:
		Rigid()
		{
			Bound.Center = Vector3();
			Bound.Orientation = Quaternion();
			Bound.Extents = Vector3(1.f, 1.f, 1.f);
		}

		// Inherited via IRigid
		virtual const Vector3 & Position() const
		{
			return static_cast<const Vector3&>(Bound.Center);
		}
		virtual void SetPosition(const Vector3 &p) override
		{
			Bound.Center = p;
		}
		virtual const Quaternion & Orientation() const override
		{
			return static_cast<const Quaternion&>(Bound.Orientation);
		}
		virtual void SetOrientation(const Quaternion &q) override
		{
			Bound.Orientation = q;
		}
		virtual const Vector3 & Scale() const override
		{
			return static_cast<const Vector3&>(Bound.Extents);
		}
		virtual void SetScale(const Vector3 & s) override
		{
			Bound.Extents = s;
		}
		virtual const BoundingOrientedBox & GetOrientedBoundingBox() override
		{
			return Bound;
		}
		virtual BoundingBox GetBoundingBox() override {
			BoundingBox box;
			Vector3 corners[BoundingOrientedBox::CORNER_COUNT];
			Bound.GetCorners(corners);
			BoundingBox::CreateFromPoints(box, BoundingOrientedBox::CORNER_COUNT, corners, sizeof(Vector3));
			return box;
		}

	public:
		BoundingOrientedBox Bound;
	};

}