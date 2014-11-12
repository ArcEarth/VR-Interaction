#pragma once
#include "BasicClass.h"

namespace DirectX
{
	namespace Scene
	{
	}
//}
//
//namespace 
//{

	// Interface for Object with 3D Position
	class ILocatable abstract
	{
	public:
		virtual const Platform::Fundation::Vector3&    Position() const = 0;
		virtual void  SetPosition(Platform::Fundation::Vector3 p) = 0;
	};

	// Interface for 3D-Oriented-Object
	class IOriented abstract
	{
	public:
		virtual const Platform::Fundation::Quaternion& Orientation() const = 0;
		virtual void  SetOrientation(Platform::Fundation::Quaternion q) = 0;
	};

	// Interface for Isotropic-Scaling
	class IScalable abstract
	{
	public:
		virtual float Scale() const = 0;
		virtual void  SetScale(float s) = 0;
	};

	class IAnisotropicScalable abstract
	{
	public:
		virtual Platform::Fundation::Vector3&Scale() const = 0;
		virtual void  SetScale(const Platform::Fundation::Vector3& s) = 0;
	};

	// Interface for object with the ability to Move / Rotate / Isotropic-Scale / Boundarize
	class IRigid abstract : public ILocatable, public IOriented, public IScalable
	{
	public:
		virtual DirectX::BoundingOrientedBox OrientedBoundingBox() = 0;
		virtual DirectX::BoundingBox BoundingBox() = 0;
	};
}