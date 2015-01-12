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
		void XM_CALLCONV Move(FXMVECTOR p) { SetPosition(XMVectorAdd((XMVECTOR) GetPosition(), p)); }
		void XM_CALLCONV Rotate(FXMVECTOR q) { SetOrientation(XMQuaternionMultiply(GetOrientation(),q)); }
		XMMATRIX GetRigidTransformMatrix() const
		{
			return XMMatrixAffineTransformation(GetScale(), XMVectorZero(), GetOrientation(), GetPosition());
		}
	};

	// Composition of Translation and Rotation
	struct RigidTransform
	{
	public:
		Quaternion  Rotation;
		Vector3		Translation;

		RigidTransform()
		{}

		explicit RigidTransform(IRigid* pRigid)
			: Rotation(pRigid->GetOrientation()), Translation(pRigid->GetPosition())
		{}

		// Extract the Matrix Representation of this rigid transform
		inline XMMATRIX TransformMatrix() const
		{
			XMMATRIX M = XMMatrixRotationQuaternion(Rotation);
			XMVECTOR VTranslation = XMVectorSelect(g_XMSelect1110.v, Translation, g_XMSelect1110.v);
			M.r[3] = XMVectorAdd(M.r[3], VTranslation);
			return M;
		}

		// Extract the Dual-Quaternion Representation of this rigid transform
		inline XMDUALVECTOR TransformDualQuaternion() const
		{
			return XMDualQuaternionRotationTranslation(Rotation, Translation);
		}

		bool NearEqual (const RigidTransform& rhs,float tEpsilon = 0.002f, float rEpsilon = 0.5f) const
		{
			Vector3 PosDiff = Translation - rhs.Translation;
			XMVECTOR RotDiff = Rotation;
			RotDiff = XMQuaternionInverse(RotDiff);
			RotDiff = XMQuaternionMultiply(RotDiff,rhs.Rotation);
			float AngDiff = 2 * acosf(XMVectorGetW(RotDiff));
			return (PosDiff.LengthSquared() <= tEpsilon*tEpsilon && AngDiff <= rEpsilon);
		}
	};

	// Composition of Translation/Rotation/Scale
	struct AffineTransform : public RigidTransform
	{
		static AffineTransform Identity()
		{
			return AffineTransform();
		}

		Vector3 Scale;

		AffineTransform()
			: Scale(1.0f)
		{}

		explicit AffineTransform(IRigid* pRigid)
			: RigidTransform(pRigid), Scale(pRigid->GetScale())
		{}

		inline explicit AffineTransform(CXMMATRIX transform)
		{
			FromTransformMatrix(transform);
		}

		inline void XM_CALLCONV FromTransformMatrix(FXMMATRIX transform)
		{
			XMVECTOR scl, rot, tra;
			XMMatrixDecompose(&scl, &rot, &tra, transform);
			Scale = scl;
			Rotation = rot;
			Translation = tra;
		}

		inline XMMATRIX TransformMatrix() const
		{
			XMMATRIX M = XMMatrixScalingFromVector(Scale);
			M *= XMMatrixRotationQuaternion(Rotation);
			XMVECTOR VTranslation = XMVectorSelect(g_XMSelect1110.v, Translation, g_XMSelect1110.v);
			M.r[3] = XMVectorAdd(M.r[3], VTranslation);
			return M;
		}

		explicit operator XMMATRIX() const
		{
			return TransformMatrix();
		}
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