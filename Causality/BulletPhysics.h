#pragma once
#include "Common\Locatable.h"
#include "Common\Model.h"
#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>


namespace Causality
{
	template <class _TTarget, class _TSource>
	const _TTarget& force_cast(const _TSource& v)
	{
		static_assert(sizeof(_TSource) >= sizeof(_TTarget), "Vector dimension must agree.");
		return reinterpret_cast<const _TTarget&>(v);
	}

	template <class _TTarget, class _TSource>
	_TTarget& force_cast(_TSource& v)
	{
		static_assert(sizeof(_TSource) >= sizeof(_TTarget), "Vector dimension must agree.");
		return reinterpret_cast<_TTarget&>(v);
	}

	template <class _TTarget, class _TSource>
	_TTarget force_cast(_TSource&& v)
	{
		static_assert(sizeof(_TSource) >= sizeof(_TTarget), "Vector dimension must agree.");
		return reinterpret_cast<_TTarget&&>(std::move(v));
	}

	template <class _T>
	using wrap_simd_t = std::conditional_t<std::alignment_of<_T>::value == std::alignment_of<float>::value, const _T&, _T>;

	template <class _TTarget, class _TSource>
	inline wrap_simd_t<_TTarget> vector_cast(const _TSource& v)
	{
		return force_cast<_TTarget>(v);
	}
	template <>
	FORCEINLINE wrap_simd_t<btVector3> vector_cast<btVector3, DirectX::XMVECTOR>(const DirectX::XMVECTOR& v)
	{
		btVector3 btv;
		btv.mVec128 = v;
		return btv;
	}

	template <>
	inline wrap_simd_t<btVector3> vector_cast<btVector3, DirectX::Vector3>(const DirectX::Vector3& v)
	{
		return btVector3(v.x, v.y, v.z);
	}

	template <>
	inline wrap_simd_t<btVector4> vector_cast<btVector4, DirectX::Vector4>(const DirectX::Vector4& v)
	{
		return btVector4(v.x, v.y, v.z, v.w);
	}

	template <>
	inline wrap_simd_t<btQuaternion> vector_cast<btQuaternion, DirectX::Quaternion>(const DirectX::Quaternion& v)
	{
		return btQuaternion(v.x,v.y,v.z,v.w);
	}

	template <>
	inline wrap_simd_t<btVector3> vector_cast<btVector3, DirectX::XMFLOAT3>(const DirectX::XMFLOAT3& v)
	{
		return btVector3(v.x,v.y,v.z);
	}

	template <>
	inline wrap_simd_t<btVector3> vector_cast<btVector3, DirectX::XMFLOAT4>(const DirectX::XMFLOAT4& v)
	{
		return btVector4(v.x, v.y, v.z, v.w);
	}

	template <>
	inline wrap_simd_t<btQuaternion> vector_cast<btQuaternion, DirectX::XMFLOAT4>(const DirectX::XMFLOAT4& v)
	{
		return btQuaternion(v.x, v.y, v.z, v.w);
	}

	class PhysicalRigid : virtual public DirectX::IRigid
	{
	public:
		PhysicalRigid();

		~PhysicalRigid();

		bool Disable();
		bool Enable(const std::shared_ptr<btDynamicsWorld> &pWorld = nullptr);
		bool IsEnabled() const;

		btRigidBody* GetBulletRigid() { return m_pRigidBody.get(); }
		btCollisionShape* GetBulletShape() { return m_pShape.get(); }
		const btRigidBody* GetBulletRigid() const { return m_pRigidBody.get(); }
		const btCollisionShape* GetBulletShape() const { return m_pShape.get(); }

		void InitializePhysics(const std::shared_ptr<btDynamicsWorld> &pWorld, const std::shared_ptr<btCollisionShape>& pShape, float mass, const DirectX::Vector3 & Pos = DirectX::Vector3::Zero, const DirectX::Quaternion & Rot = DirectX::Quaternion::Identity);

		// Inherited via IRigid
		virtual const DirectX::Vector3 & GetPosition() const override;
		virtual void SetPosition(const DirectX::Vector3 & p) override;
		virtual const DirectX::Quaternion & GetOrientation() const override;
		virtual void SetOrientation(const DirectX::Quaternion & q) override;
		virtual const DirectX::Vector3 & GetScale() const override;
		virtual void SetScale(const DirectX::Vector3 & s) override;

	protected:
		mutable btTransform					m_Transform;
		btTransform							m_MassCenterTransform;
		bool								m_IsEnabled;
		std::shared_ptr<btDynamicsWorld>	m_pWorld;
		std::unique_ptr<btRigidBody>		m_pRigidBody;
		std::shared_ptr<btCollisionShape>	m_pShape;
		mutable DirectX::Vector3					Scale;
		mutable DirectX::Vector3					Position;
		mutable DirectX::Quaternion					Orientation;
	};

	//class PhysicalGeometryModel : public DirectX::Scene::IGeometryModel, public DirectX::Scene::IRigidLocalMatrix , public PhysicalRigid
	//{
	//	void InitializePhysicalRigid(float mass = 1.0f);
	//	static void CreateFromObjFile();
	//};
}