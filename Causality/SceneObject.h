#pragma once
#include "Common\Model.h"
#include "BulletPhysics.h"

namespace Causality
{
	typedef uint64_t SceneObjectID;
	enum SceneObjectInteractionType
	{
		DynamicType,	// Passive, Collision with Static and Kinametic Object
		StaticType,		// Passive, Collision with Dynamic Type, won't move
		KinameticType,	// Active, Collision with Dynamic Type, will move
		GhostType,		// Active, Don't Collide, but will move
	};

	struct RenderingSpecification // Shadow,Bloom,etc
	{
		bool IsDropShadow;
		bool IsRecieveShadow;
		bool IsVisiable;
		bool IsBlooming;
	};

	struct ProblistiscAffineTransform : public DirectX::AffineTransform
	{
		using DirectX::AffineTransform::AffineTransform;
		float Probability;
	};

	typedef std::vector<ProblistiscAffineTransform> SuperPosition;

	class SceneObject
	{
		std::string	Name;
		SceneObjectID ID;
		RenderingSpecification RenderSpec;
		std::shared_ptr<DirectX::Scene::IModelNode> m_pRenderModel;
		std::shared_ptr<Bullet::CollisionShape>		m_CollisionShape;
	};
}