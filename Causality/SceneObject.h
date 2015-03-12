#pragma once
#include "Common\Model.h"
#include "BulletPhysics.h"
#include "Armature.h"
#include <memory>

namespace Causality
{


	enum SceneObjectCollisionType
	{
		Collision_Dynamic,	// Passive, Collision with Static and Kinametic Object
		Collision_Static,		// Passive, Collision with Dynamic Type, won't move
		Collision_Kinametic,	// Active, Collision with Dynamic Type, will move
		Collision_Ghost,		// Active, Don't Collide, but will move
	};

	enum SceneObjectType
	{
		Entity,
		Camera,
		Light,
		Misc,
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

	// Basic class for all object, camera, entity, or light
	class SceneObject : public Object, public DirectX::BasicTransform
	{
	public:
		std::string							Tag;

		SceneObject*						Parent;
		const std::vector<SceneObject*>&	Children;
		const std::vector<Component*>&		Components;

		template <typename TComponent,typename... TArgs>
		TComponent&							AddComponent(TArgs&&... args)
		{
			m_Components.push_back(new TComponent(std::forward(args)));
			m_Components.back()->Owner = this;
		}

		template <typename TComponent>
		const TComponent*					GetComponent() const 
		{
			return nullptr;
		}

		bool								IsStatic() const;
		bool								IsActive() const;

		bool								SetActive(bool active = true);

		DirectX::XMMATRIX					GlobalTransformMatrix() const;

		TypedEvent<SceneObject>				OnLoaded;
		//TypedEvent<SceneObject>			OnUnloaded;
	protected:
		std::vector<SceneObject*>			m_Children;
		std::vector<Component*>				m_Components;
	};

	class CameraObject : public SceneObject
	{

	};

	class LightingObject : public SceneObject
	{

	};

	// Scene object acts like entities for render
	class SceneEntity : public SceneObject
	{
	public:
		//int										MaxLoD() const;
		//int										CurrentLoD() const;
		//void										SetLoD(int LoD);

		void										Hide();
		void										Show();

		bool										IsFreezed() const;
		void										Freeze();
		void										Unfreeze();

		bool										IsVisible() const;
		bool										IsFocused() const;

		// Events
		//TypedEvent<SceneEntity, int>				LoDChanged;
		//TypedEvent<SceneEntity>						Rendering;
		//TypedEvent<SceneEntity>						Rendered;
		//TypedEvent<SceneEntity>						Focused;
		//TypedEvent<SceneEntity>						FocusLost;
		//TypedEvent<SceneEntity>						VisibilityChanged;
		//TypedEvent<SceneEntity>						PositionChanged;

		DirectX::Scene::IModelNode*					RenderModel(int LoD = 0);
		const DirectX::Scene::IModelNode*			RenderModel(int LoD = 0) const;

		Bullet::CollisionShape&						CollisionShape(int LoD = 0);
		const Bullet::CollisionShape&				CollisionShape(int LoD = 0) const;

	protected:
		RenderingSpecification						RenderSpec;

		std::shared_ptr<DirectX::Scene::IModelNode> m_pRenderModel;

		std::shared_ptr<Bullet::CollisionShape>		m_CollisionShape;
	};

	struct ColorHistogram
	{
	};

	struct KinematicSceneObjectPart
	{
		int								ID; // Must be same as the id referenced in Armature

		// Saliency parameters
		Color				AverageColor;
		ColorHistogram		ColorDistribution;
		Vector3				PositionDistribution;
		BoundingBox			VelocityDistribution;
		
		float							IntrinsicSaliency;
		float							ExtrinsicSaliency;
	};


	class ControlState;
	// Represent an SceneObjecy
	class KinematicSceneObject : public SceneEntity
	{
	public:
		typedef AnimationSpace::frame_type frame_type;

		const frame_type&				GetCurrentFrame() const;
		frame_type&						MapCurrentFrameForUpdate();
		void							ReleaseCurrentFrameFrorUpdate();

		IArmature&						Armature();
		const IArmature&				Armature() const;
		AnimationSpace&					Space();
		const AnimationSpace&			Space() const;

	private:
		sptr<IArmature>							m_pArmature;
		sptr<AnimationSpace>					m_pAnimationSpace;
		const frame_type&						m_DefaultFrame;
		frame_type								m_CurrentFrame;

		std::vector<KinematicSceneObjectPart>	m_Parts;

		bool									m_DirtyFlag;
		std::vector<ControlState*>				m_Controls;
	};

	// Represent an ControlObject
	class KinematicAIController
	{

	};

	class PlayerController
	{
	public:
		class ControlState
		{
		public:

			const ArmatureTransform& Binding() const;
			ArmatureTransform& Binding();

			const KinematicSceneObject& Object() const;
			KinematicSceneObject& Object();

			int						ID;
			BoneDisplacementFrame	PotientialFrame;

			KinematicSceneObject&	m_SceneObject;
			ArmatureTransform		m_Binding;
		};

		struct StateChangedEventArgs
		{
			int OldStateIndex;
			int NewStateIndex;
			float Confidence;
			ControlState& OldState;
			ControlState& NewState;
		};

		bool IsIdel() const;
		const ControlState& CurrentState() const;
		ControlState& CurrentState();
		const ControlState&	GetState(int state) const;

		Event<const StateChangedEventArgs&> StateChanged;

		bool	UpdatePlayerFrame(const BoneDisplacementFrame& frame);

		IArmature&					PlayerArmature;

	protected:
		int							CurrentIdx;
		std::vector<ControlState>	States;

		VectorX						StateProbality;
		VectorX						Likilihood;
		MatrixX						TransferMatrix;
	};

	class KinematicSceneObjectController
	{

	};
}