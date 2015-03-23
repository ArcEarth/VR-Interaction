#pragma once
#include "Common\Model.h"
//#include "BulletPhysics.h"
#include "Armature.h"
#include <memory>
#include "Object.h"
#include "RenderContext.h"
#include "Interactive.h"

namespace Causality
{
	class Component;

	class IRenderable abstract
	{
	public:
		// Camera culling
		virtual bool IsVisible(const BoundingFrustum& viewFrustum) const = 0;
		virtual void Render(RenderContext &context) = 0;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) = 0;
	};

	// Interface for Time-dependent Animation
	class ITimeAnimatable abstract
	{
	public:
		virtual void UpdateAnimation(time_seconds const& time_delta) = 0;
	};


	enum SceneObjectCollisionType
	{
		Collision_Dynamic,	// Passive, Collision with Static and Kinametic Object
		Collision_Static,		// Passive, Collision with Dynamic Type, won't move
		Collision_Kinametic,	// Active, Collision with Dynamic Type, will move
		Collision_Ghost,		// Active, Don't Collide, but will move
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
	class SceneObject : public tree_node<SceneObject>, public Object, public DirectX::BasicTransform
	{
	public:
		typedef tree_node<SceneObject> tree_base_type;

		SceneObject() = default;

		inline void AddChild(SceneObject* child)
		{
			if (child != nullptr)
				append_children_back(child);
		}

		inline void add_child(SceneObject* child)
		{
			if (child != nullptr)
				append_children_back(child);
		}

		template <typename T>
		T* FirstAncesterOfType()
		{
			auto node = parent();
			while (node && !node->Is<T>())
				node = node->parent();
			if (node)
				return node->As<T>();
			else
				return nullptr;
		}

		//template <typename TInterface>
		//auto descendents_of_type() 
		//{
		//	using namespace boost;
		//	using namespace adaptors;
		//	return 
		//		descendants()
		//		| transformed([](auto pCom) {
		//			return dynamic_cast<TInterface*>(pCom); }) 
		//		| filtered([](auto pCom){
		//			return pCom != nullptr;});
		//}

		virtual void Update() {}

		bool								IsStatic() const { return m_IsStatic; }

		bool								IsEnabled() const;
		bool								SetEnabled(bool enabled = true);

		DirectX::XMMATRIX					GlobalTransformMatrix() const;

		virtual void						SetPosition(const Vector3 &p) override;
		virtual void                        SetOrientation(const Quaternion &q) override;
		virtual void                        SetScale(const Vector3 & s) override;
		void								SetMatrixDirtyFlag() { m_GlobalTransformDirty = true; }

	public:
		string								Tag;
	protected:
		Matrix4x4							m_GlobalTransform;
		bool								m_GlobalTransformDirty;
		bool								m_IsStatic;
	};

	class LightingObject : public SceneObject
	{

	};

	// Scene object acts like entities for render
	class RenderableSceneObject : virtual public SceneObject , virtual public IRenderable
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

		virtual bool IsVisible(const BoundingFrustum& viewFrustum) const override;
		bool										IsVisible() const;
		bool										IsFocused() const;

		// Events
		//TypedEvent<RenderableSceneObject, int>				LoDChanged;
		//TypedEvent<RenderableSceneObject>						Rendering;
		//TypedEvent<RenderableSceneObject>						Rendered;
		//TypedEvent<RenderableSceneObject>						Focused;
		//TypedEvent<RenderableSceneObject>						FocusLost;
		//TypedEvent<RenderableSceneObject>						VisibilityChanged;
		//TypedEvent<RenderableSceneObject>						PositionChanged;

		DirectX::Scene::IModelNode*					RenderModel(int LoD = 0);
		const DirectX::Scene::IModelNode*			RenderModel(int LoD = 0) const;
		void										SetRenderModel(DirectX::Scene::IModelNode* pMesh, int LoD = 0);

		//Bullet::CollisionShape&						CollisionShape(int LoD = 0);
		//const Bullet::CollisionShape&				CollisionShape(int LoD = 0) const;

		// Inherited via IRenderable
		virtual void Render(RenderContext & pContext) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

	protected:
		RenderingSpecification						RenderSpec;

		std::shared_ptr<DirectX::Scene::IModelNode> m_pRenderModel;

		//std::shared_ptr<Bullet::CollisionShape>		m_CollisionShape;
	};

	struct ColorHistogram
	{
	};

	struct KinematicSceneObjectPart
	{
		int								ID; // Must be same as the id referenced in Armature

		// Saliency parameters
		Color							AverageColor;
		ColorHistogram					ColorDistribution;
		Vector3							PositionDistribution;
		BoundingBox						VelocityDistribution;
		
		float							IntrinsicSaliency;
		float							ExtrinsicSaliency;
	};

	class ControlState;
	// Represent an SceneObjecy
	class KinematicSceneObject : public RenderableSceneObject
	{
	public:
		typedef AnimationSpace::frame_type frame_type;

		const frame_type&				GetCurrentFrame() const;
		frame_type&						MapCurrentFrameForUpdate();
		void							ReleaseCurrentFrameFrorUpdate();

		IArmature&						Armature();
		const IArmature&				Armature() const;
		AnimationSpace&					Behavier();
		const AnimationSpace&			Behavier() const;
		void							SetBehavier(AnimationSpace& behaver);

	private:
		AnimationSpace*					        m_pAnimationSpace;
		int										m_FrameMapState;
		frame_type						        m_CurrentFrame;

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

			const ArmatureTransform& Binding() const { return m_Binding; }
			ArmatureTransform& Binding() { return m_Binding; }

			const KinematicSceneObject& Object() const { return *m_pSceneObject; }
			KinematicSceneObject& Object() { return *m_pSceneObject; }
			void SetTargetObject(KinematicSceneObject& object) { 
				m_pSceneObject = &object;
				m_Binding.SetTargetArmature(object.Armature());
				PotientialFrame = object.Armature().default_frame();
			}

			int						ID;
			BoneDisplacementFrame	PotientialFrame;

		private:
			KinematicSceneObject*	m_pSceneObject;
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

		bool IsIdel() const { return CurrentIdx == 0; }
		const ControlState& CurrentState() const { return States[CurrentIdx]; }
		ControlState& CurrentState() { return States[CurrentIdx]; }
		const ControlState&	GetState(int state) const { return States[state]; }

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

	class KeyboardMouseFirstPersonControl :public SceneObject, public IAppComponent, public ITimeAnimatable, public IKeybordInteractive, public ICursorInteractive
	{
	public:
		KeyboardMouseFirstPersonControl(IRigid* pTarget = nullptr);

		void SetTarget(IRigid* pTarget);

		// Inherited via ITimeAnimatable
		virtual void UpdateAnimation(time_seconds const& time_delta) override;

		// Inherited via IKeybordInteractive
		virtual void OnKeyDown(const KeyboardEventArgs & e) override;
		virtual void OnKeyUp(const KeyboardEventArgs & e) override;

		// Inherited via ICursorInteractive
		virtual void OnMouseButtonDown(const CursorButtonEvent & e) override;
		virtual void OnMouseButtonUp(const CursorButtonEvent & e) override;
		virtual void OnMouseMove(const CursorMoveEventArgs & e) override;
	public:
		float											Speed;
		float											AngularSpeed;

		DirectX::Quaternion								InitialOrientation;

		float											AddationalYaw = 0;
		float											AddationalPitch = 0;
		float											AddationalRoll = 0;

	private:
		IRigid*											m_pTarget = nullptr;

		bool											IsTrackingCursor = false;
		DirectX::Vector3								CameraVeclocity;
		DirectX::Vector3								CameraAngularVeclocity;
		DirectX::Vector3								AngularDisplacement; // Eular Angle
	};


	class KinematicSceneObjectController
	{

	};
}