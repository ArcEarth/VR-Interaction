#pragma once
#include "Common\Model.h"
//#include "BulletPhysics.h"
#include <memory>
#include "Object.h"
#include "RenderContext.h"
#include "Interactive.h"
#include "Armature.h"
#include "CharacterBehavier.h"

namespace Causality
{
	class Scene;
	class Component;

	extern bool g_DebugView;

	class IRenderable abstract
	{
	public:
		// Camera culling
		virtual bool IsVisible(const BoundingFrustum& viewFrustum) const = 0;
		virtual void Render(RenderContext &context) = 0;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) = 0;
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

		virtual ~SceneObject() override;

		SceneObject();

		virtual void AddChild(SceneObject* child)
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

		virtual void						Update(time_seconds const& time_delta);

		bool								IsStatic() const { return m_IsStatic; }

		bool								IsEnabled() const { return m_IsEnabled; }
		bool								SetEnabled(bool enabled = true) { m_IsEnabled = enabled; }

		DirectX::XMMATRIX					GlobalTransformMatrix() const;

		virtual void						SetPosition(const Vector3 &p) override;
		virtual void                        SetOrientation(const Quaternion &q) override;
		virtual void                        SetScale(const Vector3 & s) override;
		void								SetMatrixDirtyFlag() { m_GlobalTransformDirty = true; }

	public:
		string								Name;
		string								Tag;
		Scene								*Scene;

	protected:
		Matrix4x4							m_GlobalTransform;
		bool								m_GlobalTransformDirty;
		bool								m_IsStatic;
		bool								m_IsEnabled;
	};

	class LightingObject : public SceneObject
	{

	};

	// Scene object acts like entities for render
	class RenderableSceneObject : virtual public SceneObject , virtual public IRenderable
	{
	public:
		RenderableSceneObject();
		//int										MaxLoD() const;
		//int										CurrentLoD() const;
		//void										SetLoD(int LoD);

		void										Hide() { m_isVisable = false; }
		void										Show() { m_isVisable = true; }

		virtual bool								IsVisible(const BoundingFrustum& viewFrustum) const override;
		bool										IsVisible() const { return m_isVisable; }
		bool										IsFocused() const {
			return m_isFocuesd;
		}

		float										Opticity() const { return m_opticity; }
		void										SetOpticity(float value) { m_opticity = value; }

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
		float										m_opticity;
		bool										m_isVisable;
		bool										m_isFocuesd;
		DirectX::Scene::IModelNode*					m_pRenderModel;

		//std::shared_ptr<Bullet::CollisionShape>		m_CollisionShape;
	};

	class CoordinateAxis : virtual public SceneObject, virtual public IRenderable
	{
		// Inherited via IRenderable
		virtual bool IsVisible(const BoundingFrustum & viewFrustum) const override;
		virtual void Render(RenderContext & context) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;
	};

	class CharacterController;
	// Represent an SceneObjecy
	class KinematicSceneObject : public RenderableSceneObject
	{
	public:
		typedef BehavierSpace::frame_type frame_type;

		const frame_type&				GetCurrentFrame() const;
		frame_type&						MapCurrentFrameForUpdate();
		void							ReleaseCurrentFrameFrorUpdate();

		IArmature&						Armature();
		const IArmature&				Armature() const;
		BehavierSpace&					Behavier();
		const BehavierSpace&			Behavier() const;
		void							SetBehavier(BehavierSpace& behaver);
		
		bool							StartAction(const string& key, time_seconds begin_time = time_seconds(0), bool loop = false, time_seconds transition_time = time_seconds(0));
		bool							StopAction(time_seconds transition_time = time_seconds(0));

		bool							IsFreezed() const;
		void							SetFreeze(bool freeze);


		virtual void Update(time_seconds const& time_delta) override;
		// Inherited via IRenderable
		virtual bool IsVisible(const BoundingFrustum& viewFrustum) const override;
		virtual void Render(RenderContext & pContext) override;
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection) override;

	private:
		BehavierSpace*					        m_pBehavier;
		BehavierSpace::animation_type*			m_pCurrentAction;
		BehavierSpace::animation_type*			m_pLastAction;
		time_seconds							m_CurrentActionTime;
		bool									m_LoopCurrentAction;

		int										m_FrameMapState;
		frame_type						        m_CurrentFrame;

		bool									m_DirtyFlag;
		std::vector<CharacterController*>				m_Controls;
	};

	// Represent an ControlObject
	class KinematicAIController
	{

	};

	class KeyboardMouseFirstPersonControl :public SceneObject, public IAppComponent, public IKeybordInteractive, public ICursorInteractive
	{
	public:
		KeyboardMouseFirstPersonControl(IRigid* pTarget = nullptr);

		void SetTarget(IRigid* pTarget);

		// Inherited via ITimeAnimatable
		virtual void Update(time_seconds const& time_delta) override;

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