#pragma once
#include "Common\Model.h"
#include "Common\DeviceResources.h"
#include <Effects.h>
#include "Interactive.h"
#include <wrl\client.h>
#include <CommonStates.h>
#include "Common\SkyDome.h"
#include <thread>
#include <condition_variable>
#include <mutex>
#include <map>
#include <array>
#include <deque>
#include <queue>
#include <stack>
#include "Common\MetaBallModel.h"
#include <PrimitiveBatch.h>
#include "BulletPhysics.h"
#include <GeometricPrimitive.h>
#include "Common\Filter.h"
#include "Common\tree.h"

namespace Causality
{
	//class StateObject : virtual public DirectX::IRigid
	//{
	//	virtual ~StateObject()
	//	{}
	//};

	class IShaped
	{
	public:
		virtual std::shared_ptr<btCollisionShape> CreateCollisionShape() = 0;
	};

	class ShapedGeomrtricModel : public DirectX::Scene::GeometryModel, virtual public IShaped
	{
	public:
		virtual std::shared_ptr<btCollisionShape> CreateCollisionShape() override;

	private:
		std::shared_ptr<btCollisionShape> m_pShape;
	};

	struct ProblistiscAffineTransform : public DirectX::AffineTransform
	{
		using DirectX::AffineTransform::AffineTransform;
		float Probability;
	};

	// A super state which encode different states with probility
	struct ProblistiscState
	{
	public:
		std::string	Name;
		std::vector<ProblistiscAffineTransform> StatesDistribution;
	};

	struct HandPhysicalModel : public DirectX::Scene::IModelNode
	{
		HandPhysicalModel(const std::shared_ptr<btDynamicsWorld> &pWorld,
			const Leap::Hand & hand, const DirectX::Matrix4x4 & leapTransform,
			const DirectX::AffineTransform &inheritTransform);

		DirectX::XMMATRIX CaculateLocalMatrix(const Leap::Hand & hand, const DirectX::Matrix4x4 & leapTransform);

		inline const std::vector<std::shared_ptr<PhysicalRigid>>& Rigids()
		{
			return m_HandRigids;
		}

		bool Update(const Leap::Frame& frame, const DirectX::Matrix4x4& leapTransform);

		bool IsTracked() const
		{
			return m_Hand.isValid();
		}
		int LostFramesCount() const { return LostFrames; }
		// Inherited via IModelNode
		virtual void Render(ID3D11DeviceContext * pContext, DirectX::IEffect * pEffect) override;

		DirectX::XMVECTOR XM_CALLCONV FieldAtPoint(DirectX::FXMVECTOR P);
	public:
		float		Opticity;

	private:
		int			Id;
		int			LostFrames;
		Leap::Hand  m_Hand;
		DirectX::AffineTransform m_InheritTransform;
		std::array<std::pair<DirectX::Vector3, DirectX::Vector3>, 20> m_Bones;
		std::vector<std::shared_ptr<PhysicalRigid>>			m_HandRigids;
		//static std::unique_ptr<DirectX::GeometricPrimitive> s_pCylinder;
		//static std::unique_ptr<DirectX::GeometricPrimitive> s_pSphere;
	};

	class CubeModel : public DirectX::Scene::IModelNode, virtual public IShaped
	{
	public:
		CubeModel(const std::string& name = "Cube", DirectX::FXMVECTOR extend = DirectX::g_XMOne, DirectX::FXMVECTOR color = DirectX::Colors::White);
		virtual std::shared_ptr<btCollisionShape> CreateCollisionShape() override;

		DirectX::XMVECTOR GetColor();
		void XM_CALLCONV SetColor(DirectX::FXMVECTOR color);
		virtual void Render(ID3D11DeviceContext *pContext, DirectX::IEffect* pEffect);

	private:
		DirectX::Color m_Color;
	};

	// One problistic frame for current state
	class WorldBranch : public stree::tree_node<WorldBranch, false>
	{
	public:

		enum CollisionGroupEnum : short
		{
			Group_Focused_Object = 0x1,
			Group_Unfocused_Object = 0x2,
			Group_Subject = 0x4,
			Mask_Focused_Object = 0x7,
			Mask_Unfocused_Object = 0x3,
			Mask_Subject = 0x1,
		};

	public:
		static void InitializeBranchPool(int size, bool autoExpandation = true);
		static std::unique_ptr<WorldBranch> DemandCreate();
		static void Recycle(std::unique_ptr<WorldBranch>&&);

	private:
		static std::queue<std::unique_ptr<WorldBranch>> IdelFrames;

	public:
		void Reset();

		float Liklyhood() const
		{
			if (IsEnabled)
				return 1;
			else
				return 0;
		}

		WorldBranch();

		void Enable(const DirectX::AffineTransform& subjectTransform)
		{
			IsEnabled = true;
			SubjectTransform = subjectTransform;
		}

		void Disable()
		{
			IsEnabled = false;
		}

		//copy / move sementic is deleted for use with pointer
		WorldBranch(const WorldBranch& other) = delete;
		WorldBranch& operator=(const WorldBranch& other) = delete;
		WorldBranch(WorldBranch&& other) = delete;
		WorldBranch& operator=(const WorldBranch&& other) = delete;

		void AddSubjectiveObject(const Leap::Hand& hand, const DirectX::Matrix4x4& leapTransform);
		void AddDynamicObject(const std::string &name, const std::shared_ptr<btCollisionShape> &pShape, float mass, const DirectX::Vector3 & Position, const DirectX::Quaternion & Orientation);
		void Evolution(float timeStep, const Leap::Frame & frame, const DirectX::Matrix4x4 & leapTransform);
		void Fork();
		void Collapse();
		std::vector<ProblistiscState> CaculateSuperposition();
		void UpdateLiklyhood();
		void NormalizeLiklyhood(float c);

	public:
		// Internal evolution algorithm as-if this branch is a "Leaf"
		void InternalEvolution(const Leap::Frame & frame, const DirectX::Matrix4x4 & leapTransform);
	protected:
		std::string												Name;

		std::unique_ptr<std::thread>							pWorkerThread;
		std::condition_variable									queuePending;
		std::mutex												queueMutex;
	protected:
		// Evolution caculation object
		std::shared_ptr<btBroadphaseInterface>					pBroadphase = nullptr;
		// Set up the collision configuration and dispatcher
		std::shared_ptr<btDefaultCollisionConfiguration>		pCollisionConfiguration = nullptr;
		std::shared_ptr<btCollisionDispatcher>					pDispatcher = nullptr;
		// The actual physics solver
		std::shared_ptr<btSequentialImpulseConstraintSolver>	pSolver = nullptr;
		std::shared_ptr<btDynamicsWorld>						pDynamicsWorld = nullptr;

	public:
		// Object states evolution with time and interaction subjects
		std::map<std::string, std::shared_ptr<PhysicalRigid>>	Objects;

		// Interactive subjects
		std::map<int, std::shared_ptr<HandPhysicalModel>>		Subjects;

		DirectX::AffineTransform								SubjectTransform;


		bool													IsDirty;
		bool													IsEnabled;
	};

	class WorldScene : public Platform::IAppComponent, public Platform::IUserHandsInteractive, public Platform::IKeybordInteractive, public DirectX::Scene::IRenderable, public DirectX::Scene::IViewable, public DirectX::Scene::ITimeAnimatable
	{
	public:
		WorldScene(const std::shared_ptr<DirectX::DeviceResources> &pResouce, const DirectX::ILocatable* pCamera);
		~WorldScene();

		void LoadAsync(ID3D11Device* pDevice);

		std::shared_ptr<DirectX::BasicEffect> pEffect;

		void SetViewIdenpendntCameraPosition(const DirectX::ILocatable* pCamera);
		//virtual DirectX::XMMATRIX GetModelMatrix() const override;
		// Inherited via IRenderable
		virtual void Render(ID3D11DeviceContext * pContext) override;

		void DrawBox(DirectX::SimpleMath::Vector3  conners [], DirectX::CXMVECTOR color);

		// Inherited via IViewable
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view) override;
		virtual void XM_CALLCONV UpdateProjectionMatrix(DirectX::FXMMATRIX projection) override;

		// Inherited via ITimeAnimatable
		virtual void UpdateAnimation(DirectX::StepTimer const & timer) override;

		// Inherited via IUserHandsInteractive
		virtual void OnHandsTracked(const Platform::UserHandsEventArgs & e) override;
		virtual void OnHandsTrackLost(const Platform::UserHandsEventArgs & e) override;
		virtual void OnHandsMove(const Platform::UserHandsEventArgs & e) override;

		// Inherited via IKeybordInteractive
		virtual void OnKeyDown(const Platform::KeyboardEventArgs & e) override;

		virtual void OnKeyUp(const Platform::KeyboardEventArgs & e) override;

		void AddObject(const std::shared_ptr<DirectX::Scene::IModelNode>& pModel, float mass, const DirectX::Vector3 &Position, const DirectX::Quaternion &Orientation, const DirectX::Vector3 &Scale);

		std::vector<ProblistiscState> ComposeFrame();

		bool IsCurrentStateUnderdeterminate() const;

		int CurrentStateOverloadCount() const;

		const WorldBranch* MasterFrame() const
		{
			return m_StateFrames.front();
		}

		WorldBranch* MasterFrame()
		{
			return m_StateFrames.front();
		}



		// Forece the scene to resolve the ambiguity, compress the frame count into one
		void CollapseStates();
		void ForkStates();
	private:
		std::unique_ptr<DirectX::Scene::SkyDome>		pBackground;
		Microsoft::WRL::ComPtr<ID3D11RasterizerState>	pRSState;
		Microsoft::WRL::ComPtr<ID3D11InputLayout>		pInputLayout;
		DirectX::CommonStates							States;
		std::unique_ptr<DirectX::PrimitiveBatch<DirectX::VertexPositionNormal>>
			pBatch;
		bool											m_HaveHands;
		Leap::Frame										m_Frame;
		DirectX::Matrix4x4								m_FrameTransform;
		const int TraceLength = 1;
		std::deque<std::array<DirectX::Vector3, 25>>	m_HandTrace;
		std::deque<std::array<DirectX::Vector3, 2000>>	m_TraceSamples;
		std::vector<DirectX::Vector3>					m_TracePoints;
		DirectX::BoundingOrientedBox					m_CurrentHandBoundingBox;
		DirectX::BoundingOrientedBox					m_HandTraceBoundingBox;
		std::map<std::string, Eigen::VectorXf>			m_ModelFeatures;
		std::map<std::string, float>					m_ModelDetailSimilarity;
		Eigen::VectorXf									m_HandDescriptionFeature;
		std::mutex										m_HandFrameMutex;
		Geometrics::MetaBallModel						m_HandTraceModel;
		std::vector<DirectX::VertexPositionNormal>		m_HandTraceVertices;
		std::vector<uint16_t>							m_HandTraceIndices;
		std::mutex										m_RenderLock;
		const DirectX::ILocatable*						m_pCameraLocation;

		DirectX::Scene::ModelCollection					Models;

		std::unique_ptr<WorldBranch>					WorldTree;

		std::list<WorldBranch*>							m_StateFrames;

		std::shared_ptr<btConeShape>						 m_pHandConeShape;
		std::map<int, std::shared_ptr<btCollisionObject>>	 m_pHandCones;
		std::map<int, std::map<std::string, std::vector<std::shared_ptr<btRigidBody>>>> m_pHandsRigids;
		std::map<int, std::unique_ptr<btDynamicsWorld>>		 m_pDynamicsWorlds;

		std::shared_ptr<btDynamicsWorld>                     pDynamicsWorld = nullptr;

		std::map<int, std::unique_ptr<HandPhysicalModel>>	 m_HandModels;


		bool m_showTrace;
		bool m_loadingComplete;

	};
}
