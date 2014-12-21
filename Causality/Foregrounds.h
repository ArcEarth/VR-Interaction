#pragma once
#include "Common\Model.h"
#include "Common\DeviceResources.h"
#include <Effects.h>
#include "Interactive.h"
#include <wrl\client.h>
#include <CommonStates.h>
#include "Common\SkyBox.h"
#include <mutex>
#include <map>
#include <array>
#include <deque>
#include "Common\MetaBallModel.h"
#include <PrimitiveBatch.h>
#include "BulletPhysics.h"
#include <GeometricPrimitive.h>

namespace Causality
{
	class SceneObject : public PhysicalGeometryModel
	{
	};

	struct HandPhysicalModel : public DirectX::Scene::IModelNode, public DirectX::Scene::LocalMatrixHolder
	{
		HandPhysicalModel(const std::shared_ptr<btDynamicsWorld> &pWorld,const Leap::Frame& frame, const Leap::Hand& hand , const DirectX::Matrix4x4& transform);

		inline const std::vector<std::shared_ptr<PhysicalRigid>>& Rigids()
		{
			return m_HandRigids;
		}

		bool Update(const Leap::Frame& frame, const DirectX::Matrix4x4& transform);

		bool IsTracked() const
		{
			return m_Hand.isValid();
		}
		int LostFramesCount() const { return LostFrames; }
		// Inherited via IModelNode
		virtual void Render(ID3D11DeviceContext * pContext, DirectX::IEffect * pEffect) override;

	private:
		int			Id;
		int			LostFrames;
		Leap::Hand  m_Hand;
		std::vector<std::shared_ptr<PhysicalRigid>>    m_HandRigids;
		static std::unique_ptr<DirectX::GeometricPrimitive> s_pCylinder;
		static std::unique_ptr<DirectX::GeometricPrimitive> s_pSphere;
	};

	class CubeModel : public DirectX::Scene::IModelNode, public DirectX::Scene::IRigidLocalMatrix, public PhysicalRigid
	{
	public:
		CubeModel(const std::string& name = "Cube", DirectX::FXMVECTOR extend = DirectX::g_XMOne, DirectX::FXMVECTOR color = DirectX::Colors::White)
		{
			Name = name;
			m_Color = color;
			m_pShape.reset(new btBoxShape(vector_cast<btVector3>(extend)));
			XMStoreFloat3(&BoundBox.Extents,extend);
			XMStoreFloat3(&BoundOrientedBox.Extents, extend);
		}
		DirectX::XMVECTOR GetColor();
		void XM_CALLCONV SetColor(DirectX::FXMVECTOR color);
		virtual void Render(ID3D11DeviceContext *pContext, DirectX::IEffect* pEffect);

	private:
		DirectX::Color m_Color;
	};

	class Frame;
	class FramePool
	{
		std::shared_ptr<Frame> CreateFrame();
		std::shared_ptr<Frame> CopyFrame(const Frame& frame);
	};

	// One problistic frame for current state
	class Frame
	{
	public:
		// Object states evolution with time and interaction subjects
		std::map<std::string, std::shared_ptr<PhysicalRigid>>	Objects;

		// Interactive subjects
		std::map<int, std::shared_ptr<HandPhysicalModel>>		Subjects;
		std::map<int, DirectX::Matrix4x4>						SubjectTransforms;

		float Liklyhood() const;

		//copy sementic
		Frame(const Frame& other)
		{
			*this = other;
		}
		Frame& operator=(const Frame& other);
		// move sementic
		Frame(Frame&& other)
		{
			*this = std::move(other);
		}
		Frame& operator=(const Frame&& other);

		// Evolution caculation object
		std::shared_ptr<btBroadphaseInterface>					pBroadphase = nullptr;
		// Set up the collision configuration and dispatcher
		std::shared_ptr<btDefaultCollisionConfiguration>		pCollisionConfiguration = nullptr;
		std::shared_ptr<btCollisionDispatcher>					pDispatcher = nullptr;
		// The actual physics solver
		std::shared_ptr<btSequentialImpulseConstraintSolver>	pSolver = nullptr;
		std::shared_ptr<btDynamicsWorld>						pDynamicsWorld = nullptr;
	};

	class WorldScene : public Platform::IAppComponent, public Platform::IUserHandsInteractive, public Platform::IKeybordInteractive, public DirectX::Scene::IRenderable, private DirectX::Scene::ModelCollection, public DirectX::Scene::IViewable , public DirectX::Scene::ITimeAnimatable
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

		void DrawBox(DirectX::SimpleMath::Vector3  conners[], DirectX::CXMVECTOR color);

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
		std::deque<std::array<DirectX::Vector3,2000>>	m_TraceSamples;
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

		std::list<std::shared_ptr<Frame>>				m_Frames;

		std::shared_ptr<btConeShape>						 m_pHandConeShape;
		std::map<int, std::shared_ptr<btCollisionObject>>	 m_pHandCones;
		std::map<int, std::map<std::string, std::vector<std::shared_ptr<btRigidBody>>>> m_pHandsRigids;
		std::map<int, std::unique_ptr<btDynamicsWorld>>		 m_pDynamicsWorlds;

		std::shared_ptr<btDynamicsWorld>                     pDynamicsWorld = nullptr;
		
		std::map<int, std::unique_ptr<HandPhysicalModel>>	 m_HandModels;

		bool m_showTrace;
		bool m_loadingComplete;

		// Inherited via ModelCollection
		virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override;
		virtual DirectX::XMMATRIX GetModelMatrix() const override;
	};
}
