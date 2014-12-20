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
#include <boost\circular_buffer.hpp>
#include <boost\thread\shared_mutex.hpp>
#include <boost\thread\locks.hpp>
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
		HandPhysicalModel(btDynamicsWorld* pWorld,const Leap::Frame& frame, const Leap::Hand& hand , const DirectX::Matrix4x4& transform);

		inline const std::vector<std::shared_ptr<PhysicalRigid>>& Rigids()
		{
			return m_HandRigids;
		}

		bool Update(const Leap::Frame& frame, const DirectX::Matrix4x4& transform);

		bool IsTracked() const
		{
			return m_Hand.isValid();
		}

		// Inherited via IModelNode
		virtual void Render(ID3D11DeviceContext * pContext, DirectX::IEffect * pEffect) override;

	private:
		int			Id;
		Leap::Hand  m_Hand;
		std::vector<std::shared_ptr<btCollisionShape>> m_BoneShapes;
		std::vector<std::shared_ptr<PhysicalRigid>>    m_HandRigids;
		static std::unique_ptr<DirectX::GeometricPrimitive> s_pCylinder;
		static std::unique_ptr<DirectX::GeometricPrimitive> s_pSphere;
	};

	class WorldScene : public Platform::IAppComponent, public Platform::IUserHandsInteractive, public Platform::IKeybordInteractive, public DirectX::Scene::IRenderable, private DirectX::Scene::ModelCollection, public DirectX::Scene::IViewable , public DirectX::Scene::ITimeAnimatable
	{
	public:
		WorldScene(const std::shared_ptr<DirectX::DeviceResources> &pResouce);
		~WorldScene();

		void LoadAsync(ID3D11Device* pDevice);

		std::shared_ptr<DirectX::BasicEffect> pEffect;

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
		const ILocatable*								m_pCameraLocation;

		std::shared_ptr<btCollisionShape>					 pGroundShape = nullptr;
		std::shared_ptr<btRigidBody>						 pGroundRigid = nullptr;
		std::vector<std::shared_ptr<PhysicalRigid>>			 m_HandRigids;
		std::shared_ptr<btConeShape>						 m_pHandConeShape;
		std::map<int, std::shared_ptr<btCollisionObject>>	 m_pHandCones;
		std::map<int, std::map<std::string, std::vector<std::shared_ptr<btRigidBody>>>> m_pHandsRigids;
		std::map<int, std::unique_ptr<btDynamicsWorld>>		 m_pDynamicsWorlds;

		std::unique_ptr<btBroadphaseInterface>               pBroadphase = nullptr;
		// Set up the collision configuration and dispatcher
		std::unique_ptr<btDefaultCollisionConfiguration>     pCollisionConfiguration = nullptr;
		std::unique_ptr<btCollisionDispatcher>               pDispatcher = nullptr;
		// The actual physics solver
		std::unique_ptr<btSequentialImpulseConstraintSolver> pSolver = nullptr;
		std::shared_ptr<btDynamicsWorld>                     pDynamicsWorld = nullptr;
	
		std::map<int, std::unique_ptr<HandPhysicalModel>>	 m_HandModels;

		bool m_showTrace;
		bool m_loadingComplete;

		// Inherited via ModelCollection
		virtual void XM_CALLCONV SetModelMatrix(DirectX::FXMMATRIX model) override;
		virtual DirectX::XMMATRIX GetModelMatrix() const override;
	};
}
