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

namespace Causality
{
	class WorldScene : public Platform::IAppComponent, public Platform::IUserHandsInteractive, public DirectX::Scene::IRenderable, public DirectX::Scene::ModelCollection, public DirectX::Scene::IViewable , public DirectX::Scene::ITimeAnimatable
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
	private:
		std::unique_ptr<DirectX::Scene::SkyDome>		pBackground;
		Microsoft::WRL::ComPtr<ID3D11RasterizerState>	pRSState;
		Microsoft::WRL::ComPtr<ID3D11InputLayout>		pInputLayout;
		DirectX::CommonStates							States;
		bool											m_HaveHands;
		Leap::Frame										m_Frame;
		DirectX::Matrix4x4								m_FrameTransform;
		const int TraceLength = 45;
		std::deque<std::array<DirectX::Vector3, 25>>	m_HandTrace;
		std::deque<std::array<DirectX::Vector3,2000>>	m_TraceSamples;
		std::vector<DirectX::Vector3>					m_TracePoints;
		DirectX::BoundingOrientedBox					m_CurrentHandBoundingBox;
		DirectX::BoundingOrientedBox					m_HandTraceBoundingBox;
		std::map<std::string, DirectX::Vector2>			m_ModelsFeature;
		std::map<std::string, float>					m_ModelDetailSimilarity;
		DirectX::Vector2								m_HandDescriptionFeature;
		std::mutex										m_HandFrameMutex;

		bool m_loadingComplete;

	};
}
