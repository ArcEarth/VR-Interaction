#pragma once
#include "Common\Model.h"
#include "Common\DeviceResources.h"
#include <Effects.h>
#include "Interactive.h"
#include <wrl\client.h>
#include <CommonStates.h>
#include "Common\SkyBox.h"
#include <mutex>

namespace Causality
{
	class Foregrounds : public Platform::IAppComponent, public Platform::IUserHandsInteractive, public DirectX::Scene::IRenderable, public DirectX::Scene::ModelCollection, public DirectX::Scene::IViewable , public DirectX::Scene::ITimeAnimatable
	{
	public:
		Foregrounds(const std::shared_ptr<DirectX::DeviceResources> &pResouce);
		~Foregrounds();

		void LoadAsync(ID3D11Device* pDevice);

		std::shared_ptr<DirectX::BasicEffect> pEffect;

		//virtual DirectX::XMMATRIX GetModelMatrix() const override;
		// Inherited via IRenderable
		virtual void Render(ID3D11DeviceContext * pContext) override;

		// Inherited via IViewable
		virtual void XM_CALLCONV UpdateViewMatrix(DirectX::FXMMATRIX view) override;
		virtual void XM_CALLCONV UpdateProjectionMatrix(DirectX::FXMMATRIX projection) override;

		// Inherited via ITimeAnimatable
		virtual void UpdateAnimation(DirectX::StepTimer const & timer) override;

	private:
		std::unique_ptr<DirectX::Scene::SkyDome>		pBackground;
		Microsoft::WRL::ComPtr<ID3D11RasterizerState>	pRSState;
		Microsoft::WRL::ComPtr<ID3D11InputLayout>		pInputLayout;
		DirectX::CommonStates							States;
		std::mutex										m_HandFrameMutex;
		Leap::Frame										m_Frame;
		bool m_loadingComplete;

		// Inherited via IUserHandsInteractive
		virtual void OnHandsTracked(const Platform::UserHandsEventArgs & e) override;
		virtual void OnHandsTrackLost(const Platform::UserHandsEventArgs & e) override;
		virtual void OnHandsMove(const Platform::UserHandsEventArgs & e) override;
	};
}
