#include "CausalityApplication.h"
#include "Content\CubeScene.h"
#include "Content\SampleFpsTextRenderer.h"
#include "Common\SkyBox.h"
#include "Foregrounds.h"

using namespace Causality;
using namespace std;
using namespace DirectX;
using namespace DirectX::Scene;
using namespace Platform;
using namespace Platform::Fundation;
using namespace boost;
//std::unique_ptr<Causality::DXAppMain> m_main;

wstring ResourcesDirectory(L"C:\\Users\\Yupeng\\Documents\\GitHub\\VR\\Causality\\Resources\\");

const static wstring SkyBoxTextures[6] = {
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Right.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Left.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Top.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Bottom.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Front.dds"),
	ResourcesDirectory + wstring(L"Textures\\SkyBox\\GrimmNight\\Back.dds"),
};


App::App()
{
}

App::~App()
{
}

void Causality::App::OnStartup(Platform::Array<Platform::String^>^ args)
{
	ResourceDirectory = filesystem::current_path().parent_path() / "Resources";

	pWindow = make_shared<Platform::NativeWindow>();
	pWindow->Initialize(ref new String(L"Causality"), 1280U, 720, false);

	pDeviceResources = make_shared<DirectX::DeviceResources>();
	pDeviceResources->SetNativeWindow(pWindow->Handle());
	// Register to be notified if the Device is lost or recreated
	pDeviceResources->RegisterDeviceNotify(this);

	// Oculus Rift
	pRift = std::make_shared<Platform::Devices::OculusRift>();
	try
	{
		pRift->Initialize(pWindow->Handle(), pDeviceResources.get());
		pRift->DissmisHealthWarnning();
	}
	catch (std::runtime_error exception)
	{
		pRift = nullptr;
	}

	pLeap = std::make_shared<Platform::Devices::LeapMotion>(false);
	
	// Primary Camera setup
	auto pPlayer = std::make_unique<PlayerCamera>(pDeviceResources);
	if (pRift)
		pPlayer->EnableStereo(pRift);
	else
	{
		auto size = pDeviceResources->GetOutputSize();
		pPlayer->SetFov(75.f*XM_PI / 180.f, size.Width / size.Height);
	}
	pPlayer->SetPosition(Fundation::Vector3(0.0f, 0.7f, 1.5f));
	pPlayer->FocusAt(Fundation::Vector3(0, 0, 0), Fundation::Vector3(0.0f, 1.0f, 0));

	m_pPrimaryCamera = std::move(pPlayer);

	// Scenes
	Scenes.push_back(std::make_unique<CubeScene>(pDeviceResources));
	Scenes.push_back(std::make_unique<FpsTextScene>(pDeviceResources));
	Scenes.push_back(std::make_unique<SkyBox>(pDeviceResources->GetD3DDevice(), SkyBoxTextures));
	Scenes.push_back(std::make_unique<Foregrounds>(pDeviceResources));

	// Register Interactive Ecents
	for (auto& pScene : Scenes)
	{
		RegisterScene(pScene.get());
	}
}

void Causality::App::RegisterScene(IRenderable* pScene)
{
	auto pCursorInteractive = dynamic_cast<ICursorInteractive*>(pScene);
	if (pCursorInteractive)
	{
		pWindow->CursorButtonDown += MakeEventHandler(&ICursorInteractive::OnMouseButtonDown, pCursorInteractive);
		pWindow->CursorButtonUp += MakeEventHandler(&ICursorInteractive::OnMouseButtonUp, pCursorInteractive);
		pWindow->CursorMove += MakeEventHandler(&ICursorInteractive::OnMouseMove, pCursorInteractive);
	}
	auto pKeyInteractive = dynamic_cast<IKeybordInteractive*>(pScene);
	if (pKeyInteractive)
	{
		pWindow->KeyDown += MakeEventHandler(&IKeybordInteractive::OnKeyDown, pKeyInteractive);
		pWindow->KeyUp += MakeEventHandler(&IKeybordInteractive::OnKeyUp, pKeyInteractive);
	}
	auto pAnimatable = dynamic_cast<ITimeAnimatable*>(pScene);
	if (pAnimatable)
		TimeElapsed += MakeEventHandler(&ITimeAnimatable::UpdateAnimation, pAnimatable);
	auto pHands = dynamic_cast<IUserHandsInteractive*>(pScene);
	if (pHands)
	{
	}

}


void Causality::App::OnExit()
{
}

void Causality::App::OnIdle()
{
	// Time Aware update
	m_timer.Tick([&]()
	{
		TimeElapsed(m_timer);
		//// TODO: Replace this with your app's content update functions.
		//for (auto& pScene : Scenes)
		//{
		//	auto pAnimatable = dynamic_cast<ITimeAnimatable*>(pScene.get());
		//	if (pAnimatable)
		//		pAnimatable->UpdateAnimation(m_timer);
		//}
	});

	// Rendering
	auto pRenderControl = dynamic_cast<ICameraRenderControl*>(m_pPrimaryCamera.get());

	pRenderControl->BeginFrame();
	for (size_t view = 0; view < m_pPrimaryCamera->ViewCount(); view++)
	{
		pRenderControl->SetView(view);
		auto v = m_pPrimaryCamera->GetViewMatrix(view);
		auto p = m_pPrimaryCamera->GetProjectionMatrix(view);
		RenderToView(v, p);
	}
	pRenderControl->EndFrame();

	//if (pRift)
	//{
	//	pRift->BeginFrame();
	//	// Otherwise do the frame processing.
	//	for (int eye = 0; eye < 2; eye++)
	//	{
	//		pRift->EyeTexture((DirectX::Scene::EyesEnum) eye).SetAsRenderTarget(pDeviceResources->GetD3DDeviceContext(), pRift->DepthStencilBuffer());

	//		auto view = pPlayer->GetViewMatrix((DirectX::Scene::EyesEnum) eye);
	//		auto projection = pPlayer->GetProjectionMatrix((DirectX::Scene::EyesEnum) eye);

	//		RenderToView(view, projection);
	//	}
	//	pRift->EndFrame();
	//}
	//else
	//{
	//	auto context = pDeviceResources->GetD3DDeviceContext();

	//	// Reset the viewport to target the whole screen.
	//	auto viewport = pDeviceResources->GetScreenViewport();
	//	context->RSSetViewports(1, &viewport);
	//	ID3D11RenderTargetView *const targets[1] = { pDeviceResources->GetBackBufferRenderTargetView() };
	//	context->OMSetRenderTargets(1, targets, pDeviceResources->GetDepthStencilView());
	//	context->ClearRenderTargetView(pDeviceResources->GetBackBufferRenderTargetView(), DirectX::Colors::White);
	//	context->ClearDepthStencilView(pDeviceResources->GetDepthStencilView(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

	//	auto view = pPlayer->GetViewMatrix(0);
	//	auto projection = pPlayer->GetProjectionMatrix(0);
	//	RenderToView(view, projection);

	//	pDeviceResources->Present();
	//}
}

void Causality::App::OnDeviceLost()
{
}

void Causality::App::OnDeviceRestored()
{
}

boost::filesystem::path Causality::App::GetResourcesDirectory() const
{
	return ResourceDirectory.wstring();
}

void Causality::App::SetResourcesDirectory(const std::wstring & dir)
{
	ResourceDirectory = dir;
}

void XM_CALLCONV Causality::App::RenderToView(DirectX::FXMMATRIX view, DirectX::CXMMATRIX projection)
{
	auto pContext = pDeviceResources->GetD3DDeviceContext();
	for (auto& pScene : Scenes)
	{
		auto pViewable = dynamic_cast<IViewable*>(pScene.get());
		if (pViewable)
		{
			pViewable->UpdateViewMatrix(view);
			pViewable->UpdateProjectionMatrix(projection);
		}

		pScene->Render(pContext);
	}
}

//inline void SampleListener::onConnect(const Leap::Controller & controller) {
//	std::cout << "Connected" << std::endl;
//}
//
//inline void SampleListener::onFrame(const Leap::Controller & controller) {
//	auto frame = controller.frame();
//	if (frame.hands().count() > PrevHandsCount)
//	{
//		PrevHandsCount = frame.hands().count();
//	}
//	else (frame.hands().count() < PrevHandsCount)
//	{
//		PrevHandsCount = frame.hands().count();
//	}
//	//std::cout << "Frame available" << std::endl;
//}
