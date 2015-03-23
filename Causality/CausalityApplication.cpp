#include "pch.h"
#include "CausalityApplication.h"
#include "Content\CubeScene.h"
#include "Content\SampleFpsTextRenderer.h"
#include <CommonStates.h>
#include "Common\PrimitiveVisualizer.h"

using namespace Causality;
using namespace std;
using namespace DirectX;
using namespace DirectX::Scene;
using namespace Platform;
using namespace boost;
//std::unique_ptr<Causality::DXAppMain> m_main;

wstring ResourcesDirectory(L"C:\\Users\\Yupeng\\Documents\\GitHub\\VR\\Causality\\Resources\\");

App::App()
{
}

App::~App()
{
	for (auto& pCom : Components)
	{
		UnregisterComponent(pCom.get());
	}
}

inline void Application::Exit()
{
	for (auto& p : WindowsLookup)
	{
		auto pWindow = p.second.lock();
		if (pWindow)
			pWindow->Close();
	}
	PostQuitMessage(0);
}

void Causality::App::OnStartup(Array<String^>^ args)
{
	ResourceDirectory = filesystem::current_path() / "Resources";
	
	// Initialize Windows
	pConsole = make_shared<DebugConsole>();
	pConsole->Initialize(ref new String(L"CausalityDebug"), 800, 600, false);

	//pRift = Devices::OculusRift::GetForCurrentView();

	pWindow = make_shared<NativeWindow>();
	if (!pRift)
		pWindow->Initialize(ref new String(L"Causality"), 1920, 1080, false);
	else
	{
		auto res = pRift->Resoulution();
		pWindow->Initialize(ref new String(L"Causality"),(unsigned) res.x, (unsigned) res.y, false);
	}
	//bool useOvr = Devices::OculusRift::Initialize();

	// Initialize DirectX
	pDeviceResources = make_shared<DirectX::DeviceResources>();
	pDeviceResources->SetNativeWindow(pWindow->Handle());
	// Register to be notified if the Device is lost or recreated
	pDeviceResources->RegisterDeviceNotify(this);

	pDeviceResources->GetD3DDevice()->AddRef();
	pDeviceResources->GetD3DDeviceContext()->AddRef();
	pDevice.Attach(pDeviceResources->GetD3DDevice());
	pContext.Attach(pDeviceResources->GetD3DDeviceContext());

	Visualizers::g_PrimitiveDrawer.Initialize(pDeviceResources->GetD3DDeviceContext());

	// Oculus Rift
	//if (pRift)
	//{
	//	if (!pRift->InitializeGraphics(pWindow->Handle(), pDeviceResources.get()))
	//		pRift = nullptr;
	//}
	//pLeap = Devices::LeapMotion::GetForCurrentView();;
	pKinect = Devices::Kinect::GetForCurrentView();

	Scenes.emplace_back(new Scene());
	Scenes.back()->SetRenderDeviceAndContext(pDevice, pContext);
	Scenes.back()->SetCanvas(pDeviceResources->GetBackBufferRenderTarget());
	Scenes.back()->LoadFromXML((ResourceDirectory / "SelectorScene.xml").string());
}

void Causality::App::RegisterComponent(IAppComponent *pComponent)
{
	auto pCursorInteractive = pComponent->As<ICursorInteractive>();
	auto& Regs = ComponentsEventRegisterations[pComponent];
	if (pCursorInteractive)
	{
		Regs.push_back(pWindow->CursorButtonDown += MakeEventHandler(&ICursorInteractive::OnMouseButtonDown, pCursorInteractive));
		Regs.push_back(pWindow->CursorButtonUp += MakeEventHandler(&ICursorInteractive::OnMouseButtonUp, pCursorInteractive));
		Regs.push_back(pWindow->CursorMove += MakeEventHandler(&ICursorInteractive::OnMouseMove, pCursorInteractive));
	}
	auto pKeyInteractive = pComponent->As<IKeybordInteractive>();
	if (pKeyInteractive)
	{
		Regs.push_back(pWindow->KeyDown += MakeEventHandler(&IKeybordInteractive::OnKeyDown, pKeyInteractive));
		Regs.push_back(pWindow->KeyUp += MakeEventHandler(&IKeybordInteractive::OnKeyUp, pKeyInteractive));
	}
	//auto pAnimatable = pComponent->As<ITimeAnimatable>();
	//if (pAnimatable)
	//	Regs.push_back(TimeElapsed += MakeEventHandler(&ITimeAnimatable::UpdateAnimation, pAnimatable));
	auto pHands = pComponent->As<IUserHandsInteractive>();
	if (pHands && pLeap)
	{
		Regs.push_back(pLeap->HandsTracked += MakeEventHandler(&IUserHandsInteractive::OnHandsTracked, pHands));
		Regs.push_back(pLeap->HandsLost += MakeEventHandler(&IUserHandsInteractive::OnHandsTrackLost, pHands));
		Regs.push_back(pLeap->HandsMove += MakeEventHandler(&IUserHandsInteractive::OnHandsMove, pHands));
	}
	Components.push_back(std::move(pComponent));
}

void Causality::App::UnregisterComponent(IAppComponent * pComponent)
{
	auto& Regs = ComponentsEventRegisterations[pComponent];
	for (auto& connection : Regs)
	{
		connection.disconnect();
	}
}


void Causality::App::OnExit()
{
}

void Causality::App::OnIdle()
{
	//ComPtr<IBodyFrame> pBodyFrame;
	//HRESULT hr = pKinect->BodyFrameReader()->AcquireLatestFrame(&pBodyFrame);
	//if (SUCCEEDED(hr))
	//{
	//	IBody *pBodys[6];
	//	hr = pBodyFrame->GetAndRefreshBodyData(6, pBodys);
	//	INT64 nTime = 0;
	//	hr = pBodyFrame->get_RelativeTime(&nTime);

	//	int count = 0;
	//	while (pBodys[count] != nullptr && count < 6)
	//		count++;

	//	if (hr)
	//		for (const auto& pComponent : Components)
	//		{
	//			auto pBodyInteractive = pComponent->As<IUserBodyInteractive>();
	//			if (pBodyInteractive)
	//			{
	//				pBodyInteractive->OnBodyFrameUpdated(nTime, count, pBodys);
	//			}
	//		}
	//}
	// Time Aware update
	m_timer.Tick([&]()
	{
		// Processing & Distribute Extra Input
		if (pLeap)
			pLeap->PullFrame();
		if (pKinect)
			pKinect->ProcessFrame();

		for (auto& pScene : Scenes)
		{
			pScene->Update();
			pScene->Render(pContext);
		}

		//TimeElapsed(m_timer);
		//// Rendering
		//auto pRenderControl = dynamic_cast<ICameraRenderControl*>(m_pPrimaryCamera.get());

		//pRenderControl->BeginFrame();
		//for (size_t view = 0; view < m_pPrimaryCamera->ViewCount(); view++)
		//{
		//	pRenderControl->SetView(view);
		//	auto v = m_pPrimaryCamera->GetViewMatrix(view);
		//	auto p = m_pPrimaryCamera->GetProjectionMatrix(view);
		//	RenderToView(v, p);
		//}
		//pRenderControl->EndFrame();
	});

	pDeviceResources->Present();
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
	RenderContext context(pContext);
	for (auto& pScene : Components)
	{
		auto pViewable = dynamic_cast<IViewable*>(pScene.get());
		if (pViewable)
		{
			pViewable->UpdateViewMatrix(view,projection);
			//pViewable->UpdateProjectionMatrix(projection);
		}
		auto pRenderable = pScene->As<IRenderable>();
		if (pRenderable)
			pRenderable->Render(context);
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

