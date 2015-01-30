////////////////////////////////////////////////////////////////////////////////
// Filename: D3DUnity.cpp
////////////////////////////////////////////////////////////////////////////////
//#pragma comment(lib, "dxgi.lib")
//#pragma comment(lib, "d3d11.lib")
//#pragma comment(lib, "dxguid.lib") 
#include "stdafx.h"
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d11.lib")
#include "D3DUnity.h"

D3DUnity::D3DUnity()
{
	m_swapChain = 0;
	m_device = 0;
	m_deviceContext = 0;
	m_pDXGIDebug = nullptr;
	m_pD3D11Debug = nullptr;
	//m_renderTargetView = 0;
	//m_depthStencilBuffer = 0;
//	m_depthStencilState = 0;
	//m_depthStencilView = 0;
//	m_rasterState = 0;
}


D3DUnity::D3DUnity(const D3DUnity& other)
{
}


D3DUnity::~D3DUnity()
{
	Shutdown();
}


bool D3DUnity::Initialize(int screenWidth, int screenHeight, bool vsync, HWND hwnd, bool fullscreen)
{
	HRESULT result;
	IDXGIFactory* factory;
	IDXGIAdapter* adapter;
	IDXGIOutput* adapterOutput;
	unsigned int numModes, i, numerator, denominator, stringLength;
	DXGI_MODE_DESC* displayModeList;
	DXGI_ADAPTER_DESC adapterDesc;
	int error;
	DXGI_SWAP_CHAIN_DESC swapChainDesc;
	ID3D11Texture2D* backBufferPtr;
	D3D11_VIEWPORT viewport;


	// Store the vsync setting.
	m_vsync_enabled = vsync;

	// Create a DirectX graphics interface factory.
	result = CreateDXGIFactory(__uuidof(IDXGIFactory), (void**)&factory);
	DirectX::ThrowIfFailed(result);

#if defined(_DEBUG)
	{
		typedef HRESULT(__stdcall *fPtr)(const IID&, void**); 
		HMODULE hDll = GetModuleHandleW(L"dxgidebug.dll"); 
		fPtr DXGIGetDebugInterface = (fPtr)GetProcAddress(hDll, "DXGIGetDebugInterface"); 
 
		DXGIGetDebugInterface(__uuidof(IDXGIDebug), (void**)&m_pDXGIDebug); 
	}
#endif
	// Use the factory to create an adapter for the primary graphics interface (video card).
	result = factory->EnumAdapters(0, &adapter);
	DirectX::ThrowIfFailed(result);

	// Enumerate the primary adapter output (monitor).
	result = adapter->EnumOutputs(0, &adapterOutput);
	DirectX::ThrowIfFailed(result);

	// Get the number of modes that fit the DXGI_FORMAT_R8G8B8A8_UNORM display format for the adapter output (monitor).
	result = adapterOutput->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_ENUM_MODES_INTERLACED, &numModes, NULL);
	DirectX::ThrowIfFailed(result);

	// Create a list to hold all the possible display modes for this monitor/video card combination.
	displayModeList = new DXGI_MODE_DESC[numModes];
	if(!displayModeList)
		return false;

	// Now fill the display mode list structures.
	result = adapterOutput->GetDisplayModeList(DXGI_FORMAT_R8G8B8A8_UNORM, DXGI_ENUM_MODES_INTERLACED, &numModes, displayModeList);
	DirectX::ThrowIfFailed(result);

	// Now go through all the display modes and find the one that matches the screen width and height.
	// When a match is found store the numerator and denominator of the refresh rate for that monitor.
	for(i=0; i<numModes; i++)
	{
		if(displayModeList[i].Width == (unsigned int)screenWidth)
		{
			if(displayModeList[i].Height == (unsigned int)screenHeight)
			{
				numerator = displayModeList[i].RefreshRate.Numerator;
				denominator = displayModeList[i].RefreshRate.Denominator;
			}
		}
	}

	// Get the adapter (video card) description.
	result = adapter->GetDesc(&adapterDesc);
	DirectX::ThrowIfFailed(result);

	// Store the dedicated video card memory in megabytes.
	m_videoCardMemory = (int)(adapterDesc.DedicatedVideoMemory / 1024 / 1024);

	// Convert the name of the video card to a character array and store it.
	error = wcstombs_s(&stringLength, m_videoCardDescription, 128, adapterDesc.Description, 128);
	if(error != 0)
		return false;

	// Release the display mode list.
	delete [] displayModeList;
	displayModeList = 0;

	// Release the adapter output.
	adapterOutput->Release();
	adapterOutput = 0;

	// Release the adapter.
	adapter->Release();
	adapter = 0;

	// Release the factory.
	factory->Release();
	factory = 0;

	// Initialize the swap chain description.
	ZeroMemory(&swapChainDesc, sizeof(swapChainDesc));

	// Set to a single back buffer.
	swapChainDesc.BufferCount = 1;

	// Set the width and height of the back buffer.
	swapChainDesc.BufferDesc.Width = screenWidth;
	swapChainDesc.BufferDesc.Height = screenHeight;

	// Set regular 32-bit surface for the back buffer.
	swapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;

	// Set the refresh rate of the back buffer.
	if(m_vsync_enabled)
	{
		swapChainDesc.BufferDesc.RefreshRate.Numerator = numerator;
		swapChainDesc.BufferDesc.RefreshRate.Denominator = denominator;
	}
	else
	{
		swapChainDesc.BufferDesc.RefreshRate.Numerator = 0;
		swapChainDesc.BufferDesc.RefreshRate.Denominator = 1;
	}

	// Set the usage of the back buffer.
	swapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;

	// Set the handle for the window to render to.
	swapChainDesc.OutputWindow = hwnd;

	// Turn multisampling off.
	swapChainDesc.SampleDesc.Count = 1;
	swapChainDesc.SampleDesc.Quality = 0;

	// Set to full screen or windowed mode.
	if(fullscreen)
	{
		swapChainDesc.Windowed = false;
	}
	else
	{
		swapChainDesc.Windowed = true;
	}

	// Set the scan line ordering and scaling to unspecified.
	swapChainDesc.BufferDesc.ScanlineOrdering = DXGI_MODE_SCANLINE_ORDER_UNSPECIFIED;
	swapChainDesc.BufferDesc.Scaling = DXGI_MODE_SCALING_UNSPECIFIED;

	// Discard the back buffer contents after presenting.
	swapChainDesc.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

	// Don't set the advanced flags.
	swapChainDesc.Flags = 0;

	// Set the feature level to DirectX 11.
	//	featureLevel = D3D_FEATURE_LEVEL_11_1;
	D3D_FEATURE_LEVEL FeatureLevels[4] = {D3D_FEATURE_LEVEL_11_1,D3D_FEATURE_LEVEL_11_0,D3D_FEATURE_LEVEL_10_1,D3D_FEATURE_LEVEL_10_0};

	UINT DeviceFlag = D3D11_CREATE_DEVICE_SWITCH_TO_REF;//D3D11_CREATE_DEVICE_SINGLETHREADED;

#ifdef _DEBUG
	DeviceFlag = D3D11_CREATE_DEVICE_DEBUG;
#endif
	// Create the swap chain, Direct3D device, and Direct3D device context.
	result = D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, 0, FeatureLevels, DeviceFlag, 
										   D3D11_SDK_VERSION, &swapChainDesc, &m_swapChain, &m_device, NULL, &m_deviceContext);

	//result = D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_REFERENCE, NULL, 0, &featureLevel, 1, 
	//									   D3D11_SDK_VERSION, &swapChainDesc, &m_swapChain, &m_device, NULL, &m_deviceContext);
	DirectX::ThrowIfFailed(result);

#if defined(_DEBUG)
	{
	D3D_SET_OBJECT_NAME_A(m_device,"pDevice");
	D3D_SET_OBJECT_NAME_A(m_deviceContext,"pDeviceContext");
	//result =  m_device->QueryInterface<ID3D11Debug>(&m_pD3D11Debug);
	//DirectX::ThrowIfFailed(result);
	}
#endif

	// Get the pointer to the back buffer.
	result = m_swapChain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&backBufferPtr);
	DirectX::ThrowIfFailed(result);

	ID3D11RenderTargetView *pRenderTargetView;
	// Create the render target view with the back buffer pointer.
	result = m_device->CreateRenderTargetView(backBufferPtr, NULL, &pRenderTargetView);
	DirectX::ThrowIfFailed(result);

	m_pBackBuffer.reset(new DirectX::RenderTargetTexture2D(backBufferPtr,pRenderTargetView,nullptr));
	backBufferPtr->Release();
	pRenderTargetView->Release();

	m_pDefaultDepthStencilBuffer.reset( new DirectX::DepthStencilBuffer(m_device,screenWidth,screenHeight));

	// Initialize the description of the depth buffer.
	//ZeroMemory(&depthBufferDesc, sizeof(depthBufferDesc));

	//// Set up the description of the depth buffer.
	//depthBufferDesc.Width = screenWidth;
	//depthBufferDesc.Height = screenHeight;
	//depthBufferDesc.MipLevels = 1;
	//depthBufferDesc.ArraySize = 1;
	//depthBufferDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	//depthBufferDesc.SampleDesc.Count = 1;
	//depthBufferDesc.SampleDesc.Quality = 0;
	//depthBufferDesc.Usage = D3D11_USAGE_DEFAULT;
	//depthBufferDesc.BindFlags = D3D11_BIND_DEPTH_STENCIL;
	//depthBufferDesc.CPUAccessFlags = 0;
	//depthBufferDesc.MiscFlags = 0;

	//// Create the texture for the depth buffer using the filled out description.
	//result = m_device->CreateTexture2D(&depthBufferDesc, NULL, &m_depthStencilBuffer);
	//DirectX::ThrowIfFailed(result);

	//// Initialize the depth stencil view.
	//ZeroMemory(&depthStencilViewDesc, sizeof(depthStencilViewDesc));

	//// Set up the depth stencil view description.
	//depthStencilViewDesc.Format = DXGI_FORMAT_D24_UNORM_S8_UINT;
	//depthStencilViewDesc.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2D;
	//depthStencilViewDesc.Texture2D.MipSlice = 0;

	//// Create the depth stencil view.
	//result = m_device->CreateDepthStencilView(m_depthStencilBuffer, &depthStencilViewDesc, &m_depthStencilView);
	//DirectX::ThrowIfFailed(result);

	// Bind the render target view and depth stencil buffer to the output render pipeline.
	m_deviceContext->OMSetRenderTargets(1, &pRenderTargetView, m_pDefaultDepthStencilBuffer->DepthStencilView());
//	m_deviceContext->OMSetRenderTargets(1, &m_renderTargetView, m_depthStencilView);

	m_pStates.reset(new DirectX::CommonStates(m_device));
	TurnOffWireframe();
	TurnZBufferOn();
	TurnOnAlphaBlending();
//	TurnOffAlphaBlending();


	// Setup the viewport for rendering.
	viewport.Width = (float)screenWidth;
	viewport.Height = (float)screenHeight;
	viewport.MinDepth = 0.0f;
	viewport.MaxDepth = 1.0f;
	viewport.TopLeftX = 0.0f;
	viewport.TopLeftY = 0.0f;

	// Create the viewport.
	m_deviceContext->RSSetViewports(1, &viewport);

	// Initialize the description of the stencil state.
//	ZeroMemory(&depthStencilDesc, sizeof(depthStencilDesc));
//
//	// Set up the description of the stencil state.
//	depthStencilDesc.DepthEnable = true;
//	depthStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
////	depthStencilDesc.DepthFunc = D3D11_COMPA
//	depthStencilDesc.DepthFunc = D3D11_COMPARISON_LESS;
//
//	depthStencilDesc.StencilEnable = false;
//	depthStencilDesc.StencilReadMask = 0xFF;
//	depthStencilDesc.StencilWriteMask = 0xFF;
//
//	// Stencil operations if pixel is front-facing.
//	depthStencilDesc.FrontFace.StencilFailOp = D3D11_STENCIL_OP_KEEP;
//	depthStencilDesc.FrontFace.StencilDepthFailOp = D3D11_STENCIL_OP_INCR;
//	depthStencilDesc.FrontFace.StencilPassOp = D3D11_STENCIL_OP_KEEP;
//	depthStencilDesc.FrontFace.StencilFunc = D3D11_COMPARISON_ALWAYS;
////	D3D11_COMPARISON_FUNC::
//	// Stencil operations if pixel is back-facing.
//	depthStencilDesc.BackFace.StencilFailOp = D3D11_STENCIL_OP_KEEP;
//	depthStencilDesc.BackFace.StencilDepthFailOp = D3D11_STENCIL_OP_DECR;
//	depthStencilDesc.BackFace.StencilPassOp = D3D11_STENCIL_OP_KEEP;
//	depthStencilDesc.BackFace.StencilFunc = D3D11_COMPARISON_ALWAYS;
//
//	// Create the depth stencil state.
//	result = m_device->CreateDepthStencilState(&depthStencilDesc, &m_depthStencilState);
//	DirectX::ThrowIfFailed(result);
//
//	// Set the depth stencil state.
//	m_deviceContext->OMSetDepthStencilState(m_depthStencilState, 1);
//
//
//	// Setup the raster description which will determine how and what polygons will be drawn.
//	rasterDesc.AntialiasedLineEnable = true;
////	rasterDesc.CullMode = D3D11_CULL_NONE;
//	rasterDesc.CullMode = D3D11_CULL_BACK;
//	rasterDesc.DepthBias = 0;
//	rasterDesc.DepthBiasClamp = 0.0f;
//	rasterDesc.DepthClipEnable = true;
//	rasterDesc.FillMode = D3D11_FILL_SOLID;
//	rasterDesc.FrontCounterClockwise = false;
//	rasterDesc.MultisampleEnable = true;
//	rasterDesc.ScissorEnable = false;
//	rasterDesc.SlopeScaledDepthBias = 0.0f;
//
//	// Create the rasterizer state from the description we just filled out.
//	result = m_device->CreateRasterizerState(&rasterDesc, &m_rasterState);
//	DirectX::ThrowIfFailed(result);
//
////	rasterDesc.CullMode = D3D11_CULL_NONE;
//	rasterDesc.FillMode = D3D11_FILL_WIREFRAME;
//
//	result = m_device->CreateRasterizerState(&rasterDesc, &m_rasterState_Wireframe);
//	DirectX::ThrowIfFailed(result);
//
//	// Now set the rasterizer state.
//	m_deviceContext->RSSetState(m_rasterState);
//	
//	// Setup the viewport for rendering.
//    viewport.Width = (float)screenWidth;
//    viewport.Height = (float)screenHeight;
//    viewport.MinDepth = 0.0f;
//    viewport.MaxDepth = 1.0f;
//    viewport.TopLeftX = 0.0f;
//    viewport.TopLeftY = 0.0f;
//
//	// Create the viewport.
//    m_deviceContext->RSSetViewports(1, &viewport);
//
//	//Added for 2D rendering & alpha blending
//	// Clear the second depth stencil state before setting the parameters.
//	ZeroMemory(&depthDisabledStencilDesc, sizeof(depthDisabledStencilDesc));
//
//	// Now create a second depth stencil state which turns off the Z buffer for 2D rendering.  The only difference is 
//	// that DepthEnable is set to false, all other parameters are the same as the other depth stencil state.
//	depthDisabledStencilDesc.DepthEnable = true;
//	depthDisabledStencilDesc.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
//	depthDisabledStencilDesc.DepthFunc = D3D11_COMPARISON_LESS;
//	depthDisabledStencilDesc.StencilEnable = false;
//	depthDisabledStencilDesc.StencilReadMask = 0xFF;
//	depthDisabledStencilDesc.StencilWriteMask = 0xFF;
//	depthDisabledStencilDesc.FrontFace.StencilFailOp = D3D11_STENCIL_OP_KEEP;
//	depthDisabledStencilDesc.FrontFace.StencilDepthFailOp = D3D11_STENCIL_OP_KEEP;
//	depthDisabledStencilDesc.FrontFace.StencilPassOp = D3D11_STENCIL_OP_KEEP;
//	depthDisabledStencilDesc.FrontFace.StencilFunc = D3D11_COMPARISON_ALWAYS;
////	D3D11_COMPARISON_FUNC::
//	// Stencil operations if pixel is back-facing.
//	depthDisabledStencilDesc.BackFace.StencilFailOp = D3D11_STENCIL_OP_KEEP;
//	depthDisabledStencilDesc.BackFace.StencilDepthFailOp = D3D11_STENCIL_OP_KEEP;
//	depthDisabledStencilDesc.BackFace.StencilPassOp = D3D11_STENCIL_OP_KEEP;
//	depthDisabledStencilDesc.BackFace.StencilFunc = D3D11_COMPARISON_ALWAYS;
//
//	// Create the state using the device.
//	result = m_device->CreateDepthStencilState(&depthDisabledStencilDesc, &m_depthDisabledStencilState);
//	DirectX::ThrowIfFailed(result);
//
//	// Clear the blend state description.
//	ZeroMemory(&blendStateDescription, sizeof(D3D11_BLEND_DESC));
//
//	// Create an alpha enabled blend state description.
//	blendStateDescription.RenderTarget[0].BlendEnable = TRUE;
////	D3D11_BLEND
//    blendStateDescription.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
//    blendStateDescription.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
//    blendStateDescription.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
//    blendStateDescription.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
//    blendStateDescription.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ZERO;
//    blendStateDescription.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
//    blendStateDescription.RenderTarget[0].RenderTargetWriteMask = 0x0f;
//
//	// Create the blend state using the description.
//	result = m_device->CreateBlendState(&blendStateDescription, &m_alphaEnableBlendingState);
//	DirectX::ThrowIfFailed(result);
//
//	// Modify the description to create an alpha disabled blend state description.
//	blendStateDescription.RenderTarget[0].BlendEnable = FALSE;
//
//	// Create the blend state using the description.
//	result = m_device->CreateBlendState(&blendStateDescription, &m_alphaDisableBlendingState);
//	DirectX::ThrowIfFailed(result);
//
////	TurnOnAlphaBlending();
//	TurnOffAlphaBlending();
return true;
}

void D3DUnity::Shutdown()
{
	// Before shutting down set to windowed mode or when you release the swap chain it will throw an exception.
	if (m_pDXGIDebug)
	{
		//m_pDXGIDebug->ReportLiveObjects(DXGI_DEBUG_D3D11,DXGI_DEBUG_RLO_DETAIL);
		m_pDXGIDebug->Release();
	}


	if(m_swapChain)
	{
		m_swapChain->SetFullscreenState(false, NULL);
	}

	if(m_deviceContext)
	{
		m_deviceContext->ClearState();
		m_deviceContext->Release();
		m_deviceContext = 0;
	}

	if (m_pD3D11Debug)
	{
		m_pD3D11Debug->ReportLiveDeviceObjects(D3D11_RLDO_DETAIL);
		m_pD3D11Debug->Release();
	}


	if(m_device)
	{
		m_device->Release();
		m_device = 0;
	}

	if(m_swapChain)
	{
		m_swapChain->Release();
		m_swapChain = 0;
	}

	return;
}


void D3DUnity::BeginScene(float red, float green, float blue, float alpha)
{
	float color[4];


	// Setup the color to clear the buffer to.
	color[0] = red;
	color[1] = green;
	color[2] = blue;
	color[3] = alpha;

	DirectX::XMVECTOR Color = DirectX::XMVectorSet(red,green,blue,alpha);

	m_pBackBuffer->Clear(m_deviceContext,Color);
	m_pDefaultDepthStencilBuffer->Clear(m_deviceContext);
	//// Clear the back buffer.
	//m_deviceContext->ClearRenderTargetView(m_renderTargetView, color);
 //   
	//// Clear the depth buffer.
	//m_deviceContext->ClearDepthStencilView(m_depthStencilView, D3D11_CLEAR_DEPTH, 1.0f, 0);

	return;
}


void D3DUnity::EndScene()
{
	// Present the back buffer to the screen since rendering is complete.
	if(m_vsync_enabled)
	{
		// Lock to screen refresh rate.
		m_swapChain->Present(1, 0);
	}
	else
	{
		// Present as fast as possible.
		m_swapChain->Present(0, 0);
	}

	return;
}


ID3D11Device* D3DUnity::Device()
{
	return m_device;
}


ID3D11DeviceContext* D3DUnity::Context()
{
	return m_deviceContext;
}

void D3DUnity::GetVideoCardInfo(char* cardName, int& memory) const
{
	strcpy_s(cardName, 128, m_videoCardDescription);
	memory = m_videoCardMemory;
	return;
}

void D3DUnity::TurnZBufferOn()
{
	m_deviceContext->OMSetDepthStencilState(m_pStates->DepthDefault(),1);
//	m_deviceContext->OMSetDepthStencilState(m_depthStencilState, 1);
	return;
}

void D3DUnity::TurnZBufferOff()
{
	m_deviceContext->OMSetDepthStencilState(m_pStates->DepthNone(),1);
//	m_deviceContext->OMSetDepthStencilState(m_depthDisabledStencilState, 1);
	return;
}

void D3DUnity::TurnOnWireframe()
{
	m_deviceContext->RSSetState(m_pStates->Wireframe());
//	m_deviceContext->RSSetState(m_rasterState_Wireframe);
	return;
}

void D3DUnity::TurnOffWireframe()
{
	m_deviceContext->RSSetState(m_pStates->CullCounterClockwise());
//	m_deviceContext->RSSetState(m_rasterState);
	return;
}

void D3DUnity::TurnOnAlphaBlending()
{
	float blendFactor[4];
	

	// Setup the blend factor.
	blendFactor[0] = 0.0f;
	blendFactor[1] = 0.0f;
	blendFactor[2] = 0.0f;
	blendFactor[3] = 0.0f;
	
	// Turn on the alpha blending.
	m_deviceContext->OMSetBlendState(m_pStates->AlphaBlend(),blendFactor,0xffffffff);
//	m_deviceContext->OMSetBlendState(m_alphaEnableBlendingState, blendFactor, 0xffffffff);

	return;
}

void D3DUnity::TurnOffAlphaBlending()
{
	float blendFactor[4];
	

	// Setup the blend factor.
	blendFactor[0] = 0.0f;
	blendFactor[1] = 0.0f;
	blendFactor[2] = 0.0f;
	blendFactor[3] = 0.0f;
	
	// Turn off the alpha blending.
	m_deviceContext->OMSetBlendState(m_pStates->Opaque(),blendFactor,0xffffffff);
//	m_deviceContext->OMSetBlendState(m_alphaDisableBlendingState, blendFactor, 0xffffffff);

	return;
}


ID3D11DepthStencilView* D3DUnity::GetDepthStencilView()
{
	//return m_depthStencilView;
	return m_pDefaultDepthStencilBuffer->DepthStencilView();
}


void D3DUnity::SetRenderTarget(DirectX::RenderTargetTexture2D* pRenderTarget, DirectX::DepthStencilBuffer* pDepthStencilBuffer)
{
	if (!pRenderTarget)
		pRenderTarget = m_pBackBuffer.get();
	if (!pDepthStencilBuffer)
		pDepthStencilBuffer = m_pDefaultDepthStencilBuffer.get();
	auto pTargetView = pRenderTarget->RenderTargetView();
	m_deviceContext->OMSetRenderTargets(1, &pTargetView, pDepthStencilBuffer->DepthStencilView());
	auto viewPort = pRenderTarget->ViewPort();
	m_deviceContext->RSSetViewports(1,&viewPort);
}



void D3DUnity::SetBackBufferRenderTarget()
{
	// Bind the render target view and depth stencil buffer to the output render pipeline.
	//m_deviceContext->OMSetRenderTargets(1, &m_renderTargetView, m_depthStencilView);
	auto pTargetView = m_pBackBuffer->RenderTargetView();
	m_deviceContext->OMSetRenderTargets(1, &pTargetView, m_pDefaultDepthStencilBuffer->DepthStencilView());
	return;
}