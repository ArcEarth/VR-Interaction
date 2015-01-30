////////////////////////////////////////////////////////////////////////////////
// Filename: D3DUnity.h
////////////////////////////////////////////////////////////////////////////////
#ifndef _D3DUnity_H_
#define _D3DUnity_H_


/////////////
// LINKING //
/////////////
//#pragma comment(lib, "d3d11_1.lib")
//#pragma comment(lib, "d3dx11.lib")
//#pragma comment(lib, "d3dx10.lib")


//////////////
// INCLUDES //
//////////////
//#include <dxgi.h>
//#include <d3dcommon.h>
#include <d3d11_1.h>
#include "DirectXHelper.h"
#include "CommonStates.h"
#include "Textures.h"
#include <dxgidebug.h>

////////////////////////////////////////////////////////////////////////////////
// Class name: D3DUnity
////////////////////////////////////////////////////////////////////////////////
class D3DUnity
{
public:
	D3DUnity();
	D3DUnity(const D3DUnity&);
	~D3DUnity();

	bool Initialize(int, int, bool, HWND, bool);
	void Shutdown();
	
	void BeginScene(float, float, float, float);
	void EndScene();

	ID3D11Device* Device();
	ID3D11DeviceContext* Context();

	void GetVideoCardInfo(char*, int&) const;

	DirectX::CommonStates *GetStateObject();

	void TurnZBufferOn();
	void TurnZBufferOff();

	void TurnOnAlphaBlending();
	void TurnOffAlphaBlending();

	void TurnOnWireframe();
	void TurnOffWireframe();

	ID3D11DepthStencilView* GetDepthStencilView();

	void SetRenderTarget(DirectX::RenderTargetTexture2D* pRenderTarget, DirectX::DepthStencilBuffer* pDepthStencilBuffer);
	void SetBackBufferRenderTarget();

protected:
	std::unique_ptr<DirectX::CommonStates> m_pStates;
	ID3D11Device*				m_device;
	ID3D11DeviceContext*		m_deviceContext;
	IDXGIDebug*					m_pDXGIDebug;
	ID3D11Debug*				m_pD3D11Debug;

private:
	bool m_vsync_enabled;
	int m_videoCardMemory;
	char m_videoCardDescription[128];
	IDXGISwapChain* m_swapChain;

	std::unique_ptr<DirectX::RenderTargetTexture2D> m_pBackBuffer;
	std::unique_ptr<DirectX::DepthStencilBuffer>	m_pDefaultDepthStencilBuffer;
	//ID3D11RenderTargetView* m_renderTargetView;
	//ID3D11Texture2D* m_depthStencilBuffer;
	//ID3D11DepthStencilView* m_depthStencilView;

};

#endif