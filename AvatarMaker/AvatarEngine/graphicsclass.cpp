////////////////////////////////////////////////////////////////////////////////
// Filename: graphicsclass.cpp
////////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "graphicsclass.h"
#include "DXGIFormatHelper.h"

using namespace DirectX;
const static Vector4 PartsColor[]={
	Vector4(0.2f,0.8f,0.2f,1.0f),
	Vector4(0.2f,0.8f,0.2f,1.0f),
	Vector4(0.8f,0.2f,0.2f,1.0f),
	Vector4(0.8f,0.2f,0.2f,1.0f),
	Vector4(0.8f,0.2f,0.2f,1.0f),
	Vector4(0.8f,0.2f,0.2f,1.0f),
	Vector4(0.8f,0.8f,0.2f,1.0f),
	Vector4(0.8f,0.8f,0.2f,1.0f),
};

GraphicsClass::GraphicsClass()
{
	m_Lights = 0;
	m_Camera = 0;
	Mode_Rasterizer = CullCounterClockwise;
	Mode_DepthStencil = Default;
	Mode_Blend = Opaque;
}


GraphicsClass::GraphicsClass(const GraphicsClass& other)
{
}


GraphicsClass::~GraphicsClass()
{
	Shutdown();
}

bool GraphicsClass::Initialize(int screenWidth, int screenHeight, HWND hwnd)
{
	bool result;

	m_ScreenWidth = screenWidth;
	m_ScreenHeight = screenHeight;

	// Initialize the Direct3D object.
	result = D3DUnity::Initialize(screenWidth, screenHeight, VSYNC_ENABLED, hwnd, FULL_SCREEN);
	if(!result)
	{
		MessageBox(hwnd, L"Could not initialize Direct3D.", L"Error", MB_OK);
		return false;
	}

	// Create the camera object.
	// Ps.57 is the FOV of KINECT
	// ((float)screenHeight)/screenWidth
//	m_Camera = new PerspectiveCamera(Device(),0,75*DirectX::XM_PI/180.0F,(float)m_ScreenWidth/(float)m_ScreenHeight);
	m_Camera = new PerspectiveCamera(Device(),1,75*DirectX::XM_PI/180.0F,(float)m_ScreenWidth/(float)m_ScreenHeight);
	if(!m_Camera)
		return false;

	// Set the initial position of the camera.
	m_Camera->Position.Set(0.0f, 1.0f, 0.0f);
	m_Camera->Orientation = XMQuaternionIdentity();

	// Create the light object.
	m_Lights = new LightsBuffer(Device(),0);
	if(!m_Lights)
		return false;

	// Initialize the light object.
	m_Lights->AmbientLight.Color.Set(0.5f, 0.5f, 0.5f, 1.0f);
	m_Lights->DiffuseLight.Color.Set(0.4f, 0.4f, 0.4f, 1.0f);
	m_Lights->DiffuseLight.Direction.Set(-0.5f, -0.5f, 1.0f);
	m_Lights->DiffuseLight.Direction.Normalize();
	//m_Lights->SetSpecularColor(1.0f, 1.0f, 1.0f, 1.0f);
	//m_Lights->SetSpecularPower(64.0f);


	//HRESULT hr;

	//hr = FW1CreateFactory(FW1_VERSION,&m_FWFactory);
	//hr = m_FWFactory->CreateFontWrapper(Device(),L"Arial",&m_FontWrapper);
	//if (hr) {
	//	MessageBox(hwnd, L"Could not initialize the Font Wrapper.", L"Error", MB_OK);
	//	return false;
	//}

	return true;
}


void GraphicsClass::Shutdown()
{
	// Release the light object.
	if(m_Lights)
	{
		delete m_Lights;
		m_Lights = 0;
	}

	// Release the camera object.
	if(m_Camera)
	{
		delete m_Camera;
		m_Camera = 0;
	}

	// Release the D3D object.
	D3DUnity::Shutdown();
	return;
}

//bool GraphicsClass::Frame(unsigned SceneFlag , unsigned HUDFlag)
//{
//	bool result;
//
////	m_D3D->TurnOffAlphaBlending();
////	m_D3D->TurnOnAlphaBlending();
//
//	// Clear the buffers to begin the scene.
//	BeginScene(0.0f, 0.0f, 0.0f, 1.0f);
//
//	// Render the 3D scene.
//	result = RenderScene(SceneFlag);
//	if(!result)	return false;
//
//	// Render the 2D HUD
//	result = RenderHUD((HUDConfig)(HUDFlag & 0x3),(HUDFlag >> 2) & 0xf,(HUDFlag >> 6) & 0xf,(HUDFlag >> 10) & 0xf);
//	if(!result)	return false;
//
//	// Present the rendered scene to the screen.
//	if (result) 
//		m_D3D->EndScene();
//
////	m_D3D->TurnOffAlphaBlending();
//
//	return true;
//}

void GraphicsClass::BeginScene(){
	D3DUnity::BeginScene(0.0f, 0.0f, 0.0f, 1.0f);
}

void GraphicsClass::RenderModel(DirectX::IRenderObject* pModel , DirectX::IEffect* pEffect /* = nullptr*/)
{
	if (pModel == nullptr) return;
	m_Camera->Render(Context());
	m_Lights->EyePosition = m_Camera->Position;
	m_Lights->Render(Context());
	auto pBuffer = m_Lights->GetBuffer();
	m_deviceContext->VSSetConstantBuffers(2,1,&pBuffer);
	
	SetRasterizerState(Mode_Rasterizer);
	SetBlendState(Mode_Blend);
	SetDepthStencilState(Mode_DepthStencil);

	if (pEffect)
		pEffect->Apply(Context());

	pModel->Render(Context());
}


void GraphicsClass::EndScene(){
	D3DUnity::EndScene();
}

void GraphicsClass::SetRasterizerState( RasterizerMode mode )
{
	switch (mode)
	{
	case GraphicsClass::Wireframe:
		m_deviceContext->RSSetState(m_pStates->Wireframe());
		break;
	case GraphicsClass::CullNone:
		m_deviceContext->RSSetState(m_pStates->CullNone());
		break;
	case GraphicsClass::CullCounterClockwise:
		m_deviceContext->RSSetState(m_pStates->CullCounterClockwise());
		break;
	case GraphicsClass::CullClockwise:
		m_deviceContext->RSSetState(m_pStates->CullClockwise());
		break;
	default:
		break;
	}
}

void GraphicsClass::SetDepthStencilState( DepthStencilMode mode )
{
	switch (mode)
	{
	case GraphicsClass::Default:
		m_deviceContext->OMSetDepthStencilState(m_pStates->DepthDefault(),0);
		break;
	case GraphicsClass::None:
		m_deviceContext->OMSetDepthStencilState(m_pStates->DepthNone(),0);
		break;
	case GraphicsClass::Read:
		m_deviceContext->OMSetDepthStencilState(m_pStates->DepthRead(),0);
		break;
	default:
		break;
	}
}

void GraphicsClass::SetBlendState( BlendMode mode )
{
	ID3D11BlendState* blendState = nullptr;
	XMFLOAT4 BlendFactor;
	ZeroMemory(&BlendFactor,sizeof(BlendFactor));
	switch (mode)
	{
	case GraphicsClass::Opaque:
		blendState = m_pStates->Opaque();
		break;
	case GraphicsClass::AlphaBlend:
		blendState = m_pStates->AlphaBlend();
		break;
	default:
		break;
	}
	if (blendState)
		m_deviceContext->OMSetBlendState(blendState,(float*)&BlendFactor,0xFFFFFFFF);
}

void GraphicsClass::RenderText(const std::wstring& Text,float fontSize , float x , float y , FXMVECTOR vColor , unsigned int Flags/* = 0*/ ,const std::wstring& Font/* = L"Arial"*/)
{
	DirectX::PackedVector::XMCOLOR foreground;
	PackedVector::XMStoreColor(&foreground,vColor);
	m_pFontWrapper->DrawString(Context(),Text.data(),fontSize,x,y,foreground,FW1_VCENTER|FW1_CENTER);
}


void GraphicsClass::RenderButton(const std::wstring& Text, float fontsize , float x, float y , float size , DirectX::FXMVECTOR background, DirectX::FXMVECTOR foreground)
{
	const float margine = 5.0f;
	TurnOnAlphaBlending();
	TurnZBufferOff();
	TurnOffWireframe();
	m_pRectDrawer->DrawRect_Pixel(m_deviceContext,x - size/2,y - size/2,x + size/2,y + size/2,foreground);
	m_pRectDrawer->DrawRect_Pixel(m_deviceContext,x - size/2 + margine,y - size/2 + margine,x + size/2 -margine,y + size/2 -margine,background);
	RenderText(Text,fontsize,x,y,foreground,FW1_VCENTER|FW1_CENTER);
}

//
//bool GraphicsClass::RenderHUD(HUDConfig Mode , unsigned LeftPanel , unsigned RightPanel , unsigned CentrePanel){
//	const int margin = 20;
//	const int rectLength = 180;
//
//	bool result;
//
//	switch (Mode)
//	{
//	case GraphicsClass::HUD_SCAN_MODE:
//		{
//			m_FontWrapper->DrawString(Context(),L"Mode::Scanning",64,m_ScreenWidth / 2, 50 ,0xff5555aa,FW1_VCENTER|FW1_CENTER);
//		}
//		break;
//	//case GraphicsClass::HUD_ATTACH_MODE:
//	//	{
//	//		m_FontWrapper->DrawString(m_D3D->GetDeviceContext(),L"Mode::Scan",64,m_ScreenWidth / 2, 50 ,0xffffbbee,FW1_VCENTER|FW1_CENTER);
//	//	}
//	//	break;
//	case GraphicsClass::HUD_MODEL_MODE:
//		{
//			const WCHAR ButtonName[][10] = {L"Empty",L"Pinch",L"Push",L"Sculpt",L"Cut"};
//			UINT32 BackGrounds[] = {0xffcc44aa,0xffcc44aa,0xffcc44aa,0xffcc44aa};
//			UINT32 ForeGrounds[] = {0xffffbbee,0xffffbbee,0xffffbbee,0xffffbbee};
//
//			TurnZBufferOff();
//
//			if (LeftPanel>0)
//				DrawButton(Context(),ButtonName[LeftPanel],margin + rectLength/2,margin+rectLength/2,rectLength,0xffcc44aa,0xffffbbee,56);
//			else
//				DrawButton(Context(),ButtonName[LeftPanel],margin + rectLength/2,margin+rectLength/2,rectLength,0x88cc44aa,0x88ffbbee,56);
//
//			if (RightPanel>0)
//				DrawButton(Context(),ButtonName[RightPanel],m_ScreenWidth- (margin + rectLength/2),margin+rectLength/2,rectLength,0xffcc44aa,0xffffbbee,56);
//			else
//				DrawButton(Context(),ButtonName[RightPanel],m_ScreenWidth- (margin + rectLength/2),margin+rectLength/2,rectLength,0x88cc44aa,0x88ffbbee,56);
//
//			m_FontWrapper->DrawString(Context(),L"Mode::Modeling",64,m_ScreenWidth / 2, 50 ,0xffffbbee,FW1_VCENTER|FW1_CENTER);
//		}
//	break;
//	case GraphicsClass::HUD_PAINT_MODE:
//		{
//			//for (int i = 0; i < 8; i++)
//			//{
//			//	float Px = (m_pColorPalette->PigmentBox(i)->X_max + m_pColorPalette->PigmentBox(i)->X_min) / 2.0f;
//			//	float Py = (m_pColorPalette->PigmentBox(i)->Y_max + m_pColorPalette->PigmentBox(i)->Y_min) / 2.0f;
//			//	float Width = (m_pColorPalette->PigmentBox(i)->X_max - m_pColorPalette->PigmentBox(i)->X_min);
//			//	float Height = (m_pColorPalette->PigmentBox(i)->Y_max - m_pColorPalette->PigmentBox(i)->Y_min);
//
//			//	m_BaseModelFacorty->ReloadRectangleBoard(m_RectAngleBoard,m_D3D->GetDevice(),D3DXVECTOR3(Px,Py,0.0f),Width,Height,m_pColorPalette->PigmentColor(i));
//			//	m_RectAngleBoard->Render(m_D3D->GetDeviceContext(),&worldMatrix,&viewMatrix,&projectionMatrix);
//			//}
//
//			m_FontWrapper->DrawString(Context(),L"Mode::Painting",64,m_ScreenWidth / 2, 50 ,0xff55aa55,FW1_VCENTER|FW1_CENTER);
//		}
//		break;
//	case GraphicsClass::HUD_ANIMATION_MODE:
//		{
//			m_FontWrapper->DrawString(Context(),L"Mode::Attaching",64,m_ScreenWidth / 2, 50 ,0xffaa5555,FW1_VCENTER|FW1_CENTER);
////			m_FontWrapper->DrawString(Context(),L"Mode::Animation",64,m_ScreenWidth / 2, 50 ,0xffffbbee,FW1_VCENTER|FW1_CENTER);
//		}
//		break;
//	default:
//		break;
//	}
//	//Reset the RS & OM Stage to default
//	TurnZBufferOn();
//	TurnOffWireframe();
//	TurnOnAlphaBlending();
//	return true;
//}
//
//bool GraphicsClass::RenderScene(unsigned SceneFlag)
//{
//	D3DXMATRIX worldMatrix, viewMatrix, projectionMatrix, translateMatrix, scaleMatrix;
//	bool result;
//
//	// Generate the view matrix based on the camera's position.
//	m_Camera->Render();
//	m_Camera->GetViewMatrix(viewMatrix);
//	m_D3D->GetProjectionMatrix(projectionMatrix);
//	// Get the world, view, and projection matrices from the camera and d3d objects.
//
//	//Draw the floor
//	if (SceneFlag & Draw_Floor)
//	{
//		m_D3D->GetWorldMatrix(worldMatrix);
//		m_FloorModel->Render(m_D3D->GetDeviceContext());
//		m_ShaderManager->RenderBumpMapShader(m_D3D->GetDeviceContext(), m_FloorModel->GetIndexCount(), worldMatrix, viewMatrix, projectionMatrix, 
//						      m_FloorModel->GetColorTexture(), m_FloorModel->GetNormalMapTexture(), m_Light->GetDirection(), 
//						      m_Light->GetDiffuseColor());
//	}
//
//	if (SceneFlag & Draw_Skeleton){
//		m_D3D->GetWorldMatrix(worldMatrix);
//		D3DXMatrixTranslation(&translateMatrix, m_Skeleton->GetPosition().x,m_Skeleton->GetPosition().y,m_Skeleton->GetPosition().z);
//		D3DXMatrixMultiply(&worldMatrix, &translateMatrix, &worldMatrix); 
//		D3DXMatrixMultiply(&worldMatrix, &m_Skeleton->GetFacingMatrix(), &worldMatrix); 
//		m_SkeletonModel->Render(m_D3D->GetDevice(),m_D3D->GetDeviceContext(),m_Skeleton);
//		m_ShaderManager->RenderColorLineShader(m_D3D->GetDeviceContext(), m_SkeletonModel->GetIndexCount(), worldMatrix, viewMatrix, projectionMatrix);
//	}
//
//
////	 Setup the rotation and translation of the third model.
//	if (SceneFlag & Draw_Avatar){
//		m_D3D->GetWorldMatrix(worldMatrix);
//		if (SceneFlag & Draw_Avatar_Animate){
//			D3DXMatrixTranslation(&translateMatrix, m_Skeleton->GetPosition().x,m_Skeleton->GetPosition().y,m_Skeleton->GetPosition().z);
//			D3DXMatrixMultiply(&worldMatrix, &translateMatrix, &worldMatrix); 
//			D3DXMatrixMultiply(&worldMatrix, &m_Skeleton->GetFacingMatrix(), &worldMatrix); 
//		}
//
//		//Render the Avatar
//		m_AvatorModel->Render(m_D3D->GetDeviceContext());
//		result = m_ShaderManager->RenderAnimateShader(m_D3D->GetDeviceContext(), m_AvatorModel->GetIndexCount(), worldMatrix, viewMatrix, projectionMatrix, 
//										   m_AvatorModel->GetTexture(), m_Light->GetDirection(), m_Light->GetAmbientColor(), m_Light->GetDiffuseColor(), 
//										   m_Camera->GetPosition(), m_Light->GetSpecularColor(), m_Light->GetSpecularPower(),m_AvatorModel->GetSkeleton(),SceneFlag & Draw_Avatar_Animate);
//		if(!result) 
//			return false;
//		D3DXMATRIX localMatrix,buff;
//
//		if (SceneFlag & Draw_Visualization) {
////			m_D3D->TurnOnWireframe();
////			m_RectAngleBoard->Render(m_D3D->GetDeviceContext(),&worldMatrix,&viewMatrix,&projectionMatrix);
////			m_BaseModelFacorty->ReloadRectangleBoard(m_Visualizations[Part_Left],m_D3D->GetDevice(),D3DXVECTOR3(0.0f,0.0f,0.0f),5,5,white);
//			m_Visualizations[Part_Left]->Render(m_D3D->GetDeviceContext(),&worldMatrix,&viewMatrix,&projectionMatrix);
//			m_Visualizations[Part_Right]->Render(m_D3D->GetDeviceContext(),&worldMatrix,&viewMatrix,&projectionMatrix);
////			m_D3D->TurnOffWireframe();
//
//		}
//
//		//if (SceneFlag & Draw_Color_Palette){
//		//	for (int i = 0; i < 8; i++)
//		//	{
//		//		D3DXVECTOR3 tp = m_ColorSpheres[i]->Position();
//		//		D3DXMatrixTranslation(&worldMatrix,tp.x,tp.y,tp.z);
//		//		m_ColorSpheres[i]->Render(m_D3D->GetDeviceContext());
//		//		m_ShaderManager->RenderColorShader(m_D3D->GetDeviceContext(), m_ColorSpheres[i]->GetIndexCount(), worldMatrix, viewMatrix, projectionMatrix);
//		//	}
//		//}
//
//	}
//
//	return true;
//}
//
//bool GraphicsClass::ReloadAvatar(const vector<CPoint3D>& vertices,const vector<STri2D>& triangles){
//	bool hr;
//	hr = m_AvatorModel->Refresh(m_D3D->GetDevice(),vertices,triangles);
//	return hr;
//}
//
//bool GraphicsClass::ReloadAvatarTexture(BitMap *bitmap){
//	bool hr;
//	hr = m_AvatorModel->GetDynamicTexture()->Update(m_D3D->GetDeviceContext(), bitmap);
//	return hr;
//}

//bool GraphicsClass::ReloadVisualizePart(VisualizeParts Part,const vector<D3DXVECTOR3> &Vertices,const vector<pair<int,int>> &Lines,const COLOR &Color,const D3DXMATRIX * LocalTransformMatrix){
//	bool hr;
//	hr = m_VisualizeParts[Part]->Reload(m_D3D->GetDevice(),Vertices,Lines,Color);
//	//Skeleton->GetBindingBoneTransformMatrix((SkeletonClass::HumanBoneEnum)m_Geometry->m_PinchManagers[GeometryManager::LeftHand].BindingBone())
//	if (LocalTransformMatrix)
//		hr &= m_VisualizeParts[Part]->UpdateLoaclMatrix(*LocalTransformMatrix);
//	else {
//		D3DXMATRIX Identity;
//		D3DXMatrixIdentity(&Identity);
//		hr &= m_VisualizeParts[Part]->UpdateLoaclMatrix(Identity);
//	}
//
//	return hr;
//}
//
//bool GraphicsClass::ReloadVisualization(VisualizeParts Part,const vector<D3DXVECTOR3> &Vertices,const vector<TRIANGLE> &Triangles,const D3DXMATRIX * LocalTransformMatrix, const COLOR &Color){
//	bool hr;
//	COLOR TransparentColor = Color;
//	TransparentColor.w = 0.5;
//	hr = m_Visualizations[Part]->Reload(m_D3D->GetDevice(),Vertices,Triangles,TransparentColor,LocalTransformMatrix);
//	if (Part == Part_Left) 
//		m_RectAngleBoard->Reload(m_D3D->GetDevice(),Vertices,Triangles,TransparentColor,LocalTransformMatrix);
//
//	return hr;
//}
//
//bool GraphicsClass::ReloadVisualization(VisualizeParts Part,const vector<D3DXVECTOR3> &Vertices,const vector<pair<int,int>> &Lines,const D3DXMATRIX * LocalTransformMatrix, const COLOR &Color){
//	bool hr;
//	COLOR TransparentColor = Color;
//	TransparentColor.w = 0.5;
//	hr = m_Visualizations[Part]->Reload(m_D3D->GetDevice(),Vertices,Lines,TransparentColor,LocalTransformMatrix);
//	return hr;
//}
