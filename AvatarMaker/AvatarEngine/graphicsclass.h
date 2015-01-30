////////////////////////////////////////////////////////////////////////////////
// Filename: graphicsclass.h
////////////////////////////////////////////////////////////////////////////////
#ifndef _GRAPHICSCLASS_H_
#define _GRAPHICSCLASS_H_


///////////////////////
// MY CLASS INCLUDES //
///////////////////////
#include "MathHelper.h"
#include "d3dunity.h"
//#include "FW1FontWrapper.h"
#include "Carmera.h"
//#error AfterCameraClassH
#include "Lights.h"
#include "ModelInterface.h"
#include <Effects.h>
#include "FW1FontWrapper.h"
#include "RectangleDrawer.h"

/////////////
// GLOBALS //
/////////////
const bool FULL_SCREEN = false;
const bool VSYNC_ENABLED = true;
//const bool VSYNC_ENABLED = true;
const float SCREEN_DEPTH = 1000.0f;
const float SCREEN_NEAR = 0.1f;

////////////////////////////////////////////////////////////////////////////////
// Class name: GraphicsClass
////////////////////////////////////////////////////////////////////////////////
class GraphicsClass
	: public D3DUnity
{
public:
	//enum DrawConfig
	//{
	//	Draw_Floor = 1<<0,
	//	Draw_Skeleton = 1<<1,
	//	Draw_Avatar = 1<<2,
	//	Draw_Avatar_Animate = 1<<3,
	//	Draw_Visualization = 1<<4,
	//	Draw_Color_Palette = 1<<5,
	//	Draw_Model_Palette = 1<<6,
	//};
	enum DepthStencilMode{
		Default,
		None,
		Read,
	};

	enum BlendMode
	{
		Opaque,
		AlphaBlend,
	};

	enum RasterizerMode{
		Wireframe,
		CullNone,
		CullCounterClockwise,
		CullClockwise,
	};

	enum HUDConfig
	{
		HUD_SCAN_MODE = 0,
		HUD_MODEL_MODE = 1,
		HUD_PAINT_MODE = 2,
		HUD_ANIMATION_MODE = 3,
	};

	//enum VisualizeParts
	//{
	//	Part_Left = 0,
	//	Part_Right = 1,
	//	Part_Double = 0,
	//	Part_Count = 2,
	//};

	GraphicsClass();
	~GraphicsClass();

public:
	void BeginScene();
	void EndScene();

	void RenderModel(DirectX::IRenderObject* pModel , DirectX::IEffect* pEffect = nullptr);

	bool Initialize(int, int, HWND);
//	bool Initialize(int, int, HWND, SkeletonClass*, ColorPalette*);
	void Shutdown();

	void RenderText(const std::wstring& Text,float fontSize , float x , float y , DirectX::FXMVECTOR vColor , unsigned int Flags = 0 ,const std::wstring& Font = L"Arial");
	void RenderButton(const std::wstring& Text, float fontsize , float x, float y , float size , DirectX::FXMVECTOR background, DirectX::FXMVECTOR foreground);
//	bool Frame(unsigned SceneFlag = Draw_Floor , unsigned HUDFlag = 0);
	//bool ReloadAvatar(const vector<CPoint3D>& vertecis,const vector<STri2D>& triangles);
	//bool ReloadAvatarTexture(BitMap *bitmap);
	////Set *LocalTransformMatrix to NULL if you don't need local transform
	//bool ReloadVisualizePart(VisualizeParts Part,const vector<D3DXVECTOR3> &Vertices,const vector<pair<int,int>> &Lines,const COLOR &Color = green, const D3DXMATRIX * LocalTransformMatrix = NULL);
	//bool ReloadVisualization(VisualizeParts Part,const vector<D3DXVECTOR3> &Vertices,const vector<pair<int,int>> &Lines,const D3DXMATRIX * LocalTransformMatrix = NULL , const COLOR &Color = green);
	//bool ReloadVisualization(VisualizeParts Part,const vector<D3DXVECTOR3> &Vertices,const vector<TRIANGLE> &Trianles,const D3DXMATRIX * LocalTransformMatrix = NULL , const COLOR &Color = green);

//	AnimatedModelClass* GetAvatarModel();

	__declspec(property(get = GetCamera)) 
		const ICamera* Camera;

private:
	//bool RenderScene(unsigned SceneFlag);
	//bool RenderHUD(HUDConfig Mode , unsigned LeftPanel , unsigned RightPanel , unsigned CentrePanel);

	GraphicsClass(const GraphicsClass&);

public:
	//enum PiplineState
	//{
	//	Rasterizer		= 0x1,
	//	DepthSencil		= 0x2,
	//	Blend			= 0x4,
	//	RenderTarget	= 0x8,
	//	VertexShader	= 0x10,
	//	HullShader		= 0x20,
	//	DominShader		= 0x40,
	//	GeometryShader	= 0x80,
	//	PixelShader		= 0x100,
	//};
	void Push(unsigned int StateFlag = 0xffff);
	void Pop ();

	void SetBlendState(BlendMode mode);
	void SetDepthStencilState(DepthStencilMode mode);
	void SetRasterizerState(RasterizerMode mode);

private:
	int m_ScreenWidth,m_ScreenHeight;
//	D3DClass* m_D3D;
public:
	const ICamera* GetCamera() const{
		return m_Camera;
	}

	const LightsBuffer* GetLights() const{
		return m_Lights;
	}

	RasterizerMode Mode_Rasterizer;
	DepthStencilMode Mode_DepthStencil;
	BlendMode Mode_Blend;

	PerspectiveCamera* m_Camera;
	LightsBuffer* m_Lights;

	Microsoft::WRL::ComPtr<IFW1Factory> m_pFWFactory;
	Microsoft::WRL::ComPtr<IFW1FontWrapper> m_pFontWrapper;
	std::unique_ptr<RectDrawer> m_pRectDrawer;
	//One for left , one for right
	//BaseModelFactory* m_BaseModelFacorty;
	//BaseModel* m_Visualizations[Part_Count];
	//BaseModel* m_RectAngleBoard;

private:
//	const ColorPalette* m_pColorPalette; 
};

#endif