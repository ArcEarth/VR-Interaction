////////////////////////////////////////////////////////////////////////////////
// Filename: systemclass.cpp
////////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "SystemControler_Fundition.h"

/////////////
// GLOBALS //
/////////////
using namespace std;
using namespace DirectX;
using namespace Kinematics;
using namespace Geometrics;
using namespace EditingTools;
const SystemControler::ToolsSet SystemControler::ToolsForSingleGestures[5] = {
	Cursor, // FREE
	Drag,	// PINCH
	Sweep,	// SLAP
	Cut,	// FINGERGUN
	Grow,	// FIST
};

const SystemControler::ToolsSet SystemControler::ToolsForDoubleGestures[5] = {
	Cursor, // DOUBLE FREE
	Drag,	// DOUBLE PINCH
	Vculpt,	// DOUBLE SLAP
	Drive,	// DOUBLE FINGERGUN
	Grow,	// DOUBLE FIST		
};

const SystemControler::ToolsSet SystemControler::ToolsForSpetialEditing[5] = {
	Cursor,
	Drag,
	Cursor,
	Cut,
	Cursor,
};

const SystemControler::ToolsSet SystemControler::ToolsForSpetialEditingDoubleGesture[5] = {
	Cursor,
	Drag,
	Cursor,
	Cut,
	Manipulate,
};

const SystemControler::ToolsSet SystemControler::ToolsForPaintGestures[5] = {
	HoverPickCursor,	// FREE
	GaussBrush,			// PINCH
	GaussBrush,			// SLAP
	GaussBrush,			// FINGERGUN
	FillBrush,			// FIST
};

//const SystemControler::ToolsSet SystemControler::ToolsForConstraints[][5] = {
//	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_None , This palette shouldn't be used
//	{Cursor,Drag,Drag,Drag,Drag},			//Constraint_Drag_Only
//	{Cursor,Sweep,Sweep,Sweep,Sweep},	//Constraint_Sculpt_Only
//	{Cursor,Cut,Cut,Cut,Cut},				//Constraint_Cut_Only
//	{Cursor,Grow,Grow,Grow,Grow},			//Constraint_Grow_Only
//	{HoverPickCursor,GaussBrush,GaussBrush,GaussBrush,GaussBrush},		//Constraint_Paint_Only
//	{HoverPickCursor,FillBrush,FillBrush,FillBrush,FillBrush},			//Constraint_Fill_Only
//	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_Double_Sculpt_Only,
//	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_Scan_Only,
//	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_Manipulate_Only,
//};

//const SystemControler::ToolsSet SystemControler::ToolsForDoubleSculptConstraint[5] = {
//	Cursor,Vculpt,Vculpt,Vculpt,Vculpt};

//const SystemControler::ToolsSet SystemControler::ToolsForSpetialManipulateConstraint[5] = {
//	Cursor,Cursor,Cursor,Cursor,Manipulate};

const SystemControler::ToolsSet SystemControler::ToolsForConstraints[][5] = {
	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_None , This palette shouldn't be used
	{Cursor,Drag,Cursor,Cursor,Cursor},			//Constraint_Drag_Only
	{Cursor,Cursor,Sweep,Cursor,Cursor},	//Constraint_Sculpt_Only
	{Cursor,Cursor,Cursor,Cut,Cursor},				//Constraint_Cut_Only
	{Cursor,Cursor,Cursor,Cursor,Grow},			//Constraint_Grow_Only
	{HoverPickCursor,GaussBrush,GaussBrush,GaussBrush,HoverPickCursor},		//Constraint_Paint_Only
	{HoverPickCursor,HoverPickCursor,HoverPickCursor,HoverPickCursor,FillBrush},			//Constraint_Fill_Only
	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_Double_Sculpt_Only,
	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_Scan_Only,
	{Cursor,Cursor,Cursor,Cursor,Cursor},	//Constraint_Manipulate_Only,
};

const SystemControler::ToolsSet SystemControler::ToolsForDoubleSculptConstraint[5] = {
	Cursor,Cursor,Vculpt,Cursor,Cursor};

const SystemControler::ToolsSet SystemControler::ToolsForSpetialManipulateConstraint[5] = {
	Cursor,Cursor,Cursor,Cursor,Manipulate};

const static wstring ConstraintName[] = {
	L"",L"Only Drag",L"Only Sweep",L"Only Cut",L"Only Grow",L"Only Paint",L"Only Fill Color",L"Only Double Sweep",L"Only Scan",L"Only Manipulate"};


const bool SystemControler::IsConstTools[]=
{
	true,	//Cursor",
	false,	//Drag",
	false,	//Grow",
	false,	//Sweep",
	false,	//Vculpt",
	false,	//Cut",
	true,	//Sketch",
	true,	//Drive",
	true,	//HoverPickCursor",
	true,	//Eyedropper",
	false,	//GaussBrush",
	false,	//FillBrush"
	false,	//Manipulate
};

const SystemControler::SideEffectType SystemControler::ToolsSideEffectType[] =
{
	SideEffect_None,	//Cursor",
	SideEffect_OnConfirm,	//Drag",
	SideEffect_OnConfirm,	//Grow",
	SideEffect_OnConfirm,	//Sweep",
	SideEffect_OnConfirm,	//Vculpt",
	SideEffect_OnEditing,	//Cut",
	SideEffect_None,	//Sketch",
	SideEffect_None,	//Drive",
	SideEffect_None,	//HoverPickCursor",
	SideEffect_None,	//Eyedropper",
	SideEffect_OnEditing,	//GaussBrush",
	SideEffect_OnEditing,	//FillBrush"
	SideEffect_OnEditing,	//Manipulate
};



//static const char* HandSwitchName[]={ToString(LeftHand),ToString(RightHand),ToString(DoubleHand)};

const static std::wstring ConfigFileName	= L"Data\\InputMouseConfig.ini";
const static std::string ShadersPath		= "..\\Debug\\";
const static std::wstring TexturePath		= L"Data\\Textures\\";
const static std::wstring SavePath			= L"Save\\";
const static std::wstring SaveFileExtension = L".sav";
#ifdef _DEBUG
const static char VertexShaderFile[]				=	"..\\Debug\\VS_Skinning.cso";
const static char PixelShaderFile[]					=	"..\\Debug\\PS_ColorTexture_PixelLights.cso";
const static char IndicatorVertexShaderFile[]		=	"..\\Debug\\VS_AttachIndicate.cso";
const static char IndicatorPixelShaderFile[]		=	"..\\Debug\\PS_VertexColor.cso";
const static char SkyBoxVertexShaderFile[]			=	"..\\Debug\\VS_SkyBox.cso";
const static char SkyBoxPixelShaderFile[]			=	"..\\Debug\\PS_PosTex.cso";
#else
//const static char VertexShaderFile[]				=	"..\\Release\\VS_Skinning.cso";
//const static char PixelShaderFile[]					=	"..\\Release\\PS_ColorTexture_PixelLights.cso";
//const static char IndicatorVertexShaderFile[]		=	"..\\Release\\VS_AttachIndicate.cso";
//const static char IndicatorPixelShaderFile[]		=	"..\\Release\\PS_VertexColor.cso";
//const static char SkyBoxVertexShaderFile[]			=	"..\\Release\\VS_SkyBox.cso";
//const static char SkyBoxPixelShaderFile[]			=	"..\\Release\\PS_PosTex.cso";
const static char VertexShaderFile[]				=	"Data\\Shaders\\VS_Skinning.cso";
const static char PixelShaderFile[]					=	"Data\\Shaders\\PS_ColorTexture_PixelLights.cso";
const static char IndicatorVertexShaderFile[]		=	"Data\\Shaders\\VS_AttachIndicate.cso";
const static char IndicatorPixelShaderFile[]		=	"Data\\Shaders\\PS_VertexColor.cso";
const static char SkyBoxVertexShaderFile[]			=	"Data\\Shaders\\VS_SkyBox.cso";
const static char SkyBoxPixelShaderFile[]			=	"Data\\Shaders\\PS_PosTex.cso";
#endif
const static wstring SkyBoxTextures[6] = {
	wstring(L"data\\Textures\\SkyBox\\GrimmNight\\Right.dds"),
	wstring(L"data\\Textures\\SkyBox\\GrimmNight\\Left.dds"),
	wstring(L"data\\Textures\\SkyBox\\GrimmNight\\Top.dds"),
	wstring(L"data\\Textures\\SkyBox\\GrimmNight\\Bottom.dds"),
	wstring(L"data\\Textures\\SkyBox\\GrimmNight\\Front.dds"),
	wstring(L"data\\Textures\\SkyBox\\GrimmNight\\Back.dds"),
};
const static wstring FloorTexture(L"data\\Textures\\CheckerBoard.dds");

SystemControler::SystemControler()
{
	//ZeroMemory(this,sizeof(SystemControler));
	//m_pInput = nullptr;
	m_pGraphics = nullptr;
	m_pAvatar = nullptr;
	m_ScalingFlag = false;
	m_Paused = false;
	m_Constraint = Constraint_None;

	SkeletonTransparency = 0.2f;

	memset(m_Slots,0,sizeof(m_Slots));
	Initialize();
}

SystemControler::SystemControler(const SystemControler& other)
{
	throw "Please don't copy system class";
}

SystemControler::~SystemControler()
{
	Shutdown();
}

bool SystemControler::Initialize()
{
	bool result;

	wchar_t buffer[256];
	GetCurrentDirectory(256,buffer);
	CurrentDirectory = buffer;
	CurrentDirectory += L'\\';

	m_hConsole = CreateDebugConsoleWindow();

	// Initialize the windows API.
	InitializeWindows(screenWidth, screenHeight);

	// Create the graphics object.  This object will handle rendering all the graphics for this application.
	m_pGraphics = new GraphicsClass;
	if(!m_pGraphics) return false;
	result = m_pGraphics->Initialize(screenWidth, screenHeight, m_hwnd);
	if(!result) return false;
	m_pGraphics->Mode_Blend = GraphicsClass::BlendMode::AlphaBlend;
	ID3D11Device* pDevice = m_pGraphics->Device();
	ID3D11DeviceContext* pContext = m_pGraphics->Context();
	const ICamera* pCamera = m_pGraphics->Camera; 
	LightsBuffer*	pLights = m_pGraphics->m_Lights;

	// Initialize debug output;
	dxout.Initialize(pContext);

	m_pTextWrapper.reset(new FloatingTextWrapper(pDevice));
	m_pToolsIndicator[0] = m_pTextWrapper->CreateFloatingText(L"",145.0f,64.0f);
	m_pToolsIndicator[1] = m_pTextWrapper->CreateFloatingText(L"",screenWidth - 145.0f,64.0f);
	m_pMainIndicator = m_pTextWrapper->CreateFloatingText(ModeName[MODE_SCAN],(float)screenWidth / 2 ,(float)100,64.0f,ModeColorSchedle[MODE_SCAN]);
	m_pConstraintIndicator = m_pTextWrapper->CreateFloatingText(L"",200,(float)screenHeight -200 , 48 , Colors::Orange);
	m_pSnappedIndicator = m_pTextWrapper->CreateFloatingText(L"",(float)screenWidth / 2 ,(float)100,64,Colors::MediumPurple);

	m_pDepthBuffer.reset(new DirectX::DepthStencilBuffer(pDevice,screenWidth,screenHeight));

	std::cout<<"DirectX API Initialized.\nInitializing Shaders.."<<endl;
	m_pEffect.reset(new DirectX::CustomEffect<DirectX::SkinVertex>(pDevice,VertexShaderFile,nullptr,PixelShaderFile));
	m_pIndicatorEffect.reset(new DirectX::CustomEffect<DirectX::SkinVertex>(pDevice,IndicatorVertexShaderFile,nullptr,IndicatorPixelShaderFile));

	m_pSkyBox.reset(new DirectX::SkyBox(pDevice,pCamera,SkyBoxTextures));
	m_pFloor.reset(new DirectX::FloorPlane(pDevice,pCamera,FloorTexture));

	std::cout<<"Shaders Initialized.Initializing Kinect..."<<endl;
	// Create the input object.  This object will be used to handle reading the keyboard input from the user.
	// Initialize the input object.
	m_pInput.reset(new InputControler(m_hwnd));
	if (!m_pInput)
	{
		MessageBox(m_hwnd,L"Can't Initialize the Input Device.",L"Error in Initialize",MB_OK);
		throw std::bad_alloc();
	}

	std::cout<<"Kinect Initialized.Initializing Gestures Gloves..."<<endl;

	m_pPlayerUpdater.reset(new PlayerSkeletonUpdater);
	std::ifstream fin;
	fin.open(ConfigFileName);
	for (int i = 0; i < MAX_PLAYER_COUNT; i++)
	{
		m_pPlayers[i].reset(new Player(pContext,pCamera));
		// Set up the Player input
		m_pPlayerUpdater->Players.push_back(m_pPlayers[i].get());
		for (int hand = 0; hand < 2; hand++)
		{
			m_pPlayers[i]->Hands[hand].GestureHolders.clear();
			// Redundancy for each hand
			for (int k = 0; k < 2; k++)
			{
				if (m_pPlayers[i]->Hands[hand].GestureHolders.empty() || m_pPlayers[i]->Hands[hand].GestureHolders.back().DeviceIndex != -1)
					m_pPlayers[i]->Hands[hand].GestureHolders.emplace_back();
				auto& holder = m_pPlayers[i]->Hands[hand].GestureHolders.back();
				try
				{
					fin>>holder;
					holder.DeviceIndex = m_pInput->Mice().GetDeviceIndexByName(holder.DeviceName);
					if (holder.DeviceIndex != -1)
						m_pInput->Mice()[holder.DeviceIndex].Priority = m_pPlayers[i]->Hands[hand].GestureHolders.size();
				}
				catch(...)
				{
					holder.DeviceName = "Error";
					holder.DeviceIndex = -1;
					std::cout<<"Error in initializing player"<<i<<"data [hand "<<hand<<"][redundancy "<<k <<": Input configure file is corrupted , gesture input may not be working properly."<<endl;
				}
			}
		}
	}
	fin.close();

	std::cout<<"Gestures Gloves Initialized.Initializing Unities..."<<endl;

	//m_pPlayers[1]->Visible = false;
	//m_pPlayers[1]->Enabled = false;
	m_pPlayers[1]->Visiable = true;
	m_pPlayers[1]->Enabled = true;

	m_pMajorPlayer = m_pPlayers[0].get();
	//m_pMajorPlayer->Skeleton = m_pMajorPlayer->Skeleton;
#ifdef _DEBUG
		const float precise = .0f;
#else
	const float precise = 0.03f;
#endif
	m_pAvatar = make_shared<DynamicMetaBallModel>(pDevice,precise);
	//m_pAvatar->MetaballViewerDetail()->Color = Colors::Azure;
	//m_pAvatar->MetaballViewerDetail()->Color.w = 0.2f;
	//m_pAvatar->SkeletonViewerDetail()->Color = Colors::Red;

	//m_pPuppet.reset(new MetaBallModel());
	m_pPuppetViewer.reset(new MetaballViewer(pContext,pCamera));
	//m_pPuppetViewer->Target = m_pPuppet.get();

	//{
	//	DirectX::Vector4 SkinColor(Colors::White);
	//	SkinColor.w = 0.5f;
	//	dynamic_cast<DirectX::RenderTargetTexture2D*>(m_pAvatar->Skin->Texture())->Clear(pContext,SkinColor);
	//}

	auto pTexture = dynamic_cast<RenderTargetTexture2D*>(m_pAvatar->Skin().Texture());
	pTexture->Clear(m_pGraphics->Context() ,Colors::White);
	//pTexture->Clear(m_pGraphics->Context() ,Colors::Black);

	//	m_pWarpper = new NaiveWarpper(m_pMajorPlayer->Skeleton,m_pAvatar);
	//m_pWarpper = new MetaballBasedWarpper(m_pMajorPlayer->Skeleton,m_pAvatar.get());

	//m_pAttacher.reset(new AttachMap());
	m_pMotionAdaptor = std::make_shared<Kinematics::HS_MB_Adaptor>(*m_pMajorPlayer->Skeleton,*m_pAvatar);

	m_UniformRadius = Editing_Tools_Default_Radius;
	m_pColorPalette = PaintTools::ColorHoverPicker::InitializeColorPalette(pDevice,pCamera);
	// Initialize Single hand tools
	for (int player = 0 ; player < MAX_PLAYER_COUNT ; player ++)
	{
		auto pColorHoverPicker = std::make_shared<PaintTools::ColorHoverPicker>();
		for (int hand = 0; hand < 2; hand++)
		{
			auto Index = (player<<1) + hand;
			m_Slots[player][hand].Previewer = make_shared<DynamicMetaBallModel>(pDevice,precise);
			//m_Slots[player][hand].Previewer->MetaballViewerDetail()->Color = Colors::Green;
			//m_Slots[player][hand].Previewer->MetaballViewerDetail()->Color.w = 0.2f;
			//m_Slots[player][hand].Previewer->SkeletonViewerDetail()->Color = Colors::Red;
			auto pPreviewTexture = dynamic_cast<RenderTargetTexture2D*>(m_Slots[player][hand].Previewer->Skin().Texture());
			pPreviewTexture->Clear(m_pGraphics->Context() ,Colors::White);
			//pPreviewTexture->Clear(m_pGraphics->Context() ,Colors::Black);

			auto pLight = &(pLights->SphereLights[Index]);
			// Geometry Tools
			m_Slots[player][hand].ToolsInSlot[Cursor]	= new FreeCursor	(pContext,m_pAvatar,pCamera,pLight);
			m_Slots[player][hand].ToolsInSlot[Drag]		= new DragManager	(m_pAvatar,m_Slots[player][hand].Previewer);
			m_Slots[player][hand].ToolsInSlot[Grow]		= new GrowManager	(m_pAvatar,m_Slots[player][hand].Previewer);
			m_Slots[player][hand].ToolsInSlot[Sweep]	= new SweepManager	(m_pAvatar,m_Slots[player][hand].Previewer);
			m_Slots[player][hand].ToolsInSlot[Drive]	= new DriveManager	(m_pAvatar,m_pMotionAdaptor,hand);
			m_Slots[player][hand].ToolsInSlot[Cut]		= new CutManager	(pContext,m_pAvatar,pCamera);

			// Paint tools
			auto pHoverPicker = new PaintTools::HoverCursorTool(m_pAvatar , nullptr , pContext , pCamera , pLight , pColorHoverPicker);
			auto pColorSource = pHoverPicker->GetColorSource();
			m_Slots[player][hand].ToolsInSlot[HoverPickCursor] = pHoverPicker;
			m_Slots[player][hand].ToolsInSlot[Eyedropper] = new PaintTools::EyedropperTool(m_pAvatar , pColorSource , pContext , pCamera , m_pColorPalette);

			// Brushes
			auto pBrushTool = new PaintTools::BrushTool(m_pAvatar , pColorSource , pContext , pCamera , pLight );
			pBrushTool->SetBrush(pContext,unique_ptr<PaintTools::IBrush>(new PaintTools::GaussBrush(pDevice,&m_pAvatar->Skin(),Index)));
			m_Slots[player][hand].ToolsInSlot[GaussBrush] = pBrushTool;

			pBrushTool = new PaintTools::BrushTool(m_pAvatar, pColorSource , pContext , pCamera , pLight );
			pBrushTool->SetBrush(pContext,unique_ptr<PaintTools::IBrush>(new PaintTools::PaintBucket));
			m_Slots[player][hand].ToolsInSlot[FillBrush] = pBrushTool;

			m_Slots[player][hand].ToolID = Cursor;
		}
		// Double Hand tools
		m_Slots[player][0].ToolsInSlot[Vculpt] = m_Slots[player][1].ToolsInSlot[Vculpt] = //nullptr;
			new VolumeSculptManager(pContext,m_pAvatar,m_Slots[player][0].Previewer,pCamera);
		m_Slots[player][0].ToolsInSlot[Manipulate] = m_Slots[player][1].ToolsInSlot[Manipulate] = nullptr;
		//new ManipulationManager(m_pAvatar,m_pPlayers[player]->Skeleton,m_pSnappedIndicator);
	}

	SetRadius(m_UniformRadius);
	TurnRadius(0.05f);

#ifdef _DEBUG
	RenderMode.Specify(DynamicMetaBallModel::SKELETON_ANIMATED);
	RenderMode.Set(DynamicMetaBallModel::METABALLS_ANIMATED);
#else
	RenderMode.Specify(DynamicMetaBallModel::SKIN_ANIMATED);
#endif // _DEBUG
	//RefreshRenderMode();

	m_Mode = MODE_SCAN;

	thread StreamStartingThread([this](){
		m_pInput->StartStream();
	});

	m_StreamingThread.swap(StreamStartingThread);

	return true;
}

// Safe delete pointers
template<class T>
inline void delete_s(T*& ptr)
{
	if(ptr)
	{
		delete ptr;
		ptr = nullptr;
	}
}

void SystemControler::Shutdown()
{
	m_StreamingThread.join();
	FreeConsole();

	//PaintTools::ColorHoverPicker::ReleaseColorPalette();
	dxout.Release();

	//delete_s(m_pWarpper);

	// Free the tools
	for (int player = 0; player < 2; player++)
	{
		for (int hand = 0; hand < 2; hand++)
		{
			//delete_s(m_Slots[player][hand].Previewer);
			for (int tool = 0; tool < Count; tool++)
			{
				// Double tools should only be delete once
				if (tool != Vculpt && hand != RightHand)
					delete_s(m_Slots[player][hand].ToolsInSlot[tool]);
				else
					m_Slots[player][hand].ToolsInSlot[tool] = nullptr;
			}
		} 
	}
	delete_s(m_pGraphics);
	//delete_s(m_pInput);
	m_pInput.reset();
	// Shutdown the window.
	ShutdownWindows();

	return;
}

//void Remapping(DynamicMetaBallModel *pAvatar,AttachMap* pMapper)
//{
//	auto &Vertices = pAvatar->Skin.Vertices_W();
//	//auto &Skeleton = *pAvatar->Skeleton;
//	auto &Mapper = *pMapper;
//
//	pAvatar->WeightingSkin(Vertices);
//	std::vector<unsigned int> Dominater(Skeleton.Index.crbegin()->first + 1);
//	for (auto& itr : Skeleton.Index)
//	{
//		unsigned int key = itr.first;
//		auto DriverID = Mapper.FindDominateJointID(key);
//		Dominater[key] = DriverID;
//	}
//	//Concurrency::array_view<DirectX::SkinVertex,1> vertices(Vertices.size(),Vertices);
//	//concurrency::parallel_for_each(vertices.extent,
//
//	Concurrency::parallel_for_each(Vertices.begin(),Vertices.end(),[&](DirectX::SkinVertex& vertex)
//	{
//		UINT32* indices = (UINT32*)&vertex.Indices;
//		for (int i = 0; i < 4; i++)
//		{
//			indices[i] = Dominater[indices[i]];
//		}
//	});
//
//}

//void SystemControler::AutoScaling()
//{
//	// Find Scale Factor
//	auto& player = *m_pMajorPlayer->Skeleton;
//	auto minmax = std::minmax_element( m_pMajorPlayer->Skeleton->Index.cbegin() ,  m_pMajorPlayer->Skeleton->Index.cend(), 
//		[](const HumanSkeleton::Index_Item& lhs , const HumanSkeleton::Index_Item& rhs ) -> bool 
//	{
//		return lhs.second->Entity[Current].Position.y < rhs.second->Entity[Current].Position.y;
//	});
//	float Height = minmax.second->second->Entity[Current].Position.y - minmax.first->second->Entity[Current].Position.y;
//	float AHeight;
//
//	minmax = std::minmax_element( m_pAvatar->Skeleton->Index.cbegin() ,  m_pAvatar->Skeleton->Index.cend(), 
//		[](const IndexedSkeleton::Index_Item& lhs , const IndexedSkeleton::Index_Item& rhs ) -> bool 
//	{
//		return lhs.second->Entity[Current].Position.y < rhs.second->Entity[Current].Position.y;
//	});
//	AHeight = minmax.second->second->Entity[Current].Position.y - minmax.first->second->Entity[Current].Position.y;
//
//	DirectX::Vector3 Origin =  minmax.first->second->Entity[Current].Position;
//	Origin.z = player.Root->Entity[Current].Position.z;
//	float Scale = Height / AHeight;
//
//	if (std::abs(Scale - 1.0f) > 0.01f)
//	{
//		std::cout << "Human height = " << Height << " ; Avatar Height = " << AHeight << " Scale = " << Scale << std::endl;
//		m_pAvatar->Scale(Scale , Origin);
//	}
//}

void SystemControler::Attaching()
{
	if (m_ScalingFlag && m_ScalingTimer.GetTime() < 3000)
	{
		//AutoScaling();
	} else
	{
		m_ScalingFlag = false;
	}

	m_pAvatar->Adaptor() = m_pMotionAdaptor;
	m_pMotionAdaptor->Attach();
	m_pAvatar->Update();
	//m_pAttacher->Initialize(m_pMajorPlayer->Skeleton,&m_pMotionAdaptor->Intermediate);
	//m_pAttacher->AutoAttach(m_pMajorPlayer->Skeleton->HIP_CENTER);
	m_pMajorPlayer->Skeleton->Snap_Default_to_Current();
	//Remapping(m_pAvatar,m_pAttacher.get());
}

inline XMFLOAT2 SystemControler::TransformWorldCoordIntoScreenCoord(FXMVECTOR vPositionWS) const
{
	XMMATRIX ViewMatrix = m_pGraphics->Camera->GetViewMatrix();
	XMMATRIX ProjectionMatrix = m_pGraphics->Camera->GetProjectionMatrix();
	XMMATRIX ViewProjectionMatrix = ViewMatrix * ProjectionMatrix;
	XMVECTOR vPositionPS = XMVector3TransformCoord(vPositionWS,ViewProjectionMatrix);
	vPositionPS *= g_XMNegateY;
	vPositionPS *= g_XMOneHalf;
	vPositionPS += g_XMOneHalf;
	XMVECTOR vScreen = XMVectorSet((float)screenWidth,(float)screenHeight,0.0f,0.0f);
	vPositionPS = vPositionPS * vScreen;
	XMFLOAT2A vPs;
	XMStoreFloat2A(&vPs,vPositionPS);
	return vPs;
}

bool SystemControler::SwitchConstraintMode(ConstraintMode constraint)
{

	if (m_Constraint == constraint) return false;
	if (constraint >= Constraint_Count) 
	{
		cout<<"Invalid constraint"<<endl;
		return false;	
	}

	if (constraint != Constraint_None && constraint != Constraint_Scan_Only && m_Mode == MODE_SCAN)
		return false;

	switch (constraint)
	{
	case SystemControler::Constraint_None:
		break;
	case SystemControler::Constraint_Drag_Only:
	case SystemControler::Constraint_Sculpt_Only:
	case SystemControler::Constraint_Cut_Only:
	case SystemControler::Constraint_Grow_Only:
	case SystemControler::Constraint_Double_Sculpt_Only:
		SwitchSystemMode(MODE_REFERENCE_EDITING);
		break;
	case SystemControler::Constraint_Paint_Only:
	case SystemControler::Constraint_Fill_Only:
		SwitchSystemMode(MODE_REFERENCE_PAINT);
		break;
	case SystemControler::Constraint_Scan_Only:
		SwitchSystemMode(MODE_SCAN);
		break;
	case SystemControler::Constraint_Manipulate_Only:
		SwitchSystemMode(MODE_SPATIAL_EDITING);
		break;
	default:
		break;
	}

	m_Constraint = constraint;
	m_pConstraintIndicator->Content = ConstraintName[constraint];
	return true;
}

SystemControler::SystemMode SystemControler::SwitchViewMode(ViewMode view)
{
	if (view == View_Unknown)
		return m_Mode;
	auto CurrentView = CurrentViewMode();
	if (view == CurrentView)
	{
		return m_Mode;
	}

	SystemMode newMode;
	switch (m_Mode)
	{
	case SystemControler::MODE_ATTACHING:
	case SystemControler::MODE_SPATIAL_EDITING:
		newMode = (MODE_REFERENCE_EDITING);
		break;
	case SystemControler::MODE_SPATIAL_PAINT:
		newMode = (MODE_REFERENCE_PAINT);
		break;
	case SystemControler::MODE_ANIMATE:
	case SystemControler::MODE_REFERENCE_EDITING:
		newMode = (MODE_SPATIAL_EDITING);
		break;
	case SystemControler::MODE_REFERENCE_PAINT:
		newMode = (MODE_SPATIAL_PAINT);
		break;
	default:
		return m_Mode;
	}
	SwitchSystemMode(newMode);
	return m_Mode;
}
SystemControler::ViewMode SystemControler::CurrentViewMode() const
{
	switch (m_Mode)
	{
	case SystemControler::MODE_ATTACHING:
	case SystemControler::MODE_SPATIAL_EDITING:
	case SystemControler::MODE_SPATIAL_PAINT:
		return View_Spatial;
	case SystemControler::MODE_REFERENCE_EDITING:
	case SystemControler::MODE_REFERENCE_PAINT:
	case SystemControler::MODE_ANIMATE:
		return View_Reference;
	case SystemControler::MODE_PAUSE:
	case SystemControler::MODE_SCAN:
	case SystemControler::MODE_COUT:
	default:
		return View_Unknown;
	}
}

SystemControler::SystemMode SystemControler::SwitchEditMode(EditMode editMode)
{
	auto currentEdit = CurrentEditMode();
	if (editMode == currentEdit)
		return m_Mode;
	auto currentView = CurrentViewMode();

	SystemMode target = m_Mode;;
	switch (editMode)
	{
	case SystemControler::Edit_Scan:
		target = (MODE_SCAN);
		break;
	case SystemControler::Edit_Sculpt:
		switch (currentView)
		{
		case SystemControler::View_Unknown:
		case SystemControler::View_Reference:
			target = (MODE_REFERENCE_EDITING);
			break;
		case SystemControler::View_Spatial:
			target = (MODE_SPATIAL_EDITING);
			break;
		}
		break;
	case SystemControler::Edit_Paint:
		switch (currentView)
		{
		case SystemControler::View_Unknown:
		case SystemControler::View_Reference:
			target = (MODE_REFERENCE_PAINT);
			break;
		case SystemControler::View_Spatial:
			target = (MODE_SPATIAL_PAINT);
			break;
		}
		break;
	case SystemControler::Edit_None:
	default:
		return m_Mode;
	}
	SwitchSystemMode(target);
	return m_Mode;
}

SystemControler::EditMode SystemControler::CurrentEditMode() const
{
	switch (m_Mode)
	{
	case SystemControler::MODE_REFERENCE_PAINT:
	case SystemControler::MODE_SPATIAL_PAINT:
		return Edit_Paint;
	case SystemControler::MODE_REFERENCE_EDITING:
	case SystemControler::MODE_SPATIAL_EDITING:
		return Edit_Sculpt;
	case SystemControler::MODE_SCAN:
		return Edit_Scan;
	case SystemControler::MODE_ATTACHING:
	case SystemControler::MODE_PAUSE:
	case SystemControler::MODE_COUT:
	case SystemControler::MODE_ANIMATE:
	default:
		return Edit_None;
	}
}


bool SystemControler::SwitchSystemMode( SystemMode mode )
{
	if (mode >= MODE_COUT || mode < 0)
		return false;
	cout << "Editing Mode Switch : " << m_Mode <<" => " <<mode<<endl;

	auto oldMode = m_Mode;
	auto newMode = mode;
	if (oldMode == newMode) return false;
	// Finish Mode
	switch (m_Mode)
	{
	case MODE_PAUSE:
		break;
	case MODE_SCAN:
		{
			m_ScalingFlag = true;
			m_ScalingTimer.Reset();
			//m_AttachDisableTimer.Reset();
			m_pTextWrapper->CreateTimedFloatingText(L"Scanned!",(float)screenWidth / 2 , (float)100,2500,64.0f , ModeColorSchedle[MODE_SCAN]);
			m_pMainIndicator->Position.y = 200.f;
			m_pMainIndicator->Content = L"Please Attach";
			//dynamic_cast<ManipulationManager*>(m_Slots[0][0][Manipulate])->SetGroundHeight(0.0f);
			//dynamic_cast<ManipulationManager*>(m_Slots[1][1][Manipulate])->SetGroundHeight(0.0f);
			m_Constraint = Constraint_Manipulate_Only;
			m_pConstraintIndicator->Content = ConstraintName[Constraint_Manipulate_Only];
		}
		break;
	case MODE_ATTACHING:
		{
			//m_pAvatar->WeightingSkin(m_pAvatar->Skin.Vertices_W());
			if (newMode >= MODE_SPATIAL_EDITING)
				m_pTextWrapper->CreateTimedFloatingText(L"Body Attached!",(float)screenWidth / 2 , (float)100,2500,64.0f , Colors::Crimson);
		}
		break;
	case MODE_SPATIAL_EDITING:
		{
			//m_pPuppet->clear();
			Attaching();
			//m_pAvatar->WeightingSkin(m_pAvatar->Skin.Vertices_W());
			if (m_Constraint == Constraint_Manipulate_Only)
				SwitchConstraintMode(Constraint_None);
			if (newMode >= MODE_SPATIAL_EDITING)
				m_pTextWrapper->CreateTimedFloatingText(L"Body Attached!",(float)screenWidth / 2 , (float)100,2500,64.0f , Colors::Crimson);
		}
		//NO break; // !!!INTEND
	case MODE_REFERENCE_PAINT:
	case MODE_REFERENCE_EDITING:
		{
			for (int player = 0; player < MAX_PLAYER_COUNT; player++)
			{
				for (int hand = 0; hand < 2; hand++)
				{
					m_Slots[player][hand].Tool()->Finish();
				}
			}
			for (int hand = 0; hand < 2; hand++)
			{
				m_pToolsIndicator[hand]->Hide();
			}

		}
		break;
	case MODE_ANIMATE:
		break;
	default:
		break;
	}

	m_Mode = mode;

	// Start Mode
	switch (m_Mode)
	{
	case MODE_PAUSE:
		break;
	case MODE_SCAN:
		{
			if (m_pAvatar->Volume().size() > 0)
				SnapShoot();
			for (auto& player : m_pPlayers)
			{
				player->ArmScaling = false;
			}
			auto pTexture = dynamic_cast<RenderTargetTexture2D*>(m_pAvatar->Skin().Texture());
			pTexture->Clear(m_pGraphics->Context() ,Colors::White);
			m_pMainIndicator->Position.y = 100.f;
		}
		//m_pAvatar->Skin->Texture_W().Dye(Colors::White);
		break;
	case MODE_ATTACHING:
		{
			for (auto& player : m_pPlayers)
			{
				player->ArmScaling = false;
			}
			m_StayinTimer.Reset();
			//m_pAttacher->DetachAll();
			if (oldMode>MODE_SPATIAL_EDITING)
				m_pTextWrapper->CreateTimedFloatingText(L"Body Detached!",(float)screenWidth / 2 , (float)100,2500,64.0f , Colors::Aquamarine);
		}
		break;
	case MODE_SPATIAL_EDITING:
	case MODE_REFERENCE_EDITING:
	case MODE_REFERENCE_PAINT:
		{
			if (m_Mode == MODE_SPATIAL_EDITING)
			{
				m_StayinTimer.Reset();
				m_pAvatar->Disconnect();
				if (oldMode>MODE_SPATIAL_EDITING)
					m_pTextWrapper->CreateTimedFloatingText(L"Body Detached!",(float)screenWidth / 2 , (float)100,2500,64.0f , Colors::Aquamarine);
			}

			ToolsSet CursorTool = Cursor;
			if (m_Mode == MODE_REFERENCE_PAINT)
				CursorTool = HoverPickCursor;
			for (int player = 0; player < MAX_PLAYER_COUNT; player++)
			{
				if (m_Mode != MODE_SPATIAL_EDITING)
				{
					m_pPlayers[player]->ArmScaling = true;
					m_pPlayers[player]->ArmScalingRadius = m_pAvatar->Volume().BoundingSphere.Radius * 1.2f;
				}else
				{
					m_pPlayers[player]->ArmScaling = false;
				}
				for (int hand = 0; hand < 2; hand++)
				{
					if (m_Slots[0][hand][Drive]->IsActive())
					{
						m_Slots[player][hand].ToolID = Drive;
						//m_Slots[player][hand].Tool()->Start();
						m_pToolsIndicator[hand]->Content = wstring(HandSwitchName[hand]) + L"\nAttached";
						m_pToolsIndicator[hand]->SetColor(Colors::Crimson);
					} else
					{
						m_Slots[player][hand].ToolID = CursorTool;
						m_Slots[player][hand].Tool()->Start();
						m_pToolsIndicator[hand]->Content = L"";
						m_pToolsIndicator[hand]->SetColor(Colors::LightGreen);
					}
					m_pToolsIndicator[hand]->Show();
					m_HandTimers[hand].Unpause();
					m_HandTimers[hand].Reset();
				}
			}
		}
		break;
	case MODE_ANIMATE:
		break;
	default:
		break;
	}

	if (m_Constraint != Constraint_None)
	{
		switch (m_Mode)
		{
		case SystemControler::MODE_PAUSE:
			break;
		case SystemControler::MODE_SCAN:
			m_Constraint = Constraint_Scan_Only;
			m_pConstraintIndicator->Content = ConstraintName[Constraint_Scan_Only];
			break;
		case SystemControler::MODE_ANIMATE:
		case SystemControler::MODE_ATTACHING:
		case SystemControler::MODE_SPATIAL_EDITING:
			if (m_Constraint != Constraint_Manipulate_Only)
			{
				m_Constraint = Constraint_Manipulate_Only;
				m_pConstraintIndicator->Content = ConstraintName[Constraint_Manipulate_Only];
			}
			break;
		case SystemControler::MODE_REFERENCE_EDITING:
			if (m_Constraint == Constraint_Paint_Only || m_Constraint == Constraint_Fill_Only)
			{
				m_Constraint = Constraint_Drag_Only;
				m_pConstraintIndicator->Content = ConstraintName[Constraint_Drag_Only];
			}
			break;
		case SystemControler::MODE_REFERENCE_PAINT:
			if (m_Constraint != Constraint_Paint_Only && m_Constraint != Constraint_Fill_Only)
			{
				m_Constraint = Constraint_Paint_Only;
				m_pConstraintIndicator->Content = ConstraintName[Constraint_Paint_Only];
			}
			break;
		case SystemControler::MODE_COUT:
			break;
		default:
			break;
		}
	}

	//m_pTextWrapper->ShowFloatingText(oss.str(),1000,(float)100,(float)screenWidth / 2,32,Colors::White);
	if (m_Mode <= MODE_SCAN)
		m_pMainIndicator->Content = ModeName[m_Mode];
	m_pMainIndicator->SetColor(ModeColorSchedle[m_Mode]);
	//m_pTextWrapper->CreateTimedFloatingText(L"Mode changed!",(float)screenWidth / 2 , (float)200,3000,64.0f , Colors::Aquamarine);
	return true;
}

void SystemControler::Scaning(){
	if (!m_pMajorPlayer->IsActive()) return;

	m_pMajorPlayer->Skeleton->Snap_Default_to_Current();
	m_pMajorPlayer->Skeleton->FleshRadius = m_UniformRadius;
	m_pAvatar->Clear();
	for (auto& idx : m_pMajorPlayer->Skeleton->Index)
	{
		if (idx.second->IsRoot)
		{
			m_pAvatar->AddMetaball(Metaball(idx.second->Entity[Default].Position,m_UniformRadius));
		} else
		{
			XMVECTOR p0 = idx.second->Parent->Entity[Default].Position;
			XMVECTOR p1 = idx.second->Entity[Default].Position;
			m_pAvatar->AddCylinder(p0,p1,m_UniformRadius,m_UniformRadius,true);
		}
	}
	//->InterpolateSkeleton(nullptr , m_UniformRadius);

	//if (m_pPlayers[1]->IsActive())
	//{
	//	auto& pSecondPlayer = m_pPlayers[1];
	//	unique_ptr<DynamicMetaBallModel> pExtraPart(new DynamicMetaBallModel);
	//	pSecondPlayer->Skeleton->Snap_Default_to_Current();
	//	*pExtraPart->Skeleton = *pSecondPlayer->Skeleton; // It's major play , for DEBUG
	//	//*pExtraPart->Skeleton = *m_pMajorPlayer->Skeleton; // It's major play , for DEBUG
	//	//pExtraPart->Skeleton->Root->Entity[Current].Position.x += 0.2f;
	//	//pExtraPart->Skeleton->Update();
	//	//pExtraPart->Skeleton->Snap_Default_to_Current();
	//	pExtraPart->InterpolateSkeleton(nullptr,m_UniformRadius);

	//	float minDis = D3D11_FLOAT32_MAX;
	//	pair<size_t,size_t> ClosestPair;
	//	for (size_t i = 0; i < m_pAvatar->Flesh->size(); i++)
	//	{
	//		for (size_t j = 0; j < pExtraPart->Flesh->size(); j++)
	//		{
	//			float dis = Vector3::Distance(m_pAvatar->Flesh->at(i).Position,pExtraPart->Flesh->at(j).Position);
	//			if (dis < minDis)
	//			{
	//				minDis = dis;
	//				ClosestPair.first = i;
	//				ClosestPair.second= j;
	//			}
	//		}
	//	}

	//	bool connected = Metaball::Connected(m_pAvatar->Flesh->at(ClosestPair.first),pExtraPart->Flesh->at(ClosestPair.second),m_pAvatar->Flesh->GetISO());
	//	cout<<"Player 2 's distance :"<<setw(6)<<setprecision(3)<<minDis<<" Connected : "<<connected<<endl;
	//	if (connected)
	//	{
	//		cout<<"Union Scanning!!!!!! Wait for a crush!!!"<<endl;
	//		auto Idx = pExtraPart->Flesh->at(ClosestPair.second).BindingIndex;
	//		pExtraPart->Skeleton->SelectNewRoot(pExtraPart->Skeleton->at(Idx));
	//		auto pAppendTargetJoint = m_pAvatar->Skeleton->at(m_pAvatar->Flesh->at(ClosestPair.first).BindingIndex);
	//		m_pAvatar->AppendModel(pExtraPart.get(),pAppendTargetJoint,false);
	//	}
	//}

	//m_pAvatar->Flesh->Update();
	m_pAvatar->Update();
}

//void DrivePuppet(MetaBallModel& Puppet , IndexedSkeleton& Skeleton)
//{
//	if (Puppet.Primitives.empty() || Puppet[0].BlendWeights.empty()) return;
//	vector<DirectX::Vector3> NaivePoints(Puppet.Primitives.size());
//	for (size_t i = 0; i < Puppet.Primitives.size(); i++)
//	{
//		NaivePoints[i] = Puppet[i].Position;
//		Puppet[i].Position.Set(0.0f,0.0f,0.0f);
//	}
//
//	for (auto& idx : Skeleton.Index)
//	{
//		XMMATRIX Transform = idx.second->BlendMatrix();
//		for (size_t i = 0; i < Puppet.size(); i++)
//		{
//			if (Puppet[i].BlendWeights.size() <= idx.first) continue;
//			if (Puppet[i].BlendWeights[idx.first] > .0f) {
//				XMVECTOR vPos = XMVector3TransformCoord(NaivePoints[i],Transform);
//				vPos *= Puppet[i].BlendWeights[idx.first];
//				Puppet[i].Position += vPos;
//			}
//		}
//	}
//}


void SystemControler::ExcuteOperations(){
	switch (m_Mode)
	{
	case MODE_PAUSE:
		break;
	case MODE_SCAN:
		{
			Scaning();
			break;
		}
	case MODE_ATTACHING:
		{
			Attaching();
			m_pAvatar->AnimationUpdate();
			break;
		}
	case MODE_SPATIAL_EDITING:
	case MODE_REFERENCE_EDITING:
	case MODE_REFERENCE_PAINT:
		if (m_Mode != MODE_SPATIAL_EDITING)
		{
			//m_pAttacher->Drive();
			//if (m_pPuppet->size() != m_pAvatar->Flesh->size())
			//	*m_pPuppet = *m_pAvatar->Flesh;
			//else
			//{
			//	m_pPuppet->Primitives = m_pAvatar->Flesh->Primitives;
			//}
			//DrivePuppet(*m_pPuppet,*m_pMajorPlayer->Skeleton);
			m_pAvatar->AnimationUpdate();
		} else // m_Mode == MODE_SPATIAL_EDITING
		{
			Attaching();

		}

		for (int player = 0; player < MAX_PLAYER_COUNT; player++)
		{
			if (!m_pPlayers[player]->IsActive()) continue;
			auto pSk = m_pPlayers[player]->Skeleton;
			for (int hand = 0; hand < 2; hand++)
			{
				if ((m_Mode == MODE_REFERENCE_EDITING || m_Mode == MODE_REFERENCE_PAINT) && m_Slots[player][hand][Drive]->IsActive()) continue;
				m_Slots[player][hand].Tool()->Edit(
					pSk->GetActualJoint(HumanSkeleton::HAND_LEFT + hand*4),
					pSk->GetActualJoint(HumanSkeleton::WRIST_LEFT + hand*4),
					pSk->GetActualJoint(HumanSkeleton::ELBOW_LEFT + hand*4),
					pSk->GetActualJoint(HumanSkeleton::SHOULDER_LEFT + hand*4)
					);
#if defined(REMESH_EVERY_FRAME)
				m_pAvatar->Remesh();
#endif
			}
		}
		//m_pAvatar->AnimationUpdate();
		break;
	case MODE_ANIMATE:
		//m_pAttacher->Drive();
		m_pAvatar->AnimationUpdate();
		break;
	default:
		break;
	}
}

//The Function runs before each frame to render
//The main logic control part
bool SystemControler::Frame()
{
	bool result = true;
	static int FrameCount = 0;
	FrameCount++;

	// Interpret the keyboard event
	result = InterpretMisc();
	if(!result)	return false;
	// Interpret if the user proceed a speech command
	InterpretSpeech();

	auto pSkeletonFrame = m_pInput->GetSkeletonFrame();
	if(!m_Paused && UpdateSkeleton(pSkeletonFrame)) {
		// Update the skeleton input data
		//(pSkeletonFrame);
		// Interpret the posture event
		InterpretBodyPosture();
		// Interpret the Gesture event
		InterpretGesture();
		// Provides the tools new data to editing
		ExcuteOperations();

		// Use this to prevent long time scan in debug mode
		//#ifdef _DEBUG
		//		if (m_Mode == MODE_SCAN)
		//			SwitchSystemMode(MODE_SPATIAL_EDITING);
		//#endif
	}

	// Rendering Stage
	RenderScene();
	//Send date to graphics part to draw
	return true;
}

void SystemControler::RenderScene()
{
	auto pContext = m_pGraphics->Context();
	m_pGraphics->BeginScene();
	m_pDepthBuffer->Clear(pContext);
	dxout.SetProjection(m_pGraphics->Camera->GetProjectionMatrix());
	dxout.SetView(m_pGraphics->Camera->GetViewMatrix());
	dxout.SetWorld(XMMatrixIdentity());

	m_pGraphics->SetRenderTarget(nullptr,nullptr);
	//m_pGraphics->RenderModel(m_pSkyBox.get());
	m_pGraphics->RenderModel(m_pFloor.get());

	if (m_Mode == MODE_REFERENCE_PAINT)
	{
		m_pGraphics->Mode_DepthStencil = GraphicsClass::DepthStencilMode::None;
		m_pGraphics->RenderModel(m_pColorPalette.get());
		m_pGraphics->Mode_DepthStencil = GraphicsClass::DepthStencilMode::Default;
	}

	if (RenderMode >= DynamicMetaBallModel::RenderMode::SKIN_ANIMATED)
	{
		// For Customized Transparency effect.
		// 1) Render the back face of avatar model into another depth buffer
		m_pGraphics->SetRenderTarget(nullptr,m_pDepthBuffer.get());
		m_pGraphics->Mode_Rasterizer = GraphicsClass::RasterizerMode::CullClockwise;
		m_pGraphics->RenderModel(&m_pAvatar->Skin() , m_pEffect.get());
		// 2) Render the front face of the avatar model into default depth buffer
		m_pGraphics->SetRenderTarget(nullptr,nullptr);
		m_pGraphics->Mode_Rasterizer = GraphicsClass::RasterizerMode::CullCounterClockwise;
		m_pGraphics->RenderModel(&m_pAvatar->Skin() , m_pEffect.get());
		// 3) Render the semi-transparency skeleton to screen with the depth buffer storage the back-face of avatar
		m_pGraphics->SetRenderTarget(nullptr,m_pDepthBuffer.get());
		for (auto& player : m_pPlayers)
		{
			auto viewer = player->getVisualizer();
			if (!viewer) continue;

			auto skviewer = dynamic_cast<SkeletonViewer*>(viewer);
			skviewer->Color.w = SkeletonTransparency;

			SkeletonViewer::CustomStateFunc = [&](){
				m_pGraphics->SetDepthStencilState(GraphicsClass::DepthStencilMode::Default);
				//m_pGraphics->SetBlendState(GraphicsClass::BlendMode::AlphaBlend);
			};

			m_pGraphics->RenderModel(viewer);

			SkeletonViewer::CustomStateFunc = nullptr;
		}
	}
	// 4) Render the full skeleton with the default depth buffer which contains the front face of avatar
	m_pGraphics->SetRenderTarget(nullptr,nullptr);

	for (auto& player : m_pPlayers)
	{
		auto viewer = player->getVisualizer();
		if (!viewer) continue;

		auto skviewer = dynamic_cast<SkeletonViewer*>(viewer);
		skviewer->Color.w = 1.0f;
		m_pGraphics->RenderModel(viewer);
		//SkeletonViewer::CustomStateFunc = nullptr;
	}

	if (RenderMode >= DynamicMetaBallModel::RenderMode::METABALLS_ANIMATED)
	{
		m_pPuppetViewer->Target = m_pAvatar.get();
		m_pGraphics->RenderModel(m_pPuppetViewer.get());
	}

	// Finish rendering avatar and skeleton

	if (m_Mode == MODE_REFERENCE_EDITING || m_Mode == MODE_REFERENCE_PAINT || m_Mode == MODE_SPATIAL_EDITING)
	{
		for (int player = 0; player < MAX_PLAYER_COUNT; player++)
		{
			if (!m_pPlayers[player]->IsActive()) continue;
			for (int hand = 0; hand < 2; hand++)
			{
				auto pTool = m_Slots[player][hand].Tool();
				m_pGraphics->RenderModel(pTool->Visualization() , m_pEffect.get());
				if (RenderMode >= DynamicMetaBallModel::RenderMode::METABALLS_ANIMATED)
				{
					m_pPuppetViewer->Target = m_Slots[player][hand].Previewer.get();
					m_pGraphics->RenderModel(m_pPuppetViewer.get());
				}
			}
		}
	}

	//if (m_Mode == MODE_REFERENCE_EDITING)
	//{
	//	m_pGraphics->RenderModel(m_pPuppetViewer.get());
	//}


	m_pTextWrapper->Render(pContext);

	m_pGraphics->EndScene();
}

bool SystemControler::UpdateSkeleton(NUI_SKELETON_FRAME* pFrame){
	return m_pPlayerUpdater->UpdatePlayerSkeleton(pFrame);
}

//The interactive logic for speech input
bool SystemControler::InterpretSpeech()
{
	//	unsigned long long UpdateFlag;
	switch (m_pInput->GetSpeechCommand())
	{
	case SpeechCommands::Smaller:
		if (!TurnRadius(-0.04f))
			std::cout<<"You can't getting Smaller."<<std::endl;
		break;

	case SpeechCommands::Bigger:
		if (!TurnRadius(0.04f))
			std::cout<<"You can't getting Bigger."<<std::endl;
		break;

	case SpeechCommands::Scan:
		if( m_Mode == MODE_SCAN) { 
			SwitchSystemMode(MODE_SPATIAL_EDITING);
		} else {
			SwitchSystemMode(MODE_SCAN);
		}
		break;

	case SpeechCommands::Edit:
		if (m_Mode == MODE_REFERENCE_PAINT ) {
			SwitchSystemMode(MODE_REFERENCE_EDITING);
		}
		break;

	case SpeechCommands::Paint:
		if (m_Mode == MODE_REFERENCE_EDITING || m_Mode == MODE_SPATIAL_EDITING) {
			SwitchSystemMode(MODE_REFERENCE_PAINT);
		}
		break;

	case SpeechCommands::Attach:
		if (m_Mode == MODE_SPATIAL_EDITING){
			SwitchSystemMode(MODE_REFERENCE_EDITING);
		}
		break;

	case SpeechCommands::Detach:
		{
			if (m_Mode >= MODE_SPATIAL_EDITING)
			{
				SwitchSystemMode(MODE_SPATIAL_EDITING);
			}
		}
		break;

	case SpeechCommands::Cancel:
		{
			CancelLastOperation();
		}
		break;

	case SpeechCommands::Reset:
		{
			//if (m_Mode == MODE_SPATIAL_EDITING || m_Mode == MODE_SPATIAL_PAINT)
			//{
			//	m_pAvatar->AnimationUpdate();
			//	auto Box = m_pAvatar->Volume().GetBoundingBox();
			//	m_pAvatar->RotateTo(Quaternion::Identity(),XMLoadFloat3(&Box.Center));
			//	//if (Box.Center.y - Box.Extents.y < 0.2f)
			//	m_pAvatar->Ground(0.0f);
			//}
			m_Paused = !m_Paused;
		}
		break;

	case SpeechCommands::Save:
		SavingAvatarToFile();
		break;

		//case SpeechCommands::Load:
		//	LoadingAvatarFromFile();
		//	break;
		//	//case SpeechCommands::Done:
		//	//	m_Geometry->Finish();
		//	//	m_SystemStatue->mIsRightEditing = false;
		//	//	m_SystemStatue->mIsLeftEditing = false;
		//	//	break;

	case SpeechCommands::ActionNone:
	default:
		return true;
	}
	return true;
}

//The interactive logic for body posture (like jump~)
bool SystemControler::InterpretBodyPosture(){
	const double AttachingTime = 2000;
	const double AttachHintTime = 1000;
	switch (m_Mode)
	{
	case SystemControler::MODE_PAUSE:
		break;
	case SystemControler::MODE_SCAN:
		break;
	case SystemControler::MODE_SPATIAL_EDITING:
		{
			const auto& SKIdx = m_pMajorPlayer->Skeleton->Index;
			const auto& Model = m_pAvatar->Volume();
			typedef IndexedSkeleton::Index_Item idx_t;
			auto InsideJointCount = std::count_if(SKIdx.cbegin(),SKIdx.cend(),[&Model](const idx_t& idx) -> bool
			{
				return Model.Contains(idx.second->Entity[Current].Position);
			});
			if (InsideJointCount <= 8 || !IsConstTools[m_Slots[0][0].ToolID] || !IsConstTools[m_Slots[0][1].ToolID])
			{
				m_StayinTimer.Reset();
				if (m_Constraint == Constraint_Manipulate_Only)
				{
					m_pMainIndicator->Content = L"Please Attach";
				} else
				{
					m_pMainIndicator->Content = L"";
				}
				m_pMainIndicator->SetColor(Colors::LightGreen);

			} else {
				double currTime = m_StayinTimer.GetTime();

				if (currTime > AttachingTime + AttachHintTime) {
					m_StayinTimer.Reset();
					m_pMainIndicator->Content = L"";
					SwitchSystemMode(MODE_REFERENCE_EDITING);
					//m_pTextWrapper->CreateTimedFloatingText(L"Attached!",screenWidth / 2.0f,200.0f,1000,64,Colors::Crimson);
				} else {
					if (currTime > AttachHintTime)
					{
						float alpha = min<float>((float)((currTime - AttachHintTime)/ (AttachingTime)),1.0f);
						int countdown = 3 - (int)((alpha * 100.0f) / 33.33f);
						XMVECTOR vColor = XMVectorLerp(Colors::LightGreen,Colors::Crimson,alpha);
						wstringstream oss;
						oss << L"Attaching Body in "<<setw(1)<<countdown;

						m_pMainIndicator->Content = oss.str();
						m_pMainIndicator->SetColor(vColor);
					}
				}
			}
		}
		break;
	case SystemControler::MODE_REFERENCE_EDITING:
	case SystemControler::MODE_REFERENCE_PAINT:
		{
			const static HumanSkeleton::HumanJointEnum ArmJoints[] = {HumanSkeleton::SHOULDER_LEFT,HumanSkeleton::ELBOW_LEFT,HumanSkeleton::WRIST_LEFT,HumanSkeleton::HAND_LEFT};
			const float JumpHorizontalThreshold = 0.35f;
			const float JumpVecticalThreshold	= 0.3f;
			const float JumpOutThreshold		= 1.35f;
			const float WaveDownThreshold		= 2.0f;
			const float AttachingVelocityThreshold = 0.5f;

			float Speed = m_pMajorPlayer->Velocity();
			if (Speed > JumpOutThreshold)
			{
				// There's enough movement on both the x-z plane and the y direction
				// Switch to spatial mode
				std::cout << "Jumped out , Speed = " << m_pMajorPlayer->Velocity() << std::endl;
				//m_pAvatar->Skeleton->Root->Snap_Current_to_Default();
				ResetAvatarTransform();
				SwitchSystemMode(MODE_SPATIAL_EDITING);
				//m_pTextWrapper->CreateTimedFloatingText(L"Detached!",screenWidth / 2.0f,200.0f,1000,64,Colors::LightGreen);
			}

			bool HandsFree = (IsConstTools[m_Slots[0][0].ToolID] && IsConstTools[m_Slots[0][1].ToolID]);

			for (unsigned int hand = 0; hand < 2 ; hand ++)
			{

				if (!HandsFree)
				{
					m_HandTimers[hand].Reset();
					continue;
				}

				auto HandAttachedJoint	= m_pMotionAdaptor->IsAttached(HumanSkeleton::HAND_LEFT	+ hand*4);
				auto WristAttachedJoint = m_pMotionAdaptor->IsAttached(HumanSkeleton::WRIST_LEFT	+ hand*4);
				auto ElbowAttachedJoint = m_pMotionAdaptor->IsAttached(HumanSkeleton::ELBOW_LEFT	+ hand*4);

				auto attachedFlag = HandAttachedJoint || WristAttachedJoint || ElbowAttachedJoint ;

				if (!attachedFlag)
				{
					//if (!HandsFree) break;
					const auto& Model = m_pAvatar->Volume();
					vector<Vector3> Arm(3);
					Arm[0] = m_pMajorPlayer->Skeleton->GetActualJoint(HumanSkeleton::HAND_LEFT + hand*4);
					Arm[1] = m_pMajorPlayer->Skeleton->GetActualJoint(HumanSkeleton::WRIST_LEFT + hand*4);
					Arm[2] = m_pMajorPlayer->Skeleton->GetActualJoint(HumanSkeleton::ELBOW_LEFT + hand*4);
					// Elbow and one of (wrist,hand) inside model
					bool AllowedAttachFlag = Model.Contains(Arm[0]) && (Model.Contains(Arm[1]) || Model.Contains(Arm[2]));

					float DispL = m_pMajorPlayer->HandVelovity(hand);
					if (AllowedAttachFlag && DispL < AttachingVelocityThreshold)
					{
						double currTime = m_HandTimers[hand].GetTime();

						//cout << (HandSwitch)hand <<"Arm Inside Avatar."<<endl;


						if (currTime > AttachingTime + AttachHintTime) // hover long enough , attach it
						{
							m_HandTimers[hand].Reset();
							m_Slots[0][hand][Drive]->Start();
							if (!m_Slots[0][hand][Drive]->IsActive())
							{
								// Attach failed
								m_pToolsIndicator[hand]->Content = L"";
								m_pToolsIndicator[hand]->SetColor(Colors::LightGreen);
							} else
							{
								m_HandTimers[hand].Pause();
								m_HandTimers[hand].Reset();
								if (m_Mode == MODE_REFERENCE_EDITING)
									m_Slots[0][hand][Cursor]->Finish();
								else
									m_Slots[0][hand][HoverPickCursor]->Finish();
								m_Slots[0][hand].ToolID = Drive;
								//auto IShoulder = (HumanSkeleton::SHOULDER_LEFT + hand * 4);
								//m_pAttacher->Detach(IShoulder);
								//m_pAttacher->AutoAttach(IShoulder);

								m_pToolsIndicator[hand]->Content = wstring(HandSwitchName[hand]) + L"\nAttached";
								m_pToolsIndicator[hand]->SetColor(Colors::Crimson);
								XMVECTOR vPosWS = m_pMajorPlayer->Skeleton->GetActualJoint(HumanSkeleton::HAND_LEFT + hand*4);
								auto Pos = TransformWorldCoordIntoScreenCoord(vPosWS);
								m_pTextWrapper->CreateTimedFloatingText(L"Attached!",Pos.x,Pos.y,1000,64,Colors::Crimson);
							}
						} else // not enough time , observe it
						{
							if (currTime > AttachHintTime) // give a hint for attaching is in progress
							{
								float alpha = min<float>((float)((currTime - AttachHintTime)/ (AttachingTime)),1.0f);
								int countdown = 3 - (int)((alpha * 100.0f) / 33.33f);
								XMVECTOR vColor = XMVectorLerp(Colors::LightGreen,Colors::Crimson,alpha);
								wstringstream oss;

								oss << HandSwitchName[hand]<<endl<<L"Attaching in "<<setw(1)<<countdown;

								m_pToolsIndicator[hand]->Content = oss.str();
								m_pToolsIndicator[hand]->SetColor(vColor);
							}
						}

					} else // Joint outside or speed too high
					{
						m_HandTimers[hand].Reset();
						m_pToolsIndicator[hand]->Content = L"";
						m_pToolsIndicator[hand]->SetColor(Colors::LightGreen);
					}
				} else // Already attached , test if detach
				{
					float DispL = m_pMajorPlayer->HandVelovity(hand);
					//cout <<(HandSwitch)hand << " Velocity : "<<DispL <<endl;
					if ((3.0f > DispL) && (DispL > WaveDownThreshold))
					{
						m_Slots[0][hand][Drive]->Finish();
						if (m_Mode == MODE_REFERENCE_EDITING)
						{
							m_Slots[0][hand][Cursor]->Start();
							m_Slots[0][hand].ToolID = Cursor;
						}
						else
						{
							m_Slots[0][hand][HoverPickCursor]->Start();
							m_Slots[0][hand].ToolID = HoverPickCursor;
						}
						//for (auto&& IJoint : ArmJoints)
						//{
						//	auto Id = IJoint + hand*4;
						//	auto pAvatarJoint = m_pAttacher->FindAttachedJoint(m_pMajorPlayer->Skeleton->at(Id));
						//	if (pAvatarJoint)
						//	{
						//		pAvatarJoint->Snap_Current_to_Default();
						//	}
						//}
						//m_pAttacher->Detach(HumanSkeleton::SHOULDER_LEFT + hand * 4);
						m_HandTimers[hand].Unpause();
						m_HandTimers[hand].Reset();

						XMVECTOR vPosWS = m_pMajorPlayer->Skeleton->GetActualJoint(HumanSkeleton::HAND_LEFT + hand*4);
						auto Pos = TransformWorldCoordIntoScreenCoord(vPosWS);
						if (hand ==0)
							m_pTextWrapper->CreateTimedFloatingText(L"Left Hand\nDetached!",200.0f,128.0f,1000,64,Colors::LightGreen);
						else
							m_pTextWrapper->CreateTimedFloatingText(L"Right Hand\nDetached!",screenWidth-200.0f,128.0f,1000,64,Colors::LightGreen);
						//m_pTextWrapper->CreateTimedFloatingText(L"Detached!",Pos.x,Pos.y,1000,64,Colors::LightGreen);
						m_pToolsIndicator[hand]->Content = L"";
						m_pToolsIndicator[hand]->SetColor(Colors::LightGreen);
					}
				}
			}


			// Detecting WavingHand
		}
		break;
	case SystemControler::MODE_ANIMATE:
		break;
	default:
		break;
	}
	return true;
}

const SystemControler::ToolsSet* SystemControler::GetToolPalette()
{
	if (m_Constraint == Constraint_None)
	{
		if (m_Mode == MODE_REFERENCE_EDITING)
		{
			return ToolsForSingleGestures;
		} else if (m_Mode == MODE_REFERENCE_PAINT)
		{
			return ToolsForPaintGestures;
		} else if (m_Mode == MODE_SPATIAL_EDITING)
		{
			return ToolsForSpetialEditing;
		} else
		{
			return nullptr;
		}
	} else
	{
		return ToolsForConstraints[m_Constraint];
	}
	return nullptr;
}

bool SystemControler::UpdatePlayerGesture(const MultiMouseControler& Mice)
{
	bool flag = false;
	for (auto& player : m_pPlayers)
	{
		if (player->UpdateGestures(Mice))
			flag = true;
	}
	return flag;
}

void SystemControler::AdaptArmLength()
{
	float Radius = m_pAvatar->Volume().BoundingSphere.Radius;
	for (auto& player : m_pPlayers)
	{
		player->ArmScalingRadius = m_pAvatar->Volume().BoundingSphere.Radius * 1.2f;
	}
}



//The interactive logic for gesture input
bool SystemControler::InterpretGesture(){
	bool hr = UpdatePlayerGesture(m_pInput->Mice());
	if (!hr) 
		return false;
	switch (m_Mode)
	{
	case SystemControler::MODE_PAUSE:
		break;
	case SystemControler::MODE_SCAN:
		{
			auto pPlayer = m_pMajorPlayer;
			GestureEnum Gestures[2] = { pPlayer->ReadGesture(LeftHand),pPlayer->ReadGesture(RightHand)};

			if (!m_pAvatar->Empty() && (Gestures[LeftHand]==FREE || Gestures[RightHand]==FREE))
			{
				if (m_Constraint == Constraint_Scan_Only)
				{
					//Attaching();
					//SwitchSystemMode(MODE_PAUSE);
				}else
					//{
					SwitchSystemMode(MODE_SPATIAL_EDITING);
				//}
			}
		}
		break;
	case SystemControler::MODE_SPATIAL_EDITING:
	case SystemControler::MODE_REFERENCE_EDITING:
	case SystemControler::MODE_REFERENCE_PAINT:
		{
			// Interpret hand gesture event
			for (int player = 0; player < MAX_PLAYER_COUNT; player++)
			{
				if (!m_pPlayers[player]->IsActive()) continue;

				const ToolsSet* ToolPalette = GetToolPalette();
				auto pPlayer = m_pPlayers[player].get();
				GestureEnum Gestures[2] = { pPlayer->ReadGesture(LeftHand),pPlayer->ReadGesture(RightHand)};
				// It's a double hand gesture , change tools Palette
				if ((m_Constraint == Constraint_None || m_Constraint == Constraint_Double_Sculpt_Only || m_Constraint == Constraint_Manipulate_Only) && Gestures[LeftHand]==Gestures[RightHand] && Gestures[LeftHand]!=FREE && Gestures[LeftHand]!=UNUPDATED)
				{
					std::cout<<"Double Hand Gesture"<<std::endl;
					if (m_Mode == MODE_REFERENCE_EDITING )
					{
						if (m_Slots[player][0][Drive]->IsActive() || m_Slots[player][1][Drive]->IsActive()) continue;
						if (m_Constraint != Constraint_Double_Sculpt_Only)
							ToolPalette = ToolsForDoubleGestures;
						else
							ToolPalette = ToolsForDoubleSculptConstraint;
					} else if (m_Mode == MODE_SPATIAL_EDITING)
					{
						if (m_Constraint != Constraint_Manipulate_Only)
							ToolPalette = ToolsForSpetialEditingDoubleGesture;
						else
							ToolPalette = ToolsForSpetialManipulateConstraint;
					}
				}

				for (int hand = 0; hand < 2; hand++)
				{
					if (m_Slots[player][hand][Drive]->IsActive()) 
					{
						/// Add hint for trying to do editing when attached here.
						float x = (hand == LeftHand)? 145.0f : (screenWidth - 145.0f);

						if (Gestures[hand] != UNUPDATED)
							m_pTextWrapper->CreateTimedFloatingText(L"Please\nDetach",x,200,1500,64,Colors::LightGreen);
						continue;
					}

					if (Gestures[hand] != UNUPDATED && m_Slots[player][hand].ToolID!=ToolPalette[Gestures[hand]])
					{
						std::cout << (HandSwitch)hand <<" Tool changed : "<<m_Slots[player][hand].ToolID<<" -> "<<ToolPalette[Gestures[hand]] <<std::endl;
						// If finished operation isn't a constant operation , adapt arm's length to fit it.
						//if (!IsConstTools[m_Slots[player][hand].ToolID] && ToolsSideEffectType[m_Slots[player][hand].ToolID]==SideEffectType::SideEffect_OnConfirm) 
						if (ToolsSideEffectType[m_Slots[player][hand].ToolID] == SideEffectType::SideEffect_OnConfirm) 
						{
							if (m_Slots[player][hand].ToolID != Vculpt || hand==LeftHand)
								SnapShoot();
						}/* else if (!dynamic_cast<CutManager*>(m_Slots[player][hand][Cut])->CutedFlag())
						 {

						 }*/

						m_Slots[player][hand].Tool()->Finish();

						if (!IsConstTools[m_Slots[player][hand].ToolID]) 
						{
							AdaptArmLength();
							if (m_Slots[player][hand].ToolID == Cut && m_pAvatar->Volume().size() == 0)
							{
								m_pTextWrapper->CreateTimedFloatingText(L"You have cut all things down...",screenWidth/2.0f,200.0f,1000,64,Colors::Purple);
								//Scanning();
							}
						}

						m_Slots[player][hand].ToolID = ToolPalette[Gestures[hand]];

						m_Slots[player][hand].Tool()->Start();
						if (ToolsSideEffectType[m_Slots[player][hand].ToolID]==SideEffectType::SideEffect_OnEditing)
						{
							SnapShoot();
						}

					}
				}

				bool bothHandFree = IsConstTools[m_Slots[player][0].ToolID] && IsConstTools[m_Slots[player][1].ToolID];
				//if (!bothHandFree)
				{
					for (int hand = 0; hand < 2; hand++)
					{
						if (!IsConstTools[m_Slots[player][hand].ToolID]) 
						{
							if (Gestures[hand] != UNUPDATED)
								m_pToolsIndicator[hand]->Content = ToolsName[ToolPalette[Gestures[hand]]];
						} else if (!m_Slots[player][hand][Drive]->IsActive() || m_Mode == MODE_SPATIAL_EDITING)
						{
							m_pToolsIndicator[hand]->Content = L"";
							m_pToolsIndicator[hand]->SetColor(Colors::LightGreen);
						}
					}
				}
			}
		}
		break;
	case SystemControler::MODE_ANIMATE:
		break;
	default:
		break;
	}
	return false;
}

bool SystemControler::LoadingAvatarFromFile(const std::wstring & fileName)
{
	std::cout<<"Loading File..."<<endl;

	try{
		auto package = Load(fileName);
		std::cout<<"File loaded. Deserializing package..."<<endl;
		if (m_Mode >= MODE_REFERENCE_EDITING )
			SnapShoot(); // Push current things into backup stack
		m_pAvatar->LoadPackage(std::move(package));
		std::cout<<"Avatar loaded successful."<<endl;
		SwitchSystemMode(MODE_SPATIAL_EDITING);
		ResetAvatarTransform();
		m_ScalingFlag = false;
	} catch (ios::failure exception)
	{
		std::cout<<"File Not Found!"<<endl;
		return false;
	} catch	(...)
	{
		std::cout<<"File load failed."<<endl;
		return false;
	}
	return true;
}
bool SystemControler::SavingAvatarToFile(const std::wstring & fileName) const
{
	std::cout<<"Saving File..."<<endl;
	auto package = m_pAvatar->SavePackage(m_pGraphics->Context());
	Save(fileName,package);
	auto textureName = fileName + L".dds";
	if (package.pTexture.get())
		DirectX::SaveDDSTextureToFile(m_pGraphics->Context(),package.pTexture->Resource(),textureName.c_str());
	//m_pTextWrapper->CreateTimedFloatingText(L"File Saved!",screenWidth/2.0f,100.0f,1000,64,Colors::Gold);
	std::cout<<"File saved successfully..."<<endl;
	return true;
}



/// <summary>
/// Interprets the misc input.
/// </summary>
/// <returns></returns>
bool SystemControler::InterpretMisc(){
	// Keyboard have the most priority for control
	//Escape when press ESC
	if(m_pInput->IsKeyDown(VK_ESCAPE)) {
		return false;
	}
	if(m_pInput->IsKeyDown(VK_PRIOR)){
		m_pGraphics->m_Camera->Move(DirectX::Vector3(0.0f,0.03f,0.0f));
	}

	if(m_pInput->IsKeyDown(VK_NEXT)){
		m_pGraphics->m_Camera->Move(DirectX::Vector3(0.0f,-0.03f,0.0f));
	}

	if(m_pInput->IsKeyDown(VK_RIGHT)){
		m_pGraphics->m_Camera->Move(DirectX::Vector3(-0.03f,0.0f,0.0f));
	}

	if(m_pInput->IsKeyDown(VK_LEFT)){
		m_pGraphics->m_Camera->Move(DirectX::Vector3(0.03f,0.0f,0.0f));
	}
	if(m_pInput->IsKeyDown(VK_UP)){
		m_pGraphics->m_Camera->Move(DirectX::Vector3(0.0f,0.0f,0.03f));
	}

	if(m_pInput->IsKeyDown(VK_DOWN)){
		m_pGraphics->m_Camera->Move(DirectX::Vector3(0.0f,0.0f,-0.03f));
	}
	if(m_pInput->IsKeyDown(VK_NUMPAD4)){
		m_pGraphics->m_Camera->Turn(DirectX::XMQuaternionRotationRollPitchYaw(0.0f,0.03f,0.0f));
	}
	if(m_pInput->IsKeyDown(VK_NUMPAD6)){
		m_pGraphics->m_Camera->Turn(DirectX::XMQuaternionRotationRollPitchYaw(0.0f,-0.03f,0.0f));
	}
	if(m_pInput->IsKeyDown(VK_NUMPAD8)){
		m_pGraphics->m_Camera->Turn(DirectX::XMQuaternionRotationRollPitchYaw(-0.03f,0.0f,0.0f));
	}
	if(m_pInput->IsKeyDown(VK_NUMPAD2)){
		m_pGraphics->m_Camera->Turn(DirectX::XMQuaternionRotationRollPitchYaw(0.03f,0.0f,0.0f));
	}
	if(m_pInput->IsKeyDown(VK_NUMPAD5)){
		m_pGraphics->m_Camera->Position.Set(0.0f, 1.0f, 0.0f);
		m_pGraphics->m_Camera->Orientation = XMQuaternionIdentity();
	}
	if (m_pInput->IsKeyHit(VK_TAB))
	{
		bool bothHandFree = IsConstTools[m_Slots[0][0].ToolID] && IsConstTools[m_Slots[0][1].ToolID]
		&&	IsConstTools[m_Slots[1][0].ToolID] && IsConstTools[m_Slots[1][1].ToolID];
		if (m_pPlayers[1]->Tracked() && bothHandFree) 
		{
			m_pPlayers[0]->swap(*m_pPlayers[1]);
		}
	}
	if (m_pInput->IsKeyHit(VK_F12))
	{
		m_pPlayers[1]->Enabled = !m_pPlayers[1]->Enabled;
		m_pPlayers[1]->Visiable = m_pPlayers[1]->Enabled;
		if (m_pPlayers[1]->Enabled)
			cout<<"Player 2 Enabled."<<endl;
		else
			cout<<"Player 2 Disabled."<<endl;
	}


	if(m_pInput->IsKeyHit('W')){
		m_pGraphics->Mode_Rasterizer = GraphicsClass::Wireframe;
	}
	if(m_pInput->IsKeyHit('E')){
		m_pGraphics->Mode_Rasterizer = GraphicsClass::CullCounterClockwise;
	}
	if(m_pInput->IsKeyHit('T')){
		if (m_pGraphics->Mode_Blend != GraphicsClass::AlphaBlend){
			m_pGraphics->Mode_Blend = GraphicsClass::AlphaBlend;
		}else
		{
			m_pGraphics->Mode_Blend = GraphicsClass::Opaque;
		}
	}
	if (m_pInput->IsKeyHit('S'))
	{
		RenderMode ^= DynamicMetaBallModel::SKELETON_ANIMATED;
		//RefreshRenderMode();
	}
	if (m_pInput->IsKeyHit('C') || (m_pInput->IsKeyDown(VK_CONTROL)) && (m_pInput->IsKeyHit('Z')))
	{
		CancelLastOperation();
	}
	if (m_pInput->IsKeyHit('X') || (m_pInput->IsKeyDown(VK_SHIFT)) && (m_pInput->IsKeyHit('Z')))
	{
		if (!Redo())
			std::cout<< "No operation can be redo yet." <<endl;
	}

	if (m_pInput->IsKeyHit('D'))
	{
		RenderMode ^= DynamicMetaBallModel::SKELETON_DEFAULT;
		//RefreshRenderMode();
	}
	if (m_pInput->IsKeyHit('A'))
	{
		RenderMode ^= DynamicMetaBallModel::SKIN_ANIMATED;
		//RefreshRenderMode();
		if (RenderMode.Contains(DynamicMetaBallModel::SKIN_ANIMATED))
			m_pAvatar->Update();
	}
	if (m_pInput->IsKeyHit('F'))
	{
		RenderMode ^= DynamicMetaBallModel::METABALLS_ANIMATED;
		//RefreshRenderMode();
	}

	if (m_pInput->IsKeyHit('H'))
	{
		for (auto& player : m_pPlayers)
		{
			player->Visiable = !player->Visiable;
		}
	}

	if (m_pInput->IsKeyHit(VK_OEM_4))
	{
		if (!TurnRadius(-0.01f))
			std::cout<<"You can't getting Smaller."<<std::endl;
	}

	if (m_pInput->IsKeyHit(VK_OEM_6))
	{
		if (!TurnRadius(0.01f))
			std::cout<<"You can't getting Bigger."<<std::endl;
	}

	if (m_pInput->IsKeyHit(VK_SPACE))
	{
		m_Paused = !m_Paused;
	}

	if (m_pInput->IsKeyHit(VK_OEM_COMMA))
	{
		if (SkeletonTransparency < 1.0f)
			SkeletonTransparency += 0.1f;
	}

	if (m_pInput->IsKeyHit(VK_OEM_PERIOD))
	{
		if (SkeletonTransparency > 0.0f)
			SkeletonTransparency -= 0.1f;
	}

	// Mode Switching
	if (m_pInput->IsKeyHit('U')){
		if (m_Mode != MODE_SCAN ) {
			SwitchSystemMode(MODE_SCAN);
		}
	}
	if (m_pInput->IsKeyHit('I')){
		if (m_Mode != MODE_SPATIAL_EDITING ) {
			SwitchSystemMode(MODE_SPATIAL_EDITING);
		}
	}
	if (m_pInput->IsKeyHit('O')){
		if (m_Mode != MODE_REFERENCE_EDITING ) {
			SwitchSystemMode(MODE_REFERENCE_EDITING);
		}
	}
	if (m_pInput->IsKeyHit('P')){
		if (m_Mode != MODE_REFERENCE_PAINT ) {
			SwitchSystemMode(MODE_REFERENCE_PAINT);
		}
	}
	if (m_pInput->IsKeyHit('Y')){
		if (m_Mode == MODE_SPATIAL_EDITING || m_Mode == MODE_SPATIAL_PAINT)
		{
			m_pAvatar->AnimationUpdate();
			auto Box = m_pAvatar->Volume().GetBoundingBox();
			//m_pAvatar->RotateTo(Quaternion::Identity(),XMLoadFloat3(&Box.Center));
			//m_pAvatar->Ground(0.0f);
		}
	}

	if (m_pInput->IsKeyHit(VK_F4)){

		LoadWithCommandLineInput();
	}

	if (m_pInput->IsKeyHit(VK_F2)){
		SaveWithCommandLineInput();
	}

	if (m_pInput->IsKeyHit(VK_OEM_3) || m_pInput->IsKeyHit(8))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_None);
	}

	if (m_pInput->IsKeyHit('1'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Drag_Only);
	}
	if (m_pInput->IsKeyHit('2'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Sculpt_Only);
	}
	if (m_pInput->IsKeyHit('3'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Cut_Only);
	}
	if (m_pInput->IsKeyHit('4'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Grow_Only);
	}
	if (m_pInput->IsKeyHit('5'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Double_Sculpt_Only);
	}
	if (m_pInput->IsKeyHit('6'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Paint_Only);
	}
	if (m_pInput->IsKeyHit('7'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Fill_Only);
	}
	if (m_pInput->IsKeyHit('8'))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Manipulate_Only);
	}
	if (m_pInput->IsKeyHit(VK_OEM_PLUS))
	{
		SwitchConstraintMode(ConstraintMode::Constraint_Scan_Only);
	}


	// Mouse wheels
	{
		auto& Mice = m_pInput->Mice();
		pair<unsigned short,short> command(100,0); 
		for (unsigned int i = 0; i < Mice.size(); i++)
		{
			if (Mice[i].EventFlag & MultiMouseControler::WheelEvent)
			{
				Mice[i].EventFlag &= ~MultiMouseControler::WheelEvent;
				if (Mice[i].Priority < command.first)
				{
					command.second = Mice[i].WheelDelta;
					command.first = Mice[i].Priority;
					Mice[i].WheelDelta = 0;
				} else if (Mice[i].Priority == command.first)
				{
					command.second += Mice[i].WheelDelta;
					Mice[i].WheelDelta = 0;
				}
			}
		}

		if (command.second != 0)
		{
			if (!TurnRadius(command.second * 0.01f))
			{
				if (command.second > 0)
				{
					cout << "You can not make it bigger"<<endl;
				} else
				{
					cout << "You can not make it smaller"<<endl;
				}
			}


		}

	}

	return true;
}




bool SystemControler::AbortOperations()
{
	if (m_Mode != MODE_REFERENCE_EDITING && m_Mode != MODE_REFERENCE_PAINT && m_Mode != MODE_SPATIAL_EDITING) 
		return false;
	bool AbortedFlag = false;
	for (int player = 0; player < MAX_PLAYER_COUNT; player++)
	{
		if (!m_pPlayers[player]->IsActive()) continue;
		for (int hand = 0; hand < 2; hand++)
		{
			if ((!IsConstTools[m_Slots[player][hand].ToolID]) && m_Slots[player][hand].Tool()->IsActive())
			{
				AbortedFlag = true;
				m_Slots[player][hand].Tool()->Abort();
				m_Slots[player][hand].ToolID = GetToolPalette()[InputControler::Gestures::FREE];
				m_Slots[player][hand].Tool()->Start();
			}
		}
	}
	if (AbortedFlag)
		m_pTextWrapper->CreateTimedFloatingText(L"Operation Aborted!",(float)screenWidth / 2.0f,200,2000,64);
	return AbortedFlag;
}

void SystemControler::AutoSaving() const
{
	static wstring AutoFileNames[] = {
		L"AutoSave_0.sav",
		L"AutoSave_1.sav",
		L"AutoSave_2.sav",
		L"AutoSave_3.sav",
		L"AutoSave_4.sav"
		L"AutoSave_5.sav"
		L"AutoSave_6.sav"
		L"AutoSave_7.sav"
		L"AutoSave_8.sav"
		L"AutoSave_9.sav"
	};
	std::cout<<"Auto saving.."<<std::endl;
	WIN32_FIND_DATA FindFileData;
	std::wstring ValiadFileName = CurrentDirectory + SavePath + L"AutoSave_*.sav";
	auto hFind = FindFirstFile(ValiadFileName.c_str(),&FindFileData);
	std::wstring TargetFileName;
	if (hFind == INVALID_HANDLE_VALUE)
	{
		TargetFileName = SavePath + L"AutoSave_0.sav";
	} else
	{
		std::vector<WIN32_FIND_DATA> AutoFiles;
		AutoFiles.emplace_back(FindFileData);
		while (FindNextFile(hFind, &FindFileData)!= 0)
		{
			AutoFiles.emplace_back(FindFileData);
		}
		if (AutoFiles.size() < 5)
		{
			TargetFileName = SavePath + AutoFileNames[AutoFiles.size()];
		}
		else
		{
			auto oldestFile = std::min_element(AutoFiles.cbegin(),AutoFiles.cend(),[](const WIN32_FIND_DATA& lhs,const WIN32_FIND_DATA& rhs) -> bool {
				SYSTEMTIME LSysTime , RSysTime;
				FileTimeToSystemTime(&lhs.ftLastWriteTime,&LSysTime);
				FileTimeToSystemTime(&rhs.ftLastWriteTime,&RSysTime);
				if (LSysTime.wYear != RSysTime.wYear) 
					return LSysTime.wYear < RSysTime.wYear; 
				else if (LSysTime.wMonth != RSysTime.wMonth) 
					return LSysTime.wMonth < RSysTime.wMonth; 
				else if (LSysTime.wDay != RSysTime.wDay) 
					return LSysTime.wDay < RSysTime.wDay; 
				else if (LSysTime.wHour != RSysTime.wHour) 
					return LSysTime.wHour < RSysTime.wHour; 
				else if (LSysTime.wMinute != RSysTime.wMinute) 
					return LSysTime.wMinute < RSysTime.wMinute; 
				else if (LSysTime.wSecond != RSysTime.wSecond) 
					return LSysTime.wSecond < RSysTime.wSecond;
				return false;
			});
			TargetFileName = SavePath + oldestFile->cFileName;
		}
	}
	FindClose(hFind);
	SavingAvatarToFile(TargetFileName);
	std::wcout<<"Auto saving finished , file save to : "<<TargetFileName<<std::endl;
}

void SystemControler::LoadLatestSave()
{
	std::cout<<"Load Latest auto saving..."<<std::endl;
	WIN32_FIND_DATA FindFileData;
	std::wstring ValiadFileName = CurrentDirectory + SavePath + L"*.sav";
	auto hFind = FindFirstFile(ValiadFileName.c_str(),&FindFileData);
	std::wstring TargetFileName;
	if (hFind == INVALID_HANDLE_VALUE)
	{
		std::cout<<"No file found for loading."<<std::endl;
	} else
	{
		std::vector<WIN32_FIND_DATA> AutoFiles;
		AutoFiles.emplace_back(FindFileData);
		while (FindNextFile(hFind, &FindFileData)!= 0)
		{
			AutoFiles.emplace_back(FindFileData);
		}

		auto latesttFile = std::max_element(AutoFiles.cbegin(),AutoFiles.cend(),[](const WIN32_FIND_DATA& lhs,const WIN32_FIND_DATA& rhs) -> bool {
			SYSTEMTIME LSysTime , RSysTime;
			FileTimeToSystemTime(&lhs.ftLastWriteTime,&LSysTime);
			FileTimeToSystemTime(&rhs.ftLastWriteTime,&RSysTime);
			if (LSysTime.wYear != RSysTime.wYear) 
				return LSysTime.wYear < RSysTime.wYear; 
			else if (LSysTime.wMonth != RSysTime.wMonth) 
				return LSysTime.wMonth < RSysTime.wMonth; 
			else if (LSysTime.wDay != RSysTime.wDay) 
				return LSysTime.wDay < RSysTime.wDay; 
			else if (LSysTime.wHour != RSysTime.wHour) 
				return LSysTime.wHour < RSysTime.wHour; 
			else if (LSysTime.wMinute != RSysTime.wMinute) 
				return LSysTime.wMinute < RSysTime.wMinute; 
			else if (LSysTime.wSecond != RSysTime.wSecond) 
				return LSysTime.wSecond < RSysTime.wSecond;
			return false;
		});
		TargetFileName = SavePath + latesttFile->cFileName;
		LoadingAvatarFromFile(TargetFileName);
		std::wcout<<"Latest save file loaded : "<<TargetFileName<<std::endl;
	}
	FindClose(hFind);
}

void SystemControler::LoadWithCommandLineInput()
{
	std::wstring TargetFileName;

	ShowWindow(m_hwnd,0);
	ShowWindow(m_hConsole,1);
	MoveWindow(m_hConsole,0,0,800,500,1);
	GetFocus();
	do
	{
		std::wcout<<L"LOADING : Please input the load file name."<<std::endl<<L"Load file name : .\\Save\\";
		std::wcin>>TargetFileName;
	} while ((TargetFileName != L"return") && !LoadingAvatarFromFile(SavePath+TargetFileName+SaveFileExtension));

	MoveWindow(m_hConsole,-800,50,750,500,1);
	ShowWindow(m_hwnd,1);
	GetFocus();
}

void SystemControler::SaveWithCommandLineInput()
{
	std::wstring TargetFileName;

	ResetAvatarTransform();
	SwitchSystemMode(MODE_SPATIAL_EDITING);

	ShowWindow(m_hwnd,0);
	ShowWindow(m_hConsole,1);
	MoveWindow(m_hConsole,0,0,800,500,1);
	GetFocus();
	do
	{
		std::wcout<<L"SAVING : Please input the save file name."<<std::endl<<L"Save file name : .\\Save\\";
		std::wcin>>TargetFileName;
	} while ((TargetFileName != L"return") && !SavingAvatarToFile(SavePath+TargetFileName+SaveFileExtension));

	MoveWindow(m_hConsole,-800,50,750,500,1);
	ShowWindow(m_hwnd,1);
	GetFocus();
}

void SystemControler::ResetAvatarTransform()
{
	//m_pAttacher->ResetMotion();
	//m_pAvatar->ResetLocalTransform();
	//m_pAvatar->Skeleton->Root->Entity[Current].Position = m_pAvatar->Skeleton->Root->Entity[Default].Position;

	//m_pAttacher->ResetBaseState();
}

void SystemControler::SnapShoot()
{
	//m_pTextWrapper->CreateTimedFloatingText(L"Snapped!",(float)screenWidth / 2.0f,100.0f,1000);
	static unsigned int snapCount = 0;
	auto package = m_pAvatar->SavePackage(m_pGraphics->Context());
	if (m_UndoStack.size() >= 20)
		m_UndoStack.pop_front();
	++ snapCount;
	if (snapCount == 3)
	{
		snapCount = 0;
		std::thread AutoSavingThread([this]()
		{
			this->AutoSaving();
		});
		AutoSavingThread.detach();
	}

	m_UndoStack.emplace_back(std::move(package));
	m_RedoStack.clear();
}

void SystemControler::CancelLastOperation()
{
	std::cout<<"Canceling Operation..."<<endl;
	if (!AbortOperations())
	{
		cout<<"Recovering Model..."<<endl;
		bool hr = Undo();
		cout<<"Model Recover Finish."<<endl;
		if (hr)
		{
			m_pTextWrapper->CreateTimedFloatingText(L"Last Operation Canceled!",(float)screenWidth / 2.0f,200,2000,64,Colors::LightGreen);
			if (m_Mode == MODE_SCAN)
			{
				SwitchSystemMode(MODE_REFERENCE_EDITING);
			}
		}else
		{
			m_pTextWrapper->CreateTimedFloatingText(L"No operation could be canceled!",(float)screenWidth / 2.0f,200,2000,64,Colors::LightGreen);
			//SwitchSystemMode(MODE_SCAN);
		}
	}
	std::cout<<"Cancel Operation Finish."<<endl;
}


bool SystemControler::Undo()
{
	if (m_UndoStack.empty())
		return false;
	auto &package = m_UndoStack.back();
	m_pAvatar->LoadPackage(m_pGraphics->Context(),package);
	m_RedoStack.emplace_back(std::move(package));
	m_UndoStack.pop_back();
	return true;
}

bool SystemControler::Redo()
{
	if (m_RedoStack.empty())
		return false;
	auto &package = m_RedoStack.back();
	m_pAvatar->LoadPackage(m_pGraphics->Context(),package);
	m_UndoStack.emplace_back(std::move(package));
	m_RedoStack.pop_back();
	return true;
}


void SystemControler::Save(const std::wstring &fileName ,const DynamicMetaBallModel::Package &package) const
{
	auto blob = package.Serialize();
	std::ofstream fout(fileName,ios::binary | ios::out | ios::trunc);
	if (!fout.bad()){
		fout << blob;
		fout.close();
	};
}

DynamicMetaBallModel::Package SystemControler::Load(const std::wstring &fileName)
{
	std::ifstream fin(fileName,ios::binary | ios::in);
	ISerializable::Blob blob;
	if (!fin.bad()){
		fin >> blob;
		fin.close();
	} else
	{
		throw ios::failure("File not found.");
	}

	DynamicMetaBallModel::Package package;
	package.Deserialize(blob);
	blob.Release();
	return std::move(package);
}


bool SystemControler::TurnRadius(float delta)
{
	const float Superior_Limit = 0.3f;
	const float Infieror_Limit = 0.05f;
	if (m_UniformRadius + delta < Infieror_Limit)
	{
		delta = Infieror_Limit - m_UniformRadius;
	} else if (m_UniformRadius + delta > Superior_Limit)
	{
		delta = Superior_Limit - m_UniformRadius;
	}

	if (abs(delta) < 0.01f) return false;

	m_UniformRadius += delta;

	for (int player = 0; player < MAX_PLAYER_COUNT; player++)
		for (int hand = 0; hand < 2; hand++)
			for (int i = 0; i < Count; i++)
			{
				auto pAreaTool = dynamic_cast<EditingTools::IAreaTool*>(m_Slots[player][hand][i]);
				if (pAreaTool!=nullptr)
				{
					pAreaTool->TurnRadius(delta);
				}
			}
			return true;
}

bool SystemControler::SetRadius(float Radius)
{
	m_UniformRadius = Radius;
	for (int player = 0; player < MAX_PLAYER_COUNT; player++)
		for (int hand = 0; hand < 2; hand++)
			for (int i = 0; i < Count; i++)
			{
				auto pAreaTool = dynamic_cast<EditingTools::IAreaTool*>(m_Slots[player][hand][i]);
				if (pAreaTool!=nullptr)
				{
					pAreaTool->SetRadius(Radius);
				}
			}
			return true;
}

