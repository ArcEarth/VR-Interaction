////////////////////////////////////////////////////////////////////////////////
// Filename: systemclass.h
////////////////////////////////////////////////////////////////////////////////
#ifndef _SYSTEMCLASS_H_
#define _SYSTEMCLASS_H_


///////////////////////////////
// PRE-PROCESSING DIRECTIVES //
///////////////////////////////
#define WIN32_LEAN_AND_MEAN


//////////////
// INCLUDES //
//////////////
#define NOMINMAX
#include <windows.h>


///////////////////////
// MY CLASS INCLUDES //
///////////////////////
//#include "SystemStatues.h"
#include "MathHelper.h"
#include "InputControler.h"
#include "graphicsclass.h"
#include "HumanSkeleton.h"
#include "DynamicMetaBallModel.h"
#include "EditingToolsInterface.h"
#include "SpaceWarpperInterface.h"
#include "ColorPalette.h"
#include "Visualizers.h"
#include "BaseEffect.h"
#include "Player.h"
#include "SkyBox.h"
#include "PostProcesser.h"
#include "FloatingText.h"
#include "DebugVisualizer.h"
#include <stack>
////////////////////////////////////////////////////////////////////////////////
// Class name: SystemControler
////////////////////////////////////////////////////////////////////////////////
class SystemControler
{
public:
	enum HandSwitch{
		LeftHand = 0,
		RightHand = 1,
		DoubleHand = 2,
	};

	enum SystemMode
	{
		MODE_PAUSE = 0,
		MODE_SCAN,
		MODE_ATTACHING,
		MODE_SPATIAL_EDITING,
		MODE_REFERENCE_EDITING,
		MODE_REFERENCE_PAINT,
		MODE_SPATIAL_PAINT,
		MODE_ANIMATE,
		MODE_COUT,
	};

	enum ViewMode
	{
		View_Unknown,	// Unspecific state (such as MODE_PAUSE)
		View_Reference,	// 1st person mode
		View_Spatial,	// 3rd person mode
	};

	enum EditMode
	{
		Edit_None,
		Edit_Scan,
		Edit_Sculpt,
		Edit_Paint,
	};



	enum ConstraintMode
	{
		Constraint_None,
		Constraint_Drag_Only,
		Constraint_Sculpt_Only,
		Constraint_Cut_Only,
		Constraint_Grow_Only,
		Constraint_Paint_Only,
		Constraint_Fill_Only,
		Constraint_Double_Sculpt_Only,
		Constraint_Scan_Only,
		Constraint_Manipulate_Only,
		Constraint_Count,
	};


	enum ToolsSet
	{
		Cursor = 0, // Use as the cursor
		Drag, 
		Grow,
		Sweep, // The Surface sculpt , Single Hand
		Vculpt, // The Volume sculpt , Double Hand
		Cut,
		Sketch,
		Drive, // Use to drive the avatar body
		HoverPickCursor,
		Eyedropper,
		GaussBrush,
		FillBrush,
		Manipulate, // Handle Rigid Object Transform
		Count,
	};

	enum SideEffectType
	{
		SideEffect_None,
		SideEffect_OnConfirm,
		SideEffect_OnEditing,
	};

	struct ToolSlot
	{
		ToolsSet ToolID;
		EditingTools::IEditingTool *ToolsInSlot[ToolsSet::Count];
		std::shared_ptr<Geometrics::DynamicMetaBallModel> Previewer;
		inline EditingTools::IEditingTool * Tool()
		{
			return ToolsInSlot[ToolID];
		}
		EditingTools::IEditingTool*& operator[] (unsigned int index)
		{
			return ToolsInSlot[index];
		}

	};


public:
	SystemControler();
	SystemControler(const SystemControler&);
	~SystemControler();

	bool Initialize();
	void Shutdown();
	bool Frame();

	LRESULT CALLBACK MessageHandler(HWND, UINT, WPARAM, LPARAM);

private:
	void InitializeWindows(unsigned int&, unsigned int&);
	void ShutdownWindows();

	bool UpdateSkeleton(NUI_SKELETON_FRAME* pFrame);
	bool UpdatePlayerGesture(const MultiMouseControler& Mice);

	//The interactive logic for keyboard input , mouse wheel input , administrator mouse button , return false when press ESC
	bool InterpretMisc();
	//The interactive logic for speech input
	bool InterpretSpeech();
	//The interactive logic for hand gesture input
	bool InterpretGesture();
	//The interactive logic for body posture (like jump~)
	bool InterpretBodyPosture();
	//The interactive logic for update geometry date to graphic model (Refresh model)
	//bool InterpretUpdateSignal(const unsigned long long UpdateFlag);

	/// <summary>
	/// Transforms the world coordinate into screen coordinate.
	/// </summary>
	/// <param name="vPositionWS">The position in World Space. (unit:meter)</param>
	/// <returns> Returns the proper position within the screen space (unit:pixel)</returns>
	DirectX::XMFLOAT2 TransformWorldCoordIntoScreenCoord(DirectX::FXMVECTOR vPositionWS) const;

	void Scaning();
	void AutoScaling();
	void Attaching();
	void AdaptArmLength();

	void ExcuteOperations();

	bool AbortOperations();

	void Save(const std::wstring &fileName ,const Geometrics::DynamicMetaBallModel::Package &package) const;
	Geometrics::DynamicMetaBallModel::Package Load(const std::wstring &fileName);
	bool LoadingAvatarFromFile(const std::wstring & fileName=L"SaveFile.sav");
	bool SavingAvatarToFile(const std::wstring &fileName=L"SaveFile.sav") const;
	void AutoSaving() const;
	void LoadLatestSave();
	void LoadWithCommandLineInput();
	void SaveWithCommandLineInput();

	// Push the current state and model in stack , which could be recover from "Undo" command
	void SnapShoot();

	// Pop the state and model from stack
	bool Undo();
	// Pop the state from Redo-stack and turn it into Undo-stack
	bool Redo();


	void CancelLastOperation();

	void ResetAvatarTransform();

	// Radius Tune interface
	bool SetRadius(float Radius);
	bool TurnRadius(float delta);

	bool SwitchSystemMode(SystemMode mode);
	bool SwitchConstraintMode(ConstraintMode constraint);

	SystemMode SwitchViewMode(ViewMode viewMode);
	ViewMode CurrentViewMode() const;

	SystemMode SwitchEditMode(EditMode editMode);
	EditMode CurrentEditMode() const;

	void RenderScene();
	//void RefreshRenderMode();

	const ToolsSet* GetToolPalette();
private:
	LPCWSTR			m_applicationName;
	HINSTANCE		m_hinstance;
	HWND			m_hwnd;
	HWND			m_hConsole;
	unsigned int	screenWidth, screenHeight;
	std::thread		m_StreamingThread;

	std::wstring	CurrentDirectory;

	//Timer for handle the "Attach" event
	TIMER	m_StayinTimer;
	TIMER	m_ScalingTimer;
	//TIMER	m_AttachDisableTimer;
	std::array<TIMER,2>				m_HandTimers;

	// Text Blocks for basic text output
	std::array<IFloatingText*,2>	m_pToolsIndicator;
	IFloatingText*					m_pMainIndicator;
	IFloatingText*					m_pConstraintIndicator;
	IFloatingText*					m_pSnappedIndicator;

	bool	m_ScalingFlag;
	bool	EnableEidting;
	float	SkeletonTransparency;

	float	m_UniformRadius;
	SystemMode m_Mode;
	ConstraintMode	m_Constraint;
	bool	m_Paused;
	// Handel the input event

	std::unique_ptr<InputControler>			m_pInput;

	Player*					m_pMajorPlayer;
	std::unique_ptr<Player> m_pPlayers[MAX_PLAYER_COUNT];

	// For filtering , grounding and arranging the input skeleton from Kinect
	std::unique_ptr<PlayerSkeletonUpdater> m_pPlayerUpdater;

	// avatar data
	std::shared_ptr<Geometrics::DynamicMetaBallModel> m_pAvatar;
	std::list<Geometrics::DynamicMetaBallModel::Package> m_UndoStack;
	std::list<Geometrics::DynamicMetaBallModel::Package> m_RedoStack;
	// Handel the "Attach/Detach" operation
	//std::unique_ptr<Kinematics::AttachMap>			m_pAttacher;
	std::shared_ptr<Kinematics::HS_MB_Adaptor>		m_pMotionAdaptor;
	//std::unique_ptr<Geometrics::MetaBallModel>		m_pPuppet;
	std::unique_ptr<MetaballViewer>					m_pPuppetViewer;
	//Handel the DirectX device and other useful interface
	GraphicsClass* m_pGraphics;
	// Handel the text display
	std::unique_ptr<FloatingTextWrapper>			m_pTextWrapper;
	// Handel the coordinate transform and snap
	//IWarpper*										m_pWarpper;
	// Effect for rendering
	std::unique_ptr<DirectX::IEffect>				m_pEffect;
	std::unique_ptr<DirectX::IEffect>				m_pIndicatorEffect;
	// Extra structures for rendering
	std::unique_ptr<DirectX::PostProcesser>			m_pPostProcessor;
	std::unique_ptr<DirectX::DepthStencilBuffer>	m_pDepthBuffer;

	// Environment Models
	std::unique_ptr<DirectX::SkyBox>				m_pSkyBox;
	std::unique_ptr<DirectX::FloorPlane>			m_pFloor;
	// Color palette
	std::shared_ptr<EditingTools::PaintTools::ColorPalette> m_pColorPalette;

	static const ToolsSet ToolsForSingleGestures[5];
	static const ToolsSet ToolsForDoubleGestures[5];
	static const ToolsSet ToolsForPaintGestures[5];
	static const ToolsSet ToolsForConstraints[][5];
	static const ToolsSet ToolsForDoubleSculptConstraint[5];
	static const ToolsSet ToolsForSpetialEditing[5];
	static const ToolsSet ToolsForSpetialEditingDoubleGesture[5];
	static const ToolsSet ToolsForSpetialManipulateConstraint[5];

	static const bool IsConstTools[ToolsSet::Count];
	static const SideEffectType ToolsSideEffectType[ToolsSet::Count];
	//ToolsSet m_CurrentTools[2];
	//EditingTools::IEditingTool *m_pTools[2][ToolsSet::Count];
	//Geometrics::DynamicMetaBallModel* m_pEditingPreviews[2];

	//ToolSlot m_Slots[4];
	ToolSlot m_Slots[MAX_PLAYER_COUNT][2];
	//Handel of the SystemStatus
//	SystemStatueStruct *m_SystemStatue;
	CompositeFlag<Geometrics::DynamicMetaBallModel::RenderMode> RenderMode;


};


#endif