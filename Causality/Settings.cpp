#include "Settings.h"

namespace Causality
{
	bool				g_UseGeneralTransform = false;
	bool				g_UsePersudoPhysicsWalk = false;

	//					Use GPR instead of CCA for rigging local chain
	bool				g_UseStylizedIK = true;
	bool				g_UseVelocity = true;

	bool				g_EnableDebugLogging = true;
	bool				g_EnableRecordLogging = false;

	bool				g_StepFrame = false;

	//					When this flag set to true, subtle motion will be synthesised by major motion
	bool				g_EnableDependentControl = false;

	//					When this flag set to true, the kinect input's rotation will be ingnored
	bool				g_IngnoreInputRootRotation = true;

	float				g_FrameTimeScaleFactor = 20;

	float				g_PlayerPcaCutoff = 0.04f; // 0.2^2
										   //static const float	EnergyCutoff = 0.3f;
	float				g_PlayerEnergyCutoff = 0.3f;

	float				g_BlendWeight = 0.8f;

	float				g_MatchAccepetanceThreshold = 0.2f;

	bool				g_EnableInputFeatureLocalization = true;

	bool				g_LoadCharacterModelParameter = true;
	float				g_CharacterPcaCutoff = 0.004f; // 0.2^2
	float				g_CharacterActiveEnergy = 0.40f;
	float				g_CharacterSubactiveEnergy = 0.02f;

}