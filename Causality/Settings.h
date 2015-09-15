#pragma once

namespace Causality
{
	const static size_t	g_PvDimension = 6U;

	extern bool			g_UseGeneralTransform;
	extern bool			g_UsePersudoPhysicsWalk;

	//					Use GPR instead of CCA for rigging local chain
	extern bool			g_UseStylizedIK;
	extern bool			g_UseVelocity;

	extern bool			g_EnableDebugLogging;
	extern bool			g_EnableRecordLogging;

	extern bool			g_StepFrame;

	//					When this flag set to true, subtle motion will be synthesised by major motion
	extern bool			g_EnableDependentControl;

	//					When this flag set to true, the kinect input's rotation will be ingnored
	extern bool			g_IngnoreInputRootRotation;

	extern float		g_FrameTimeScaleFactor;

	extern float		g_PlayerPcaCutoff; // 0.2^2
												   //static const float	EnergyCutoff = 0.3f;
	extern float		g_PlayerEnergyCutoff;

	extern float		g_BlendWeight;

	extern float		g_MatchAccepetanceThreshold;

	extern bool			g_EnableInputFeatureLocalization;

	extern bool			g_LoadCharacterModelParameter;

	extern float		g_CharacterPcaCutoff; // 0.2^2

	extern float		g_CharacterActiveEnergy;

	extern float		g_CharacterSubactiveEnergy;

}