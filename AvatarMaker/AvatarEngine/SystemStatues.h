#ifndef SYSTEM_STATUES_H
#define SYSTEM_STATUES_H
typedef struct _SystemStatueStruct
{
	bool SpatialMapping;
	bool ReferenceMapping;

	bool mIsScanned;

	bool mIsRightEditing;
	bool mIsLeftEditing;

	float smallR;
	float bigR;

	float mBodyJumpHorizonThreshold;
	float mBodyJumpVecticalThreshold;
	//Default constructor for initialize
	_SystemStatueStruct(){
		SpatialMapping = false;
		ReferenceMapping = true;
		mIsScanned = false;
		smallR = 0.15;
		bigR = 0.25;
		mIsRightEditing = false;
		mIsLeftEditing = false;
		//Horizon Threshold
		mBodyJumpHorizonThreshold = 0.2;//0.03?
		//Vectical Threshold
		mBodyJumpVecticalThreshold = 0.2;//0.03?

	}
} SystemStatueStruct;

#endif