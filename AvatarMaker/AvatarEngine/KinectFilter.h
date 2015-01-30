#pragma once

#ifndef FILTERING

#define FILTERING
#include "MathHelper.h"
//#include "vector3d_nobu.h"
#include <math.h>
#include <NuiApi.h>
//#ifndef BOOL
//#define BOOL bool
//#define TRUE true
//#define FALSE false
//#define NULL 0
//#endif

namespace Filters
{

//////////////////////////////////////////////////////////////////////
//
// Filter 
// 
//////////////////////////////////////////////////////////////////////
// Template class for filtering
//  
template <class X> class Filter
{
public:
	Filter(float* mUpdateFrequency);
	Filter() { };
	virtual ~Filter(void);

	void Clear() { mFirstTime = true; };
	void SetUpdateFrequency(float* updateFrequency) { mUpdateFrequency = updateFrequency; };


protected:

	float* mUpdateFrequency;

	X mPrevValue;
	bool mFirstTime;

};

//
//////////////////////////////////////////////////////////////////////
// //		mUpdateFrequency (Hz) 
template <class X> Filter<X>::Filter(float* updateFrequency)
{
	mUpdateFrequency = updateFrequency;
	mFirstTime = true;
}

//
//////////////////////////////////////////////////////////////////////
// 
template <class X> Filter<X>::~Filter(void)
{
}

//////////////////////////////////////////////////////////////////////
//
// Low Pass Filter 
// 
//////////////////////////////////////////////////////////////////////
// set the following before using: 
//		mCutoffFrequency
//
template <class X> class LowPassFilter : public Filter<X>
{
public:
	LowPassFilter(float* mUpdateFrequency) : Filter<X>(mUpdateFrequency) { };
	LowPassFilter() : Filter<X>() { };
	X Apply(X NewValue);
	void SetCutoffFrequency(float f) { mCutoffFrequency = f; };
	float GetCutoffFrequency(void) { return mCutoffFrequency; };

protected:
	float mCutoffFrequency;

};

//
//////////////////////////////////////////////////////////////////////
// 
template <class X>
X LowPassFilter<X>::Apply(X newValue)
{
	/*

	Let's say Pnf the filtered position, Pn the non filtered position and Pn-1f the previous filtered position, 
	Te the sampling period (in second) and tau a time constant calculated from the cut-off frequency fc.

	tau = 1 / (2 * pi * fc)
	Pnf = ( Pn + tau/Te * Pn-1f ) * 1/(1+ tau/Te)
	 
	Attention: tau >= 10 * Te
	*/

	if (mFirstTime) 
	{
		mPrevValue = newValue;
		mFirstTime = false;
	}

	float updateFrequency = *mUpdateFrequency;

	float Te = (float)(1.0f / updateFrequency);		// the sampling period (in seconds)
	float Tau = (float)(1.0f / (DirectX::XM_2PI*mCutoffFrequency));	// a time constant calculated from the cut-off frequency

	X filteredValue = (newValue + (Tau/Te) * mPrevValue) * (1.0f / (1.0f + Tau / Te));

//	Vecteur3D filteredValue = ((newValue - PositionPrev) * (1.0/Te) +  mPrevValue * (Tau/Te) ) * (1.0 / (1.0 + Tau / Te));
	mPrevValue = filteredValue;
//	PositionPrev = Position;
//	if (fabs(Velocity.x) < SpeedDeadband) newValue.x = 0.0;
//	if (fabs(Velocity.y) < SpeedDeadband) newValue.y = 0.0;
////	if (fabs(Velocity.z) < speedDeadband) Velocity.z = 0.0;
	return filteredValue;
//}

}


//////////////////////////////////////////////////////////////////////
//
// Low Pass Dynamic (or Adjustable) Filter 
// 
//////////////////////////////////////////////////////////////////////
// adjusts the cutoff filter depending on the velocity
// set the following before using: 
//		mUpdateFrequency (Hz) 
//		mCutoffFrequency the lower cuttoff frequency
//		mCutoffFrequencyHigh
//		mVelocityLow (mm/s) the speed at which mCutoffFrequencyLow is reached 
//		mVelocityHigh (mm/s) the speed at which mCutoffFrequencyHigh is reached

template <class X> class LowPassDynamicFilter : public LowPassFilter<X>
{
public:
	LowPassDynamicFilter(float* updateFrequency) : LowPassFilter<X>(updateFrequency), mVelocityFilter(updateFrequency) {  };
	LowPassDynamicFilter() {  };
	X Apply(X NewValue);
	void SetUpdateFrequency(float* updateFrequency) { mUpdateFrequency = updateFrequency; mVelocityFilter.SetUpdateFrequency(updateFrequency);};

	void SetCutoffFrequencyLow(float f) { mCutoffFrequency = f; SetCutoffFrequencyVelocity(); };
	void SetCutoffFrequencyHigh(float f) { mCutoffFrequencyHigh = f; SetCutoffFrequencyVelocity(); };
	void SetVelocityLow(float f) { mVelocityLow = f; };
	void SetVelocityHigh(float f) { mVelocityHigh = f; };

protected:

	// cutoff freq for velocity
	// equal to mCutoffFrequency  + 0.75 * (mCutoffFrequencyHigh - mCutoffFrequency)
	void SetCutoffFrequencyVelocity() 
	{ 
		mVelocityFilter.SetCutoffFrequency((float)(mCutoffFrequency  + 0.75 * (mCutoffFrequencyHigh - mCutoffFrequency)));
	}
	LowPassFilter<X> mVelocityFilter;
	X mLastPositionForVelocity;

	float mCutoffFrequencyHigh, mVelocityLow, mVelocityHigh;


};

//
//////////////////////////////////////////////////////////////////////
// 
//template <class X>
//X LowPassDynamicFilter<X>::Apply(X x)
//{
//	return x;
//}

template <class X>
X LowPassDynamicFilter<X>::Apply(X x)
{

	// special case if first time being used
	if (mFirstTime) 
	{
		mPrevValue = x;
		mLastPositionForVelocity = x;
		mFirstTime = false;
	}


	float updateFrequency = *mUpdateFrequency;

	// first get an estimate of velocity (with filter)
	X mPositionForVelocity = mVelocityFilter.Apply(x);
	X vel = (mPositionForVelocity - mLastPositionForVelocity) * updateFrequency;
	mLastPositionForVelocity = mPositionForVelocity;
	vel = fabs(vel);

	// interpolate between frequencies depending on velocity
	double t;

	// Modify by Yupeng
	t = (vel - mVelocityLow) / (mVelocityHigh - mVelocityLow);   
//	t = vel / mVelocityHigh;   

	t = max(t, 0.0);
	t = min(t, 1.0);

	X cutoff;

	cutoff = (mCutoffFrequencyHigh * t) + (mCutoffFrequency * (1 - t));


	X Te(1.0 / updateFrequency);		// the sampling period (in seconds)
	X Tau(1.0 / (2*3.14159265*cutoff));	// a time constant calculated from the cut-off frequency

	X filteredValue = (x + (Tau / Te) * mPrevValue) * (1.0 / (1.0 + Tau / Te));

	mPrevValue = filteredValue;
	return filteredValue;

}

//
//////////////////////////////////////////////////////////////////////
// 


class Vector3DynamicFilter : public LowPassDynamicFilter<DirectX::Vector3>
{
public:
	DirectX::Vector3 Apply(DirectX::Vector3 NewValue);
private:
//	LowPassFilter<float> mVelocityFilter;
};

class SkeletonFilter
{
public:
	SkeletonFilter (float *UpdateFrequency,float CutoffFrequencyLow,float CutoffFrequencyTracked,float CutoffFrequencyInferred,float CutoffFrequencyIncreasement,float VelocityLow,float VelocityHigh);
	SkeletonFilter (float UpdateFrequency,float CutoffFrequencyLow,float CutoffFrequencyTracked,float CutoffFrequencyInferred,float CutoffFrequencyIncreasement,float VelocityLow,float VelocityHigh);
	SkeletonFilter ();
	SkeletonFilter::SkeletonFilter(float *UpdateFrequency);
	SkeletonFilter::SkeletonFilter(float UpdateFrequency);
	~SkeletonFilter();
	void Smooth(NUI_SKELETON_FRAME *SkeletonFrame);
	void Reduce(NUI_SKELETON_FRAME *SkeletonFrame);

protected:
	float *Fe,FcLow,FcTracked,FcInferred,FcInc,VcLow,VcHigh;
	Vector3DynamicFilter JointFilter[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT];
	float	VFreqCutH[NUI_SKELETON_COUNT][NUI_SKELETON_POSITION_COUNT];
	bool VarFPS;

	void Setup();

};

}


//#undef BOOL
//#undef TRUE
//#undef FALSE

#endif