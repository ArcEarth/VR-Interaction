#include "stdafx.h"
#include "KinectFilter.h"

using namespace std;
namespace Filters
{
	DirectX::Vector3 Convert(const Vector4 &v){
		return DirectX::Vector3(v.x,v.y,v.z);
	}

	DirectX::Vector3 Vector3DynamicFilter::Apply(DirectX::Vector3 v)
	{
		using DirectX::XM_2PI;

		// special case if first time being used
		if (mFirstTime) 
		{
			mPrevValue = v;
			mLastPositionForVelocity = v;
			mFirstTime = false;
		}

		float updateFrequency = *mUpdateFrequency;

		// first get an estimate of velocity (with filter)
		DirectX::Vector3 mPositionForVelocity = mVelocityFilter.Apply(v);
		DirectX::Vector3 vel = (mPositionForVelocity - mLastPositionForVelocity) * updateFrequency;
		mLastPositionForVelocity = mPositionForVelocity;
//		vel = Abs(vel);

		// interpolate between frequencies depending on velocity
		float t = vel.Length();
//		DirectX::Vector3 t;

		t = (t - mVelocityLow) / (mVelocityHigh - mVelocityLow);
		
//		t = t / mVelocityHigh;   

		t = max(t, 0.0f);
		t = max(t, 1.0f);

//		float ft=max(t.x,t.y,t.z);
//		if (ft>1) 
//		{ 
////			mVelocityHigh = mVelocityHigh*ft*20; 
//			if (ft>1.2) t=DirectX::Vector3(0.05,0.05,0.05);
//			else t = DirectX::Vector3(0.25,0.25,0.25);
//		}
//		else
//		{
////			mVelocityHigh = max(mVelocityHigh-mVelocityLow*0.25,mVelocityLow);
//		}

		float cutoff;

		cutoff = (mCutoffFrequencyHigh * t) + (mCutoffFrequency * (1 - t));


		DirectX::Vector3 Te(1.0f / updateFrequency, 1.0f / updateFrequency, 1.0f / updateFrequency);		// the sampling period (in seconds)
		DirectX::Vector3 Tau(1.0f / (XM_2PI*cutoff), 1.0f / (XM_2PI*cutoff), 1.0f / (XM_2PI*cutoff));	// a time constant calculated from the cut-off frequency

		DirectX::Vector3 filteredValue;
		filteredValue.x = (v.x + (Tau.x / Te.x) * mPrevValue.x) * (1.0f / (1.0f + Tau.x / Te.x));
		filteredValue.y = (v.y + (Tau.y / Te.y) * mPrevValue.y) * (1.0f / (1.0f + Tau.y / Te.y));
		filteredValue.z = (v.z + (Tau.z / Te.z) * mPrevValue.z) * (1.0f / (1.0f + Tau.z / Te.z));

		mPrevValue = filteredValue;

		//cout << cutoff << "   " << v << "    " << filteredValue << endl;

		return filteredValue;
	}

	void SkeletonFilter::Setup(){
		for (int i = 0; i < NUI_SKELETON_COUNT; i++)
		{
			for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
			{
				JointFilter[i][j].SetUpdateFrequency(Fe);
				JointFilter[i][j].SetCutoffFrequencyLow(FcLow);
				JointFilter[i][j].SetCutoffFrequencyHigh(FcTracked);
				JointFilter[i][j].SetVelocityLow(VcLow);
				JointFilter[i][j].SetVelocityHigh(VcHigh);
				VFreqCutH[i][j] = FcTracked;
			}
		}
	}

	SkeletonFilter::SkeletonFilter (float *UpdateFrequency,float CutoffFrequencyLow,float CutoffFrequencyTracked,float CutoffFrequencyInferred,float CutoffFrequencyIncreasement,float VelocityLow,float VelocityHigh)
	{
		VarFPS=true;
		Fe=UpdateFrequency;
		FcLow=CutoffFrequencyLow;
		FcTracked=CutoffFrequencyTracked;
		FcInferred=CutoffFrequencyInferred;
		FcInc=CutoffFrequencyIncreasement;
		VcLow=VelocityLow;
		VcHigh=VelocityHigh;
		Setup();
	}

	SkeletonFilter::SkeletonFilter (float UpdateFrequency,float CutoffFrequencyLow,float CutoffFrequencyTracked,float CutoffFrequencyInferred,float CutoffFrequencyIncreasement,float VelocityLow,float VelocityHigh)
	{
		VarFPS=false;
		Fe=new float(UpdateFrequency);
		FcLow=CutoffFrequencyLow;
		FcTracked=CutoffFrequencyTracked;
		FcInferred=CutoffFrequencyInferred;
		FcInc=CutoffFrequencyIncreasement;
		VcLow=VelocityLow;
		VcHigh=VelocityHigh;
		Setup();
	}

	SkeletonFilter::SkeletonFilter(float *UpdateFrequency){
		VarFPS=true;
		Fe = UpdateFrequency;
	}

	SkeletonFilter::SkeletonFilter(float UpdateFrequency){
		VarFPS=false;
		Fe = new float(UpdateFrequency);
	}

	//Using the default paramenters for the filter, frame rate is set to 30 as a const
	SkeletonFilter::SkeletonFilter(){
		VarFPS=false;
		Fe=new float(30);
		FcLow=0.01f;
		VcLow=0.0f;
		VcHigh=3.0f;
		FcTracked=3.0f;
		FcInferred=1.0f;
		FcInc=0.1f;
		Setup();
	}

	SkeletonFilter::~SkeletonFilter(){
		if (!VarFPS) delete Fe;
	}

	void SkeletonFilter::Smooth(NUI_SKELETON_FRAME *SkeletonFrame)
	{
		for (int i = 0; i < NUI_SKELETON_COUNT; i++)
		{
			for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
			{
				if  (SkeletonFrame->SkeletonData[i].eSkeletonPositionTrackingState[j] == NUI_SKELETON_POSITION_TRACKED){
//					VcHigh = VcHigh - VcLow;
//					if (VcHigh < VcLow) VcHigh = VcLow;
//					JointFilter[i][j].SetVelocityHigh(VcHigh);
//					JointFilter[i][j].SetCutoffFrequencyLow(0.1);
					VFreqCutH[i][j]+=FcInc;
					if (VFreqCutH[i][j]>FcTracked) VFreqCutH[i][j]=FcTracked;
					JointFilter[i][j].SetCutoffFrequencyHigh(VFreqCutH[i][j]);
				}
				else
				{
//					VcHigh = VcHigh * 1.2;
//					if (VcHigh >10) VcHigh=10;
//					VcHigh = VcLow*300;
//					JointFilter[i][j].SetVelocityHigh(VcHigh);
//					JointFilter[i][j].SetCutoffFrequencyLow(0.05);
					VFreqCutH[i][j] = FcInferred;
					JointFilter[i][j].SetCutoffFrequencyHigh(FcInferred);
				}
				DirectX::Vector3 val = JointFilter[i][j].Apply(Convert(SkeletonFrame->SkeletonData[i].SkeletonPositions[j]));
				SkeletonFrame->SkeletonData[i].SkeletonPositions[j].x=val.x;
				SkeletonFrame->SkeletonData[i].SkeletonPositions[j].y=val.y;
				SkeletonFrame->SkeletonData[i].SkeletonPositions[j].z=val.z;
			}
		}
	}
	
}