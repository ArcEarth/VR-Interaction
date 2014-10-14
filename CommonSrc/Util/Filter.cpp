#include "Filter.h"
//
//using namespace std;
//namespace Filters
//{
//	DirectX::Vector3 Convert(const Vector4 &v){
//		return DirectX::Vector3(v.x, v.y, v.z);
//	}
//
//	DirectX::Vector3 Vector3DynamicFilter::Apply(DirectX::Vector3 v)
//	{
//		using DirectX::XM_2PI;
//
//		// special case if first time being used
//		if (mFirstTime)
//		{
//			mPrevValue = v;
//			mLastPositionForVelocity = v;
//			mFirstTime = false;
//		}
//
//		float updateFrequency = *mUpdateFrequency;
//
//		// first get an estimate of velocity (with filter)
//		DirectX::Vector3 mPositionForVelocity = mVelocityFilter.Apply(v);
//		DirectX::Vector3 vel = (mPositionForVelocity - mLastPositionForVelocity) * updateFrequency;
//		mLastPositionForVelocity = mPositionForVelocity;
//		//		vel = Abs(vel);
//
//		// interpolate between frequencies depending on velocity
//		float t = vel.Length();
//		//		DirectX::Vector3 t;
//
//		t = (t - mVelocityLow) / (mVelocityHigh - mVelocityLow);
//
//		//		t = t / mVelocityHigh;   
//
//		t = max(t, 0.0f);
//		t = max(t, 1.0f);
//
//		//		float ft=max(t.x,t.y,t.z);
//		//		if (ft>1) 
//		//		{ 
//		////			mVelocityHigh = mVelocityHigh*ft*20; 
//		//			if (ft>1.2) t=DirectX::Vector3(0.05,0.05,0.05);
//		//			else t = DirectX::Vector3(0.25,0.25,0.25);
//		//		}
//		//		else
//		//		{
//		////			mVelocityHigh = max(mVelocityHigh-mVelocityLow*0.25,mVelocityLow);
//		//		}
//
//		float cutoff;
//
//		cutoff = (mCutoffFrequencyHigh * t) + (mCutoffFrequency * (1 - t));
//
//
//		DirectX::Vector3 Te(1.0f / updateFrequency, 1.0f / updateFrequency, 1.0f / updateFrequency);		// the sampling period (in seconds)
//		DirectX::Vector3 Tau(1.0f / (XM_2PI*cutoff), 1.0f / (XM_2PI*cutoff), 1.0f / (XM_2PI*cutoff));	// a time constant calculated from the cut-off frequency
//
//		DirectX::Vector3 filteredValue;
//		filteredValue.x = (v.x + (Tau.x / Te.x) * mPrevValue.x) * (1.0f / (1.0f + Tau.x / Te.x));
//		filteredValue.y = (v.y + (Tau.y / Te.y) * mPrevValue.y) * (1.0f / (1.0f + Tau.y / Te.y));
//		filteredValue.z = (v.z + (Tau.z / Te.z) * mPrevValue.z) * (1.0f / (1.0f + Tau.z / Te.z));
//
//		mPrevValue = filteredValue;
//
//		//cout << cutoff << "   " << v << "    " << filteredValue << endl;
//
//		return filteredValue;
//	}
//}