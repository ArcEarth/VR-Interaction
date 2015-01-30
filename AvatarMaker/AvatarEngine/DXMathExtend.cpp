#include "stdafx.h"
#include "DXMathExtend.h"
namespace DirectX{

	float FindDistancePointToPath(const Vector3 &p,const LinePath &path){
		if (path.size()==1) 
		{
			Vector3 v(p-path[0]);
			return v.Length();
		}
		float dis,mindis=10000000.0f;
		for (unsigned int i = 1; i < path.size(); i++)
		{
			dis = FindDistancePointToSegment(p,path[i-1],path[i]);
			if (dis<mindis) 
				mindis =dis;
		}
		return mindis;
	}

	float FindDistancePointToCylinder(const Vector3 &p,const std::vector<Vector3> &CylinderCentrePath, std::vector<float> &CylinderRadius){
		return 0.0f;
	}

	inline bool NearEqual(float A , float B , float epsilon = XM_EPSILON)
	{
		float delta = abs(A-B);
		return delta < epsilon;
	}

	//	    * (Return Point) 
	//	  r |
	//p1.---*---.p2
	//	 \ L|  /
	//	 R\ | /
	//	   \|/
	//	    * (SphereCentre)
	 Vector3 FindCircumSphereCentre_TwoPoint_ApicalAngle_ProjectDirection(const Vector3 & p1,const Vector3 & p2, const Vector3 &projectDirction, float TopAngle)
	{
		Vector3 CentreP = (p1 +p2 )/2;
		Vector3 v = p2-p1 ,direction = projectDirction;
		direction = direction ^ v;
		direction = v ^ direction;
		direction.Normalize();
		float r = (v.Length())/2 /*, L , R*/;
		float L = r / tanf(TopAngle / 2);
//		R = sqrtf(L*L+r*r);

		return CentreP + L*direction;
//		return SphereCentre - R*direction;
	}


	//D3DXMATRIX GetRotateMatrixFromVectorToVector(const Vector3 &VecForm,const Vector3 &VecTo){
	//	Vector3 RotateAxias;
	//	float Arc,Dot;
	//	D3DXVec3Cross(&RotateAxias,&VecForm,&VecTo);
	//	D3DXVec3Normalize(&RotateAxias,&RotateAxias);
	//	Dot = D3DXVec3Dot(&VecForm,&VecTo)/D3DXVec3Length(&VecForm)/D3DXVec3Length(&VecTo);
	//	Arc = acosf(Dot);
	//	D3DXMATRIX RotateMatrix;
	//	// /D3DX_PI*180
	//	D3DXMatrixRotationAxis(&RotateMatrix,&RotateAxias,Arc);
	//	return RotateMatrix;
	//}


	float FindDistancePointToSurface(const Vector3 &p,const std::vector<Vector3> &path){
		return 0;
	}

	//BindingBox::BindingBox() { 
	//	 X_min=Y_min=Z_min=1e12f;
	//	 X_max=Y_max=Z_max=-1e12f;
	//}

	//BindingBox::BindingBox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max){
	//	X_min = x_min;
	//	Y_min = y_min;
	//	Z_min = z_min;
	//	X_max = x_max;
	//	Y_max = y_max;
	//	Z_max = z_max;
	//}


	//void BindingBox::Expand(const Vector3 &Point){
	//	if (Point.x > X_max) X_max = Point.x;
	//	if (Point.x <X_min) X_min = Point.x;
	//	if (Point.y > Y_max) Y_max = Point.y;
	//	if (Point.y <Y_min) Y_min = Point.y;
	//	if (Point.z > Z_max) Z_max = Point.z;
	//	if (Point.z <Z_min) Z_min = Point.z;
	//}

	//void BindingBox::Expand(const Vector4 &Sphere){
	//	if (Sphere.x + Sphere.w > X_max) X_max = Sphere.x + Sphere.w ;
	//	if (Sphere.x - Sphere.w < X_min) X_min = Sphere.x - Sphere.w ;
	//	if (Sphere.y + Sphere.w > Y_max) Y_max = Sphere.y + Sphere.w ;
	//	if (Sphere.y - Sphere.w < Y_min) Y_min = Sphere.y - Sphere.w ;
	//	if (Sphere.z + Sphere.w > Z_max) Z_max = Sphere.z + Sphere.w ;
	//	if (Sphere.z - Sphere.w < Z_min) Z_min = Sphere.z - Sphere.w ;
	//}

	//const bool BindingBox::IsPointInside(const Vector3 &Point) const{
	//	return (Point.x > X_min && Point.x < X_max && 
	//			Point.y > Y_min && Point.y < Y_max && 
	//			Point.z > Z_min && Point.z < Z_max);
	//}


	//void BindingBox::Clear(){
	//	 X_min=X_max=Y_min=Y_max=Z_min=Z_max=0.0f;
	//}
	SpaceCurveSampler::SpaceCurveSampler(const std::vector<Vector3>& Trajectory)
	{
		for (auto& point : Trajectory)
		{
			this->push_back(point);
		}
	}


	std::unique_ptr<std::vector<Vector3>> SpaceCurveSampler::FixCountSampling( unsigned int SampleSegmentCount , bool Smooth/* = true*/) const
	{
		if (Anchors.size() <= 1) {
			return nullptr;
		}
		float Interval = length() / SampleSegmentCount;
		// Allocate C+1 size array for storage the result
		std::unique_ptr<std::vector<Vector3>> pSample(new std::vector<Vector3>(SampleSegmentCount+1));
		Vector3 ptr;
		float r = 0.0f;
		unsigned int k = 1;

		for (unsigned int i = 0; i < SampleSegmentCount+1; i++,r+=Interval)
		{
			while ((k<Anchors.size()-1) && (Anchors[k].w < r)) k++;
			float t = r - Anchors[k-1].w;
			float f = Anchors[k].w - Anchors[k-1].w;
			if (abs(f)<1.192092896e-7f) 
			{
				XMVECTOR v2 = XMLoadFloat4A(&Anchors[k]);
				XMStoreFloat3(&(*pSample)[i],v2);
			} else
			{
				XMVECTOR v0 = XMLoadFloat4A(&Anchors[k-1]);
				XMVECTOR v1 = XMLoadFloat4A(&Anchors[k]);
				XMVECTOR v2 = XMVectorLerp(v0,v1,t);
				XMStoreFloat3(&(*pSample)[i],v2);
			}
		}

		if (Smooth)
			LaplaianSmoothing(*pSample,16);

		return pSample;
	}

	std::unique_ptr<std::vector<Vector3>> SpaceCurveSampler::FixCountSampling2(unsigned int SampleSegmentCount) const
	{
		auto pTrajectory = FixCountSampling(SampleSegmentCount*15,true);
		SpaceCurveSampler smoothSampler(*pTrajectory);
		return smoothSampler.FixCountSampling(SampleSegmentCount,false);
	}


	std::unique_ptr<std::vector<Vector3>> SpaceCurveSampler::FixIntervalSampling( float Interval ) const
	{
		assert(Interval!=0.0f);
		float r = length();
		unsigned int Count = (int) (r / Interval) + 1;
		auto pTrajectory = FixCountSampling(Count*15,true);
		SpaceCurveSampler smoothSampler(*pTrajectory);
		r = smoothSampler.length();
		Count = (int) (r / Interval) + 1;
		return smoothSampler.FixCountSampling(Count,false);
	}

	void SpaceCurveSampler::push_back(const Vector3& p)
	{
		const float* ptr = reinterpret_cast<const float*>(&p);
		XMFLOAT4A content(ptr);
		if (Anchors.empty()) {
			content.w = .0f;
			Anchors.push_back(content);
		} else {
			XMVECTOR vtr = XMLoadFloat4A(&content);
			XMVECTOR btr = XMLoadFloat4A(&(Anchors.back()));
			float len = XMVectorGetW(btr + XMVector3Length(vtr - btr));
			if (abs(len- Anchors.back().w) < XM_EPSILON*8)
				return;
			content.w = len;
		}
		Anchors.push_back(content);
	}

	DirectX::XMVECTOR SpaceCurveSampler::extract( float t ) const
	{
		unsigned int a = 0, b = Anchors.size()-1;
		t = t * length();
		while (b - a >1)
		{
			unsigned int k =(a+b)/2;
			if (Anchors[k].w > t) b = k;
			else a = k;
		}
		XMVECTOR v0 = XMLoadFloat4A(&Anchors[a]);
		XMVECTOR v1 = XMLoadFloat4A(&Anchors[b]);
		float rt = (t - Anchors[a].w) / (Anchors[b].w - Anchors[a].w);
		XMVECTOR v2 = XMVectorLerp(v0,v1,rt);
		return v2;
	}

	void SpaceCurveSampler::LaplaianSmoothing( std::vector<Vector3>& Curve , unsigned IterationTimes /*= 1*/ )
	{
		unsigned int N = Curve.size();
		std::vector<Vector3> Cache(N);
		Vector3* BUFF[2] = {&Curve[0],&Cache[0]};
		unsigned int src = 0 , dst = 1;
		for (unsigned int k = 0; k < IterationTimes; k++)
		{
			BUFF[dst][0] = BUFF[src][0];
			BUFF[dst][N-1] = BUFF[src][N-1];
			for (unsigned int i = 1; i < N-1 ; i++)
			{
				BUFF[dst][i] = (BUFF[src][i-1] + BUFF[src][i+1])*0.5f;
			}
			dst = !dst;
			src = !src;
		}
		if (dst == 0)
		{
			for (unsigned int i = 0; i < N; i++)
			{
				BUFF[0][i] = BUFF[1][i];
			}
		}
	}

}