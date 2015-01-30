#pragma once

#include "MathHelper.h"
#include <DirectXCollision.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <memory>

namespace std
{
	inline std::ostream& operator << (std::ostream& lhs, const DirectX::Vector3& rhs)
	{
		lhs << '(' << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.x
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.y
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.z << ')';
		return lhs;
	};

	inline std::ostream& operator << (std::ostream& lhs, const DirectX::XMFLOAT4& rhs)
	{
		lhs << '(' << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.x
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.y
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.z
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.w << ')';
		return lhs;
	};

	inline std::ostream& operator << (std::ostream& lhs, const DirectX::Quaternion& rhs)
	{
		float theta = std::acosf(rhs.w) * 2 / DirectX::XM_PI;
		DirectX::Vector3 axis(rhs);
		axis = DirectX::Vector3::Normalize(axis);
		lhs << '(' << axis
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << theta << "*Pi)";
		return lhs;
	};
}


namespace DirectX{
	typedef std::pair<Vector3,Vector3> LineSegement;
	typedef std::vector<Vector3> LinePath;
	static const float g_INFINITY = 1e12f;

	//template <typename _Key,typename _T>
	//class LinearSampler<_Key,_T>
	//{
	//public:
	//	typedef _Key KeyType;
	//	typedef _T ValueType;
	//protected:
	//	std::map<KeyType,ValueType> Crontrols;
	//};

	class SpaceCurveSampler
	{
	public:
		SpaceCurveSampler() {}
		~SpaceCurveSampler() {}
		explicit SpaceCurveSampler(const std::vector<Vector3>& Trajectory);

		std::size_t size() const {return Anchors.size();}
		bool empty() const {return Anchors.empty();}
		float length() const 
		{
			if (Anchors.empty()) return 0.0f; 
			return Anchors.back().w;
		}
		void clear() {Anchors.clear();}
		void push_back(const Vector3& p);
		XMVECTOR back() const {return XMLoadFloat4A(&Anchors.back());}
//		void push_front(const Vector3& p);

		// This O(LogN) level operation
		// perform a binary search in anchors 
		// returns a 4D vector where xyz represent the Position 
		// w is the length from start to point;
		XMVECTOR extract(float t) const;
		XMVECTOR operator[](float t) const {return extract(t);}

		std::unique_ptr<std::vector<Vector3>> FixIntervalSampling(float Interval) const;
		std::unique_ptr<std::vector<Vector3>> FixCountSampling(unsigned int SampleSegmentCount , bool Smooth = true) const;
		std::unique_ptr<std::vector<Vector3>> FixCountSampling2(unsigned int SampleSegmentCount) const;

		static void LaplaianSmoothing(std::vector<Vector3>& Curve , unsigned IterationTimes = 2);

		const std::vector<XMFLOAT4A,DirectX::AlignedAllocator<XMFLOAT4A>>& ControlPoints() const
		{
			return Anchors;
		}

	private:
		// The data we stored is actually aligned on 16-byte boundary , so , use it as a XMFLOAT4A
		std::vector<XMFLOAT4A,DirectX::AlignedAllocator<XMFLOAT4A>> Anchors;
	};

	float DistanceSegmentToSegment(const LineSegement &S1 , const LineSegement &S2);
	float DistanceSegmentToSegment(FXMVECTOR S0P0,FXMVECTOR S0P1, FXMVECTOR S1P0, GXMVECTOR S1P1);
	//inline float Length(const LineSegement& Ls){
	//	return Distance(Ls.first,Ls.second);
	//}

	//Some supporting method

	inline float FindDistancePointToSegment(FXMVECTOR p,FXMVECTOR s0,FXMVECTOR s1)
	{
		XMVECTOR s=s1-s0;
		XMVECTOR v=p-s0;
		float Ps = Vector3::Dot(v,s);
			//p-s0 is the shortest distance
		if (Ps<=0.0f) 
			return Vector3::Length(v);

		float Ds = Vector3::LengthSq(s);
			//p-s1 is the shortest distance
		if (Ps>Ds) 	
			return Vector3::Length(p-s1);
		//find the projection point on line segment U
		return Vector3::Length(v - s * (Ps/Ds));
	}

	// Takes a space point and space line segment , return the projection point on the line segment
	//  A0  |		A1		  |
	//      |s0-------------s1|
	//      |				  |		A2
	// where p in area of A0 returns s0 , area A2 returns s1 , point in A1 returns the really projection point 
	inline XMVECTOR Projection(FXMVECTOR p,FXMVECTOR s0,FXMVECTOR s1)
	{
		XMVECTOR s=s1-s0;
		XMVECTOR v=p-s0;
		XMVECTOR Ps = XMVector3Dot(v,s);

		//p-s0 is the shortest distance
		//if (Ps<=0.0f) 
		//	return s0;
		if (XMVector4LessOrEqual(Ps,g_XMZero))
			return s0;
		XMVECTOR Ds = XMVector3LengthSq(s);
		//p-s1 is the shortest distance
		//if (Ps>Ds) 	
		//	return s1;
		if (XMVector4Less(Ds,Ps))
			return s1;
		//find the projection point on line segment U
		return (s * (Ps/Ds)) + s0;
	}

	inline XMVECTOR Projection(FXMVECTOR p , const std::vector<Vector3> &path)
	{
		const auto N = path.size();
		XMVECTOR vBegin = path.front();
		XMVECTOR vEnd = path[1];
		XMVECTOR vMinProj = Projection(p,vBegin,vEnd);
		XMVECTOR vMinDis = XMVector3LengthSq(p-vMinProj);
		vBegin = vEnd;

		for (size_t i = 2; i < N-1; i++)
		{
			vEnd = path[i];
			XMVECTOR vProj = Projection(p,vBegin,vEnd);
			XMVECTOR vDis = XMVector3LengthSq(p-vProj);
			if (XMVector4LessOrEqual(vDis,vMinDis))
			{
				vMinDis = vDis;
				vMinProj = vProj;
			}
			vBegin = vEnd;
		}

		return vMinProj;
	}


	float FindDistancePointToPath(const Vector3 &p,const std::vector<Vector3> &path);

	float FindDistancePointToCylinder(const Vector3 &p,const std::vector<Vector3> &CylinderCentrePath, std::vector<float> &CylinderRadius);

	Vector3 FindCircumSphereCentre_TwoPoint_ApicalAngle_ProjectDirection(const Vector3 & p1,const Vector3 & p2, const Vector3 &projectDirction, float TopAngle);

//	D3DXMATRIX GetRotateMatrixFromVectorToVector(const Vector3 &VecForm,const Vector3 &VecTo);

	inline XMVECTOR Slerp(FXMVECTOR SphereCentre, FXMVECTOR p1,FXMVECTOR p2, float InterplotRatio)
	{
		XMVECTOR v0 = p1 - SphereCentre , v1 = p2 - SphereCentre;

		XMVECTOR RotateAxias = Vector3::Cross(v0,v1);
		RotateAxias = Vector3::Normalize(RotateAxias);
		float Arc,Dot;
		Dot = XMVectorGetX(XMVector3Dot(v0,v1)/XMVector3Length(v0)/XMVector3Length(v1));
		Arc = XMScalarACos(Dot);
		// Store Rotate Quaternion into RotateAxias
		RotateAxias = XMQuaternionRotationNormal(RotateAxias,Arc*InterplotRatio);
		// Make the right direction
		XMVECTOR SlerpPoint = XMVector3Rotate(v0,RotateAxias);
		SlerpPoint = Vector3::Normalize(SlerpPoint);
		// Make the right length
		SlerpPoint *= XMVectorLerp(XMVector3Length(v0),XMVector3Length(v1),InterplotRatio);
		return SlerpPoint+SphereCentre;
	}

	//struct BindingBox
	//{
	//	float X_min,X_max,Y_min,Y_max,Z_min,Z_max;
	//	BindingBox();
	//	BindingBox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
	//	void Expand(const Vector3 &Point);
	//	void Expand(const Vector4 &Sphere);
	//	const bool IsPointInside(const Vector3 &Point) const;
	//	void Clear();
	//};
}
