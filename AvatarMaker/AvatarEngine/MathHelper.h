#ifndef DX_MATH_HELPER_H
#define DX_MATH_HELPER_H
#pragma once
#include <DirectXMath.h>
#include <SimpleMath.h>
#include <smmintrin.h>
#include "AlignedNew.h"
#include <type_traits>
#include <boost\operators.hpp>

namespace DirectX{

	struct XMDUALVECTOR;
	// Calling convetion
	typedef const XMDUALVECTOR& CXMDUALVECTOR;

#if (defined(_M_IX86) || defined(_M_AMD64) || defined(_M_ARM)) && defined(_XM_NO_INTRINSICS_)
	struct XMDUALVECTOR
#else
	__declspec(align(16)) struct XMDUALVECTOR
		: boost::additive<DirectX::CXMDUALVECTOR>
		, boost::multiplicative<DirectX::CXMDUALVECTOR,float>
#endif
	{
#ifdef _XM_NO_INTRINSICS_
		union
		{
			XMVECTOR r[2];
			struct
			{
				float _11, _12, _13, _14;
				float _21, _22, _23, _24;
			};
			float m[2][4];
		};
#else
		XMVECTOR r[2];
#endif

		XMDUALVECTOR() {}
		XMDUALVECTOR(FXMVECTOR V0, FXMVECTOR V1) { r[0] = V0; r[1] = V1;}
		XMDUALVECTOR(float m00, float m01, float m02, float m03,
			float m10, float m11, float m12, float m13);
		explicit XMDUALVECTOR(_In_reads_(8) const float *pArray);

		XMDUALVECTOR&   operator= (const XMDUALVECTOR& DV) { r[0] = DV.r[0]; r[1] = DV.r[1]; return *this; }

		XMDUALVECTOR    operator+ () const { return *this; }
		XMDUALVECTOR    operator- () const
		{
			XMDUALVECTOR dvResult(r[0],-r[1]);
			return dvResult;
		}


		XMDUALVECTOR&   operator+= (CXMDUALVECTOR M)
		{
			r[0] += M.r[0];
			r[1] += M.r[1];
			return *this;
		}

		XMDUALVECTOR&   operator-= (CXMDUALVECTOR M)
		{
			r[0] -= M.r[0];
			r[1] -= M.r[1];
			return *this;
		}

		//XMDUALVECTOR&   operator*= (CXMDUALVECTOR M);
		XMDUALVECTOR&   operator*= (float S)
		{
			r[0] *= S;
			r[1] *= S;
			return *this;
		}

		XMDUALVECTOR&   operator/= (float S)
		{
			r[0] /= S;
			r[1] /= S;
			return *this;
		}



		//friend XMDUALVECTOR operator* (float S, CXMMATRIX M);
	};

	XMVECTOR XMVector3Displacement(FXMVECTOR V, FXMVECTOR RotationQuaternion , FXMVECTOR TranslationQuaternion);

	inline XMDUALVECTOR XMDualQuaternionTranslation(FXMVECTOR T)
	{
		static const XMVECTORF32 Control = {0.5f,0.5f,0.5f,.0f};
		XMVECTOR Qe = XMVectorMultiply(T,Control);
		XMVECTOR Qr = XMQuaternionIdentity();
		XMDUALVECTOR dqRes;
		dqRes.r[0] = Qr;
		dqRes.r[1] = Qe;
		return dqRes;
	}

	inline XMDUALVECTOR XMDualQuaternionRotation(FXMVECTOR Q)
	{
		XMDUALVECTOR dqRes;
		dqRes.r[0] = Q;
		dqRes.r[1] = XMVectorZero();
	}

	inline XMDUALVECTOR XMDualQuaternionRigidTransform(FXMVECTOR RotationOrigin , FXMVECTOR RotationQuaternion , FXMVECTOR Translation)
	{
		static const XMVECTORF32 Control = {0.5f,0.5f,0.5f,.0f};

		XMVECTOR Qr = RotationQuaternion;
		XMVECTOR Qe = RotationOrigin + Translation;
		Qe = XMVectorMultiply(Qe,Control);
		XMVECTOR Qo = XMVectorMultiply(RotationOrigin,Control);
		Qo = XMVectorNegate(Qo);

		Qe = XMQuaternionMultiply(Qr,Qe);
		Qo = XMQuaternionMultiply(Qo,Qr);
		Qe += Qo;

		XMDUALVECTOR dqRes;
		dqRes.r[0] = Qr;
		dqRes.r[1] = Qe;
		return dqRes;
	}



	inline XMDUALVECTOR XMDualQuaternionRotationTranslation(FXMVECTOR Q,FXMVECTOR T)
	{
		// non-dual part (just copy q0):
		// dual part:
		static const XMVECTORF32 Control = {0.5f,0.5f,0.5f,.0f};
		XMVECTOR Qe = XMVectorMultiply(T,Control);
		Qe = XMQuaternionMultiply(Q,Qe);
		XMDUALVECTOR dqRes;
		dqRes.r[0] = Q;
		dqRes.r[1] = Qe;

		return dqRes;
		//static const XMVECTORF32 ControlX = {0.5f,0.5f,-0.5f,.0f};
		//static const XMVECTORF32 ControlY = {-0.5f,0.5f,0.5f,.0f};
		//static const XMVECTORF32 ControlZ = {0.5f,-0.5f,0.5f,.0f};
		//static const XMVECTORF32 ControlW = {-0.5f,-0.5f,-0.5f,.0f};
		//XMVECTOR vDualPart;
		//XMVECTOR Qe = XMVectorMultiply(Q,ControlW);
		//Qe = XMVector3Dot(T,Qe); // real part
		//vDualPart = XMVectorAndInt(Qe,g_XMMaskX);

		//Qe = XMVectorSwizzle<3,2,1,0>(Q);
		//Qe = XMVectorMultiply(Qe,ControlX);
		//Qe = XMVector3Dot(T,Qe);
		//Qe = XMVectorAndInt(Qe,g_XMMaskY);
		//vDualPart = XMVectorOrInt(vDualPart,Qe);

		//Qe = XMVectorSwizzle<2,3,0,1>(Q);
		//Qe = XMVectorMultiply(Qe,ControlY);
		//Qe = XMVector3Dot(T,Qe);
		//Qe = XMVectorAndInt(Qe,g_XMMaskZ);
		//vDualPart = XMVectorOrInt(vDualPart,Qe);

		//Qe = XMVectorSwizzle<1,0,3,2>(Q);
		//Qe = XMVectorMultiply(Qe,ControlZ);
		//Qe = XMVector3Dot(T,Qe);
		//Qe = XMVectorAndInt(Qe,g_XMMaskW);
		//vDualPart = XMVectorOrInt(vDualPart,Qe);
		//XMDUALVECTOR dResult(Q,vDualPart);
		//return dResult;
	};

	inline XMDUALVECTOR XMDualQuaternionNormalize(CXMDUALVECTOR Dq)
	{
		XMVECTOR Length = XMQuaternionLength(Dq.r[0]);
		XMDUALVECTOR dqRes;
		dqRes.r[0] = XMVectorDivide(Dq.r[0],Length);
		dqRes.r[1] = XMVectorDivide(Dq.r[1],Length);
		return dqRes;
	}
	inline XMVECTOR XMDualQuaternionNormalizeEst(CXMDUALVECTOR Dq)
	{
		return XMVector4NormalizeEst(Dq.r[0]);
	}
	inline XMVECTOR XMDualQuaternionLength(CXMDUALVECTOR Dq)
	{
		return XMVector4Length(Dq.r[0]);
	}
	inline XMVECTOR XMDualQuaternionLengthSq(CXMDUALVECTOR Dq)
	{
		return XMVector4LengthSq(Dq.r[0]);
	}
	inline XMVECTOR XMDualQuaternionLengthEst(CXMDUALVECTOR Dq)
	{
		return XMVector4LengthEst(Dq.r[0]);
	}
	inline XMDUALVECTOR XMDualVectorConjugate(CXMDUALVECTOR Dq)
	{
		XMDUALVECTOR dvResult(Dq.r[0],XMVectorNegate(Dq.r[1]));
		return dvResult;
	}
	inline XMDUALVECTOR XMDualQuaternionConjugate(CXMDUALVECTOR Dq)
	{
		XMDUALVECTOR dvResult;
		dvResult.r[0] = XMQuaternionConjugate(Dq.r[0]);
		dvResult.r[1] = XMQuaternionConjugate(Dq.r[1]);
		return dvResult;
	}

	inline XMDUALVECTOR XMDualQuaternionInverse(CXMDUALVECTOR Dq)
	{
		XMDUALVECTOR dvResult = XMDualQuaternionConjugate(Dq);
		dvResult.r[0] = XMQuaternionNormalize(dvResult.r[0]);
	}

	inline XMDUALVECTOR XMDualQuaternionMultipy(CXMDUALVECTOR DQ0,CXMDUALVECTOR DQ1)
	{
		XMDUALVECTOR dvResult;
		dvResult.r[0] = XMQuaternionMultiply(DQ0.r[0],DQ1.r[1]);
		dvResult.r[1] = XMQuaternionMultiply(DQ0.r[0],DQ1.r[1]);
		dvResult.r[1] += XMQuaternionMultiply(DQ0.r[1],DQ1.r[0]);
		return dvResult;
	}

	inline bool XMDualQuaternionDecompose(XMVECTOR* outRotQuat, XMVECTOR* outTrans ,CXMDUALVECTOR Dq)
	{
		static const XMVECTORF32 ControlX = {2.0f,-2.0f,2.0f,-2.0f};
		static const XMVECTORF32 ControlY = {2.0f,2.0f,-2.0f,-2.0f};
		static const XMVECTORF32 ControlZ = {-2.0f,2.0f,2.0f,-2.0f};

		//vT.x = 2.0f*( Qe.x*Q.w - Qe.y*Q.z + Qe.z*Q.y - Qe.w*Q.x);
		//vT.y = 2.0f*( Qe.x*Q.z + Qe.y*Q.w - Qe.z*Q.x - Qe.w*Q.y);
		//vT.z = 2.0f*(-Qe.x*Q.y + Qe.y*Q.x + Qe.z*Q.w - Qe.w*Q.z);
		XMVECTOR vT;
		XMVECTOR Qr = Dq.r[0];
		XMVECTOR Qe = Dq.r[1];
		Qr = XMQuaternionNormalize(Qr);

		XMVECTOR Q = XMVectorSwizzle<3,2,1,0>(Qr);
		Q = XMVectorMultiply(Qe,Q);
		Q = XMVector4Dot(Q,ControlX.v);
		vT = XMVectorAndInt(Q,g_XMMaskX);

		Q =  XMVectorSwizzle<2,3,0,1>(Qr);
		Q = XMVectorMultiply(Qe,Q);
		Q = XMVector4Dot(Q,ControlY.v);
		Q = XMVectorAndInt(Q,g_XMMaskY);
		vT = XMVectorOrInt(vT,Q);

		Q =  XMVectorSwizzle<1,0,3,2>(Qr);
		Q = XMVectorMultiply(Qe,Q);
		Q = XMVector4Dot(Q,ControlZ.v);
		Q = XMVectorAndInt(Q,g_XMMaskZ);
		vT = XMVectorOrInt(vT,Q);

		*outRotQuat = Qr;
		*outTrans = vT;
		return true;
	}

	inline XMVECTOR XMVector3Displacement(FXMVECTOR V, FXMVECTOR RotationQuaternion , FXMVECTOR TranslationQuaternion)
	{
		XMVECTOR Qr = RotationQuaternion;
		XMVECTOR Qe = TranslationQuaternion;
		XMVECTOR Dual = XMVector4Dot(Qr,Qe);
		assert(XMVector4NearEqual(Dual,g_XMZero,XMVectorReplicate(0.01f)));

		XMVECTOR vRes = XMVector3Rotate(V,Qr);
		Qr = XMQuaternionConjugate(Qr);
		XMVECTOR vTrans = XMQuaternionMultiply(Qr,Qe);
		//XMVECTOR vTrans = XMVectorSplatW(Qr) * Qe - Qr * XMVectorSplatW(Qe) + XMVector3Cross(Qr,Qe);
		vTrans *= g_XMTwo;

		vRes += vTrans;
		return vRes;
	}

	inline XMVECTOR XMVector3Displacement(FXMVECTOR V, CXMDUALVECTOR TransformDualQuaternion)
	{
		return XMVector3Displacement(V,TransformDualQuaternion.r[0],TransformDualQuaternion.r[1]);
	}

	const float XM_EPSILON	= 1.192092896e-7f;

	inline float invsqrt( float x ) {
		float xhalf = 0.5f*x;
		int i = *(int*)&x;
		i = 0x5f3759df - (i>>1);
		x = *(float*)&i;
		x = x*(1.5f - xhalf*x*x);
		return x;
	}

	inline XMMATRIX XMMatrixZero()
	{
#if defined(_XM_NO_INTRINSICS_) || defined(_XM_SSE_INTRINSICS_) || defined(_XM_ARM_NEON_INTRINSICS_)

		XMMATRIX M;
		M.r[0] = XMVectorZero();
		M.r[1] = XMVectorZero();
		M.r[2] = XMVectorZero();
		M.r[3] = XMVectorZero();
		return M;

#else // _XM_VMX128_INTRINSICS_
#endif // _XM_VMX128_INTRINSICS_
	}

	//#ifndef _Vector4_
	//#define _Vector4_

	class Vector3;
	class Vector4;
	//class Quaternion;
	typedef SimpleMath::Quaternion Quaternion;

	template <typename _T, size_t _Align_Boundary=std::alignment_of<_T>::value>
	class AlignedAllocator
	{
	public:
		typedef	_T		 value_type;
		typedef	size_t		 size_type;
		typedef	ptrdiff_t	 difference_type;
		typedef	_T		*pointer;
		typedef const _T		*const_pointer;
		typedef	_T		&reference;
		typedef const _T		&const_reference;
		inline AlignedAllocator() throw(){}

		template <typename _T2>
		inline  AlignedAllocator(const AlignedAllocator<_T2, _Align_Boundary> &) throw(){}
		inline ~AlignedAllocator() throw(){}	

		inline pointer adress(reference r)
		{ return &r; }

		inline const_pointer adress(const_reference r) const
		{ return &r; }

		inline pointer allocate(size_type n)
		{ return (pointer)_mm_malloc(n*sizeof(value_type), _Align_Boundary); }

		inline void deallocate(pointer p, size_type)
		{ _mm_free(p); }

		inline void construct(pointer p, const_reference wert)
		{ ::new(p) value_type(wert); }

		inline void destroy(pointer p)
		{ /* C4100 */ p; p->~value_type(); }

		inline size_type max_size() const throw()
		{ return size_type(-1) / sizeof(value_type); }

		template <typename _T2>
		struct rebind { typedef AlignedAllocator<_T2, _Align_Boundary> other; };
	};

	//#ifndef _Vector3_
	//#define _Vector3_
	//////////////////////////////////////////////////////////////////////////
	///Summary
	///Helper class for DirectX Math , but whatever , the most efficient way is :
	///Vector3 t; // In memery
	///XMVECOTR vtr = t; // Load into register
	///Bulabula to vtr;
	///t = vtr; // Store into memery
	//////////////////////////////////////////////////////////////////////////
	class Vector3 : public XMFLOAT3
	{
	public:
		inline static float Dot(FXMVECTOR V1,FXMVECTOR V2)
		{
			return XMVectorGetX(XMVector3Dot(V1,V2));
		}
		inline static float Length(FXMVECTOR V)
		{
			return XMVectorGetX(XMVector3Length(V));
		}
		inline static float LengthSq(FXMVECTOR V)
		{
			return XMVectorGetX(XMVector3LengthSq(V));
		}
		inline static float Distance(FXMVECTOR PointA, FXMVECTOR PointB)
		{
			return Vector3::Length(PointA - PointB);
		}
		inline static XMVECTOR Cross(FXMVECTOR V1,FXMVECTOR V2)
		{
			return XMVector3Cross(V1,V2);
		}
		inline static XMVECTOR Normalize(FXMVECTOR V)
		{
			return XMVector3Normalize(V);
		}
		inline static XMVECTOR Rotate(FXMVECTOR V,FXMVECTOR RotateQuaternion)
		{
			return XMVector3Rotate(V,RotateQuaternion);
		}
		inline static XMVECTOR RotateInverse(FXMVECTOR V,FXMVECTOR RotateQuaternion)
		{
			return XMVector3InverseRotate(V,RotateQuaternion);
		}
	public:
		inline Vector3(void) {}
		inline explicit Vector3(FXMVECTOR vtr){
			XMStoreFloat3(this,vtr);
		}
		inline Vector3(float x,float y,float z)
			: XMFLOAT3(x,y,z)
		{}
		inline explicit Vector3(float* pArray)
			: XMFLOAT3(pArray)
		{}
		inline explicit Vector3(const XMFLOAT4& vtr4)
			: XMFLOAT3(vtr4.x,vtr4.y,vtr4.z)
		{}

		inline explicit Vector3(const XMFLOAT3& vtr3)
			: XMFLOAT3(vtr3.x,vtr3.y,vtr3.z)
		{}

		inline ~Vector3(void){
		}

		inline Vector3& operator = (FXMVECTOR rhs){
			XMStoreFloat3(this,rhs);
			return *this;
		}

		inline Vector3& operator = (const XMFLOAT3 &rhs){
			*(XMFLOAT3*)this = rhs;
			return *this;
		}

		inline void Set(float _x,float _y,float _z){
			x = _x; y=_y; z=_z;
		}

		inline Vector3& operator += (const Vector3& rhs){
			x += rhs.x;y += rhs.y;z += rhs.z;
			return *this;
		}
		inline Vector3& operator += (FXMVECTOR rhs){
			XMVECTOR vThis = *this;
			vThis += rhs;
			*this = vThis;
			return *this;
		}

		inline Vector3& operator -= (const Vector3& rhs){
			x -= rhs.x;y -= rhs.y;z -= rhs.z;
			return *this;
		}

		inline Vector3& operator -= (FXMVECTOR rhs){
			XMVECTOR vThis = *this;
			vThis -= rhs;
			*this = vThis;
			return *this;
		}
		// Each Product of two 3d-vector
		inline Vector3& operator *= (const Vector3& rhs){
			x *= rhs.x;y *= rhs.y;z *= rhs.z;
			return *this;
		}

		inline Vector3& operator *= (const float rhs){
			x *= rhs;y *= rhs;z *= rhs;
			return *this;
		}
		inline Vector3& operator /= (const float rhs){
			auto f = 1.0f / rhs;
			x *= f;y *= f;z *= f;
			return *this;
		}

		// Equals to transform this with rhs
		inline Vector3& operator *= (CXMMATRIX rhs){
			*this = XMVector3TransformCoord(*this,rhs);
			return *this;
		}

		//unary operator positive
		inline Vector3 operator + () const{
			return *this;
		}

		//unary operator negative
		inline Vector3 operator - () const{
			return Vector3(-x,-y,-z);
		}

		inline bool operator == (const Vector3& rhs) const{
			return XMVector3Equal(*this,rhs);
		}
		inline bool operator != (const Vector3& rhs) const{
			return !(*this == rhs);
		}

		inline float Length() const{
			return std::sqrtf(LengthSq());
		}
		inline float LengthSq() const{
			return x*x+y*y+z*z;
		}

		// This method will normalize itself.
		inline void Normalize() {
			float L = invsqrt(LengthSq());
			*this *= L;
			//*this = XMVector3Normalize((XMVECTOR)*this);
		}

		inline Vector3 Normalization() const
		{
			float factor = invsqrt(LengthSq());
			Vector3 temp = (*this);
			temp *= factor;
			return temp;
		}

		inline void Rotate(FXMVECTOR RotateQuaternion) {
			*this = XMVector3Rotate((XMVECTOR)*this,RotateQuaternion);
		}
		inline void RotateInverse(FXMVECTOR RotateQuaternion) {
			*this = XMVector3InverseRotate((XMVECTOR)*this,RotateQuaternion);
		}

		inline void Transform(CXMMATRIX TransformMatrix) {
			*this = XMVector3TransformCoord((XMVECTOR)*this,TransformMatrix);
		}

		inline operator XMFLOAT4 () const{
			return XMFLOAT4(x,y,z,1.0f);
		}

		inline operator const XMFLOAT3& () const{
			return *this;
		}

		inline operator XMFLOAT3& (){
			return *this;
		}

		inline operator XMVECTOR () const{
			return XMLoadFloat3(this);
		}

		inline operator float* ()
		{
			return reinterpret_cast<float*>(this);
		}

		inline operator const float * () const{
			return reinterpret_cast<const float*>(this);
		}
	};

	inline Vector3 operator + (const Vector3 & lhs , const Vector3 & rhs){
		Vector3 temp = lhs;
		temp += rhs;
		return temp;
	}

	inline Vector3 operator - (const Vector3 & lhs , const Vector3 & rhs){
		Vector3 temp = lhs;
		temp -= rhs;
		return temp;
	}

	inline Vector3 operator * (const Vector3 & lhs , float scale) {
		Vector3 temp = lhs;
		temp *= scale;
		return temp;
	}

	inline Vector3 operator * (float scale,const Vector3& rhs){
		Vector3 temp = rhs;
		temp *= scale;
		return temp;
	}

	inline Vector3 operator / (const Vector3 & lhs , const float rhs) {
		Vector3 temp = lhs;
		temp /= rhs;
		return temp;
	}

	// Dot product of two 3d-vector
	inline float operator * (const Vector3 & lhs , const Vector3& rhs){
		return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
	}

	// Cross product of two 3d-vector
	inline Vector3 operator ^ (const Vector3 & lhs , const Vector3& rhs){
		Vector3 temp;
		temp.x = lhs.y*rhs.z - lhs.z*rhs.y;
		temp.y = lhs.z*rhs.x - lhs.x*rhs.z;
		temp.z = lhs.x*rhs.y - lhs.y*rhs.x;
		return temp;
		//		return XMVector3Cross((XMVECTOR)*this,(XMVECTOR)rhs);
	}

	// Transform this vector with a matrix
	inline XMVECTOR operator * (const Vector3 & lhs , CXMMATRIX rhs){
		return XMVector3Transform((XMVECTOR)lhs,rhs);
	}

	//#endif // !_Vector3_

	class Vector4 : public XMFLOAT4
	{
	public:
		inline Vector4(){}
		inline explicit Vector4(FXMVECTOR vtr){
			XMStoreFloat4(this,vtr);
		}
		inline Vector4(float _x,float _y,float _z,float _w)
			: XMFLOAT4(_x,_y,_z,_w)
		{}
		inline explicit Vector4(_In_reads_(4) float* pArray)
			:XMFLOAT4(pArray)
		{}
		inline explicit Vector4(const XMFLOAT3 &vtr3)
			: XMFLOAT4(vtr3.x,vtr3.y,vtr3.z,1.0f)
		{}
		inline explicit Vector4(const XMFLOAT3 &vtr3 ,const float _w)
			: XMFLOAT4(vtr3.x,vtr3.y,vtr3.z,_w)
		{}
		inline  ~Vector4()
		{}
		inline void Set(float _x,float _y,float _z,float _w){
			x =_x; y=_y; z=_z; w=_w;
		}
		inline Vector4& operator = (FXMVECTOR rhs){
			XMStoreFloat4(this,rhs);
			return *this;
		}

		inline Vector4& operator = (const XMFLOAT4 &rhs){
			*this = rhs;
			return *this;
		}

		inline operator const XMFLOAT4& () const{
			return *this;
		}

		inline operator XMFLOAT4& (){
			return *this;
		}

		inline operator XMVECTOR () const{
			return XMLoadFloat4(this);
		}

		inline Vector3& xyz() {
			return *(reinterpret_cast<Vector3*>(this));
		}

		//inline Vector4 operator + (const Vector4 & rhs) const{
		//	return (XMVECTOR)*this + (XMVECTOR)rhs;
		//}
		//inline Vector4 operator + (CXMVECTOR rhs) const{
		//	return (XMVECTOR)*this + rhs;
		//}
		//inline Vector4 operator - (const Vector4 & rhs) const{
		//	return (XMVECTOR)*this - (XMVECTOR)rhs;
		//}
		//inline Vector4 operator - (CXMVECTOR rhs) const{
		//	return (XMVECTOR)*this - rhs;
		//}
		//inline Vector4 operator * (const float rhs) const{
		//	return (XMVECTOR)*this * rhs;
		//}
		//inline Vector4 operator / (const float rhs) const{
		//	return (XMVECTOR)*this / rhs;
		//}
		//inline float operator * (const Vector4& rhs) const{
		//	return XMVectorGetX(XMVector4Dot(*this,rhs));
		//}

	};
	//#endif _Vector4_

	inline static XMVECTOR XMQuaternionRotationVectorToVector(FXMVECTOR v1, FXMVECTOR v2){
		assert(!XMVector3Equal(v1, XMVectorZero()));
		assert(!XMVector3Equal(v2, XMVectorZero()));
		XMVECTOR n1 = XMVector3Normalize(v1);
		XMVECTOR n2 = XMVector3Normalize(v2);
		if (XMVector4NearEqual(n1,n2,g_XMEpsilon))
			return XMQuaternionIdentity();
		XMVECTOR axias = XMVector3Cross(n1,n2);
		float angle = std::acosf(XMVectorGetX(XMVector3Dot(n1,n2)));
		auto rot = XMQuaternionRotationAxis(axias,angle);
		return rot;
	}


	//class Quaternion : public  XMFLOAT4
	//{
	//public:
	//	inline static XMVECTOR RotationQuaternion(FXMVECTOR Axis, float Angle){
	//		return XMQuaternionRotationAxis(Axis,Angle);
	//	}
	//	inline static XMVECTOR RotationQuaternion(CXMMATRIX M){
	//		return XMQuaternionRotationMatrix(M);
	//	}
	//	inline static XMVECTOR RotationQuaternionRollYawPitch(const float Pitch , const float Yaw , const float Roll){
	//		return XMQuaternionRotationRollPitchYaw(Pitch,Yaw,Roll);
	//	}
	//	// Get the Rotation Quaternion from v0 -> v1
	//	inline static XMVECTOR RotationQuaternion(FXMVECTOR v1, FXMVECTOR v2){
	//		assert(!XMVector3Equal(v1, XMVectorZero()));
	//		assert(!XMVector3Equal(v2, XMVectorZero()));
	//		XMVECTOR n1 = XMVector3Normalize(v1);
	//		XMVECTOR n2 = XMVector3Normalize(v2);
	//		if (XMVector4NearEqual(n1,n2,g_XMEpsilon))
	//			return XMQuaternionIdentity();
	//		XMVECTOR axias = XMVector3Cross(n1,n2);
	//		float angle = std::acosf(XMVectorGetX(XMVector3Dot(n1,n2)));
	//		auto rot = XMQuaternionRotationAxis(axias,angle);
	//		//#if _DEBUG
	//		//			auto res = XMVector3Rotate(n1,rot);
	//		//			XMINT4 Res ;
	//		//			XMStoreSInt4(&Res,XMVectorNearEqual(res,n2,g_XMEpsilon*100));
	//		//			assert(Res.x);
	//		//			assert(Res.y);
	//		//			assert(Res.z);
	//		//#endif // _DEBUG
	//		return rot;
	//	}
	//	inline static const Quaternion Identity(){
	//		return XMQuaternionIdentity();
	//	}
	//public:
	//	inline Quaternion()
	//	{
	//		//			XMStoreFloat4(this,XMQuaternionIdentity());
	//	}
	//	inline Quaternion(FXMVECTOR vtr){
	//		XMStoreFloat4(this,vtr);
	//	}
	//	inline Quaternion(const float* pArray)
	//		: XMFLOAT4(pArray)
	//	{}

	//	// Construct a Quaternion from Rotate Axias and Angle
	//	inline Quaternion(FXMVECTOR Axias , const float Angle)
	//	{
	//		auto vtr = XMQuaternionRotationAxis(Axias,Angle);
	//		XMStoreFloat4(this,vtr);
	//	}
	//	// Construct a Quaternion from Roll,Pitch,Yaw
	//	inline Quaternion(const float Roll , const float Pitch , const float Yaw)
	//	{
	//		auto vtr = XMQuaternionRotationRollPitchYaw(Roll,Pitch,Yaw);
	//		XMStoreFloat4(this,vtr);
	//	}
	//	inline ~Quaternion()
	//	{
	//	}

	//public:
	//	inline void Identify(){
	//		*this = g_XMIdentityR3;
	//	}

	//	inline XMVECTOR Conjugate() const{
	//		return XMQuaternionConjugate((XMVECTOR)*this);
	//	}

	//	inline XMVECTOR Inverse() const{
	//		return XMQuaternionInverse((XMVECTOR)*this);
	//	}

	//	/// <summary>
	//	/// Get the Rotation Axis of the rotation represent by this Quaternion.
	//	/// </summary>
	//	/// <returns>Rotation Axis</returns>
	//	inline Vector3 Axis() const
	//	{
	//		Vector3 axis ((const XMFLOAT4)*this);
	//		axis.Normalize();
	//		return axis;
	//	}

	//	/// <summary>
	//	/// Get the Rotation Angle of the rotation represent by this Quaternion.
	//	/// </summary>
	//	/// <returns>Rotation Angle</returns>
	//	inline float Angle() const
	//	{
	//		return acosf(w) * 2;
	//	}


	//	inline Quaternion& operator = (FXMVECTOR vtr){
	//		XMStoreFloat4(this,vtr);
	//		return *this;
	//	}

	//	inline Quaternion& operator = (const XMFLOAT4 &vtr){
	//		this->x = vtr.x; this->y = vtr.y; this->z = vtr.z; this->w = vtr.w;
	//		return *this;
	//	}

	//	inline Quaternion& operator *= (FXMVECTOR rhs)
	//	{
	//		*this = XMQuaternionMultiply(rhs,(XMVECTOR)*this);
	//		return *this;
	//	}

	//	inline XMVECTOR operator * (const Quaternion& rhs) const
	//	{
	//		return XMQuaternionMultiply((XMVECTOR)rhs,(XMVECTOR)(*this));
	//	}

	//	inline operator XMFLOAT4& ()
	//	{
	//		return *this;
	//	}

	//	inline operator const XMFLOAT4& () const
	//	{
	//		return *this;
	//	}

	//	inline operator XMVECTOR () const
	//	{
	//		return XMLoadFloat4(this);
	//	}
	//};



	// 4X4Matrix Class for storage data
	class Matrix4X4 : public XMFLOAT4X4
	{
	public:
		inline Matrix4X4();

		inline Matrix4X4(CXMMATRIX Mtx){
			XMStoreFloat4x4(this,Mtx);
		}

		inline Matrix4X4& operator = (CXMMATRIX Mtx){
			XMStoreFloat4x4(this,Mtx);
		}

		inline operator XMMATRIX () const
		{
			return XMLoadFloat4x4(this);
		}
	};

	// 4X4Matrix Class for storage data
	class Matrix4X4A : public XMFLOAT4X4A , public AlignedNew<Matrix4X4A>
	{
	public:
		inline Matrix4X4A();

		inline Matrix4X4A(CXMMATRIX Mtx){
			XMStoreFloat4x4A(this,Mtx);
		}

		inline Matrix4X4A& operator = (CXMMATRIX Mtx){
			XMStoreFloat4x4A(this,Mtx);
		}

		inline operator XMMATRIX () const
		{
			return XMLoadFloat4x4A(this);
		}
	};

	// 3x4 Matrix: 32 bit floating point components
	// Row Major , Usually use to pass float4x3 matrix to GPU
	// Transpose befor store and after load
	struct XMFLOAT3X4
	{
		union
		{
			struct
			{
				float _11, _12, _13, _14;
				float _21, _22, _23, _24;
				float _31, _32, _33, _34;
			};
			float m[3][4];
		};

		XMFLOAT3X4() {}
		XMFLOAT3X4(float m00, float m01, float m02, float m03,
			float m10, float m11, float m12, float m13,
			float m20, float m21, float m22, float m23);
		explicit XMFLOAT3X4(_In_reads_(12) const float *pArray);

		float       operator() (size_t Row, size_t Column) const { return m[Row][Column]; }
		float&      operator() (size_t Row, size_t Column) { return m[Row][Column]; }

		XMFLOAT3X4& operator= (const XMFLOAT3X4& Float4x3);
	};

	inline XMFLOAT3X4::XMFLOAT3X4
		(
		float m00, float m01, float m02, float m03,
		float m10, float m11, float m12, float m13,
		float m20, float m21, float m22, float m23
		)
	{
		m[0][0] = m00;
		m[0][1] = m01;
		m[0][2] = m02;
		m[0][3] = m03;

		m[1][0] = m10;
		m[1][1] = m11;
		m[1][2] = m12;
		m[1][3] = m13;

		m[2][0] = m20;
		m[2][1] = m21;
		m[2][2] = m22;
		m[2][3] = m23;

	}

	//------------------------------------------------------------------------------
	_Use_decl_annotations_
		inline XMFLOAT3X4::XMFLOAT3X4
		(
		const float* pArray
		)
	{
		assert( pArray != nullptr );

		m[0][0] = pArray[0];
		m[0][1] = pArray[1];
		m[0][2] = pArray[2];
		m[0][3] = pArray[3];

		m[1][0] = pArray[4];
		m[1][1] = pArray[5];
		m[1][2] = pArray[6];
		m[1][3] = pArray[7];

		m[2][0] = pArray[8];
		m[2][1] = pArray[9];
		m[2][2] = pArray[10];
		m[2][3] = pArray[11];

	}

	//------------------------------------------------------------------------------
	inline XMFLOAT3X4& XMFLOAT3X4::operator=
		(
		const XMFLOAT3X4& Float4x4
		)
	{
		XMVECTOR V1 = XMLoadFloat4((const XMFLOAT4*)&Float4x4._11);
		XMVECTOR V2 = XMLoadFloat4((const XMFLOAT4*)&Float4x4._21);
		XMVECTOR V3 = XMLoadFloat4((const XMFLOAT4*)&Float4x4._31);

		XMStoreFloat4((XMFLOAT4*)&_11, V1);
		XMStoreFloat4((XMFLOAT4*)&_21, V2);
		XMStoreFloat4((XMFLOAT4*)&_31, V3);

		return *this;
	}

	inline void XMStoreFloat3x4(
		XMFLOAT3X4 *pDestination,
		CXMMATRIX M
		)
	{
		assert(pDestination);
#if defined(_XM_NO_INTRINSICS_)

		pDestination->m[0][0] = M.r[0].vector4_f32[0];
		pDestination->m[0][1] = M.r[0].vector4_f32[1];
		pDestination->m[0][2] = M.r[0].vector4_f32[2];
		pDestination->m[0][3] = M.r[0].vector4_f32[3];

		pDestination->m[1][0] = M.r[1].vector4_f32[0];
		pDestination->m[1][1] = M.r[1].vector4_f32[1];
		pDestination->m[1][2] = M.r[1].vector4_f32[2];
		pDestination->m[1][3] = M.r[1].vector4_f32[3];

		pDestination->m[2][0] = M.r[2].vector4_f32[0];
		pDestination->m[2][1] = M.r[2].vector4_f32[1];
		pDestination->m[2][2] = M.r[2].vector4_f32[2];
		pDestination->m[2][3] = M.r[2].vector4_f32[3];

#elif defined(_XM_ARM_NEON_INTRINSICS_)
		vst1q_f32( reinterpret_cast<float*>(&pDestination->_11), M.r[0] );
		vst1q_f32( reinterpret_cast<float*>(&pDestination->_21), M.r[1] );
		vst1q_f32( reinterpret_cast<float*>(&pDestination->_31), M.r[2] );
#elif defined(_XM_SSE_INTRINSICS_)
		_mm_storeu_ps( &pDestination->_11, M.r[0] );
		_mm_storeu_ps( &pDestination->_21, M.r[1] );
		_mm_storeu_ps( &pDestination->_31, M.r[2] );
#else // _XM_VMX128_INTRINSICS_
#endif // _XM_VMX128_INTRINSICS_
	}

	inline XMMATRIX XMLoadFloat3x4
		(
		const XMFLOAT3X4* pSource
		)
	{
		assert(pSource);
#if defined(_XM_NO_INTRINSICS_)

		XMMATRIX M;
		M.r[0].vector4_f32[0] = pSource->m[0][0];
		M.r[0].vector4_f32[1] = pSource->m[0][1];
		M.r[0].vector4_f32[2] = pSource->m[0][2];
		M.r[0].vector4_f32[3] = pSource->m[0][3];

		M.r[1].vector4_f32[0] = pSource->m[1][0];
		M.r[1].vector4_f32[1] = pSource->m[1][1];
		M.r[1].vector4_f32[2] = pSource->m[1][2];
		M.r[1].vector4_f32[3] = pSource->m[1][3];

		M.r[2].vector4_f32[0] = pSource->m[2][0];
		M.r[2].vector4_f32[1] = pSource->m[2][1];
		M.r[2].vector4_f32[2] = pSource->m[2][2];
		M.r[2].vector4_f32[3] = pSource->m[2][3];

		return M;

#elif defined(_XM_ARM_NEON_INTRINSICS_)
		XMMATRIX M;
		M.r[0] = vld1q_f32( reinterpret_cast<const float*>(&pSource->_11) );
		M.r[1] = vld1q_f32( reinterpret_cast<const float*>(&pSource->_21) );
		M.r[2] = vld1q_f32( reinterpret_cast<const float*>(&pSource->_31) );
		return M;
#elif defined(_XM_SSE_INTRINSICS_)
		XMMATRIX M;
		M.r[0] = _mm_loadu_ps( &pSource->_11 );
		M.r[1] = _mm_loadu_ps( &pSource->_21 );
		M.r[2] = _mm_loadu_ps( &pSource->_31 );
		M.r[3] = g_XMIdentityR3;
		return M;
#elif defined(XM_NO_MISALIGNED_VECTOR_ACCESS)
#endif // _XM_VMX128_INTRINSICS_
	}

	class RigidObject
	{
	public:
		inline RigidObject()
		{
			Position.Set(.0f,.0f,.0f);
			Orientation = XMQuaternionIdentity();
		}

		inline XMMATRIX WorldMatrix() const
		{
			return XMMatrixRotationQuaternion(Orientation) * XMMatrixTranslationFromVector(Position);
		}

		inline XMMATRIX InverseWorldMatrix() const
		{
			XMVECTOR invRotation = XMQuaternionInverse(Orientation);
			return XMMatrixTranslationFromVector(-Position) * XMMatrixRotationQuaternion(invRotation);
		}

		inline void Identitify(){
			Position.Set(0.0f,0.0f,0.0f);
			Orientation = XMQuaternionIdentity();
		}
		inline void Translate(const Vector3 &Offset){
			Position += Offset;
		}
		inline void Rotate(FXMVECTOR RotateQuaterion){
			Orientation = XMQuaternionMultiply(Orientation,RotateQuaterion);
		}
		inline void Rotate(const Vector3 &Axis , const float Angle){
			XMVECTOR qRotation = XMQuaternionRotationAxis(Axis,Angle);
			Orientation = XMQuaternionMultiply(Orientation,qRotation);
		}
		inline void Rotate(const float Yaw , const float Pitch ,const float Roll){
			XMVECTOR qRotation = XMQuaternionRotationRollPitchYaw(Pitch,Yaw,Roll);
			Orientation = XMQuaternionMultiply(Orientation,qRotation);
		}

	public:
		inline const static RigidObject Identity()
		{
			return RigidObject();
		}
	public:
		Vector3 Position;
		Quaternion Orientation;
	};

	template <typename T>
	inline bool Sgn(T val){
		return val>T(0);
	}

	namespace SimpleMath
	{
		//template <typename T>
		//inline float Length(const T &V)
		//{
		//	return V.Length();
		//}

		//template <typename T>
		//inline float Length(DirectX::FXMVECTOR V)
		//{
		//	return V.Length();
		//}

		class DualQuaternion
			: boost::additive<DualQuaternion>
			, boost::multipliable<DualQuaternion>
			, boost::multiplicative<DualQuaternion,float>
		{
		public:
			DirectX::Quaternion Qr,Qe;
			DualQuaternion()
				: Qr(),Qe(0.0f,0.0f,0.0f,0.0f)
			{}
			DualQuaternion(const Quaternion& Rotation,const Vector3& Translation)
				: Qr(Rotation)
			{
				Qr.Normalize();
				const float* Q0 = reinterpret_cast<float*>(&Qr);
				const float* T = reinterpret_cast<const float*>(&Translation);
				Qe.w = -0.5f*(T[0]*Q0[0] + T[1]*Q0[1] + T[2]*Q0[2]);
				Qe.x = 0.5f*( T[0]*Q0[3] + T[1]*Q0[2] - T[2]*Q0[1]);
				Qe.y = 0.5f*(-T[0]*Q0[2] + T[1]*Q0[3] + T[2]*Q0[0]);
				Qe.z = 0.5f*( T[0]*Q0[1] - T[1]*Q0[0] + T[2]*Q0[3]);
			}
			DualQuaternion(FXMVECTOR Qr,FXMVECTOR Qe)
				: Qr(Qr) , Qe(Qe)
			{}
			DualQuaternion(CXMDUALVECTOR DQ)
				: Qr(DQ.r[0]) , Qe(DQ.r[1])
			{}

			explicit DualQuaternion(_In_reads_(8) const float* pArray)
				: Qr(pArray) , Qe(pArray+4)
			{}
			explicit DualQuaternion(_In_reads_(2) const Quaternion* pQArray)
				: Qr(*pQArray) , Qe(*(pQArray+1))
			{}

			inline operator XMDUALVECTOR() const
			{
				XMDUALVECTOR dqRes;
				dqRes.r[0] = Qr;
				dqRes.r[1] = Qe;
				return dqRes;
			}

			void Normarlize();
			void Normarlize( DualQuaternion& result) const;

			void Inverse();
			void Inverse( DualQuaternion& result) const;

			void Conjugate()
			{
				Qr.Conjugate();
				Qe.Conjugate();
			}

			void Conjugate( DualQuaternion& result ) const
			{
				result.Qr = XMQuaternionConjugate(Qr);
				result.Qe = XMQuaternionConjugate(Qe);
			}

			Vector2 Norm() const
			{
				Vector2 value;
				XMVECTOR q0 = Qr;
				XMVECTOR q1 = Qe;
				XMVECTOR len = XMQuaternionLength(q0);
				q1 = XMVector4Dot(q0,q1);
				q0 = XMVectorDivide(q1,len);
				q1 = XMVectorSelect(len,q0,g_XMSelect0101);
				value = q1;
				return value;
			}

			bool IsUnit() const
			{
				XMVECTOR q0 = Qr;
				XMVECTOR q1 = Qe;
				q1 = XMVector4Dot(q0,q1);
				return XMVector4NearEqual(q0,g_XMZero.v,g_XMEpsilon.v);
			}

			bool Decompose(Quaternion& Rotation,Vector3& Translation) const
			{
				const auto& Q = Rotation = XMQuaternionNormalize(Qr);
				// translation vector:
				Translation.x = 2.0f*(-Qe.w*Q.x + Qe.x*Q.w - Qe.y*Q.z + Qe.z*Q.y);
				Translation.y = 2.0f*(-Qe.w*Q.y + Qe.x*Q.z + Qe.y*Q.w - Qe.z*Q.x);
				Translation.z = 2.0f*(-Qe.w*Q.z - Qe.x*Q.y + Qe.y*Q.x + Qe.z*Q.w);
			}

			DualQuaternion& operator+= (const DualQuaternion& rhs)
			{
				Qr += rhs.Qr;
				Qe += rhs.Qe;
				return *this;
			}

			DualQuaternion& operator-= (const DualQuaternion& rhs)
			{
				Qr -= rhs.Qr;
				Qe -= rhs.Qe;
				return *this;
			}

			DualQuaternion& operator*= (const DualQuaternion& rhs)
			{
				XMVECTOR A = this->Qr;
				XMVECTOR B = this->Qe;
				XMVECTOR C = rhs.Qr;
				XMVECTOR D = rhs.Qe;
				D = XMQuaternionMultiply(A,D);
				B = XMQuaternionMultiply(B,C);
				Qe = XMVectorAdd(D,B);
				Qr = XMQuaternionMultiply(A,C);
			}

			DualQuaternion& operator*= (float scale)
			{
				Qr *= scale;
				Qr *= scale;
				return *this;
			}

			DualQuaternion& operator/= (float scale)
			{
				float s = 1.0f/scale;
				return (*this) *= s;
			}
		};
	}

}

#endif