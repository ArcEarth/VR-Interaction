#ifndef DX_MATH_EXT_H
#define DX_MATH_EXT_H
#endif
#pragma once

#include <DirectXMath.h>
#include <DirectXCollision.h>
#include <SimpleMath.h>
#include <smmintrin.h>
#include <type_traits>
#include <boost\operators.hpp>

namespace DirectX
{

	using SimpleMath::Vector2;
	using SimpleMath::Vector3;
	using SimpleMath::Vector4;
	using SimpleMath::Quaternion;
	using SimpleMath::Color;
	using SimpleMath::Plane;
	using SimpleMath::Ray;
	typedef SimpleMath::Matrix Matrix4x4;

	//inline namespace Operators
	//{
		using SimpleMath::operator*;
		using SimpleMath::operator+;
		using SimpleMath::operator-;
		using SimpleMath::operator/;
	//}


	// Derive from this to customize operator new and delete for
	// types that have special heap alignment requirements.
	//
	// Example usage:
	//
	//      _declspec(align(16)) struct MyAlignedType : public AlignedNew<MyAlignedType>
	template<typename TDerived>
	struct AlignedNew
	{
		// Allocate aligned memory.
		static void* operator new (size_t size)
		{
			const size_t alignment = __alignof(TDerived);

			static_assert(alignment > 8, "AlignedNew is only useful for types with > 8 byte alignment. Did you forget a _declspec(align) on TDerived?");

			void* ptr = _aligned_malloc(size, alignment);

			if (!ptr)
				throw std::bad_alloc();

			return ptr;
		}


			// Free aligned memory.
			static void operator delete (void* ptr)
		{
			_aligned_free(ptr);
		}


		// Array overloads.
		static void* operator new[](size_t size)
		{
			return operator new(size);
		}


			static void operator delete[](void* ptr)
		{
			operator delete(ptr);
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
		assert(pArray != nullptr);

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
		XMVECTOR V1 = XMLoadFloat4((const XMFLOAT4*) &Float4x4._11);
		XMVECTOR V2 = XMLoadFloat4((const XMFLOAT4*) &Float4x4._21);
		XMVECTOR V3 = XMLoadFloat4((const XMFLOAT4*) &Float4x4._31);

		XMStoreFloat4((XMFLOAT4*) &_11, V1);
		XMStoreFloat4((XMFLOAT4*) &_21, V2);
		XMStoreFloat4((XMFLOAT4*) &_31, V3);

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
		vst1q_f32(reinterpret_cast<float*>(&pDestination->_11), M.r[0]);
		vst1q_f32(reinterpret_cast<float*>(&pDestination->_21), M.r[1]);
		vst1q_f32(reinterpret_cast<float*>(&pDestination->_31), M.r[2]);
#elif defined(_XM_SSE_INTRINSICS_)
		_mm_storeu_ps(&pDestination->_11, M.r[0]);
		_mm_storeu_ps(&pDestination->_21, M.r[1]);
		_mm_storeu_ps(&pDestination->_31, M.r[2]);
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
		M.r[0] = vld1q_f32(reinterpret_cast<const float*>(&pSource->_11));
		M.r[1] = vld1q_f32(reinterpret_cast<const float*>(&pSource->_21));
		M.r[2] = vld1q_f32(reinterpret_cast<const float*>(&pSource->_31));
		return M;
#elif defined(_XM_SSE_INTRINSICS_)
		XMMATRIX M;
		M.r[0] = _mm_loadu_ps(&pSource->_11);
		M.r[1] = _mm_loadu_ps(&pSource->_21);
		M.r[2] = _mm_loadu_ps(&pSource->_31);
		M.r[3] = g_XMIdentityR3;
		return M;
#elif defined(XM_NO_MISALIGNED_VECTOR_ACCESS)
#endif // _XM_VMX128_INTRINSICS_
	}

	struct XMDUALVECTOR;
	// Calling convetion
	typedef const XMDUALVECTOR& CXMDUALVECTOR;
	typedef XMDUALVECTOR FXMDUALVECTOR;

#if (defined(_M_IX86) || defined(_M_AMD64) || defined(_M_ARM)) && defined(_XM_NO_INTRINSICS_)
	struct XMDUALVECTOR
#else
	__declspec(align(16)) struct XMDUALVECTOR
		: boost::additive<DirectX::CXMDUALVECTOR>
		, boost::multiplicative<DirectX::CXMDUALVECTOR, float>
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
		XMDUALVECTOR(FXMVECTOR V0, FXMVECTOR V1) { r[0] = V0; r[1] = V1; }
		XMDUALVECTOR(float m00, float m01, float m02, float m03,
			float m10, float m11, float m12, float m13)
		{
			r[0] = XMVectorSet(m00, m01, m02, m03);
			r[1] = XMVectorSet(m10, m11, m12, m13);
		}
		explicit XMDUALVECTOR(_In_reads_(8) const float *pArray)
		{
			r[0] = XMLoadFloat4(reinterpret_cast<const XMFLOAT4*>(pArray));
			r[1] = XMLoadFloat4(reinterpret_cast<const XMFLOAT4*>(pArray + 4));
		}

		XMDUALVECTOR&   operator= (const XMDUALVECTOR& DV) { r[0] = DV.r[0]; r[1] = DV.r[1]; return *this; }

		XMDUALVECTOR    operator+ () const { return *this; }
		XMDUALVECTOR    operator- () const
		{
			XMDUALVECTOR dvResult(r[0], -r[1]);
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

	XMVECTOR XMVector3Displacement(FXMVECTOR V, FXMVECTOR RotationQuaternion, FXMVECTOR TranslationQuaternion);

	inline XMDUALVECTOR XMDualQuaternionTranslation(FXMVECTOR T)
	{
		static const XMVECTORF32 Control = { 0.5f,0.5f,0.5f,.0f };
		XMVECTOR Qe = XMVectorMultiply(T, Control);
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

	inline XMDUALVECTOR XMDualQuaternionRigidTransform(FXMVECTOR RotationOrigin, FXMVECTOR RotationQuaternion, FXMVECTOR Translation)
	{
		static const XMVECTORF32 Control = { 0.5f,0.5f,0.5f,.0f };

		XMVECTOR Qr = RotationQuaternion;
		XMVECTOR Qe = RotationOrigin + Translation;
		Qe = XMVectorMultiply(Qe, Control);
		XMVECTOR Qo = XMVectorMultiply(RotationOrigin, Control);
		Qo = XMVectorNegate(Qo);

		Qe = XMQuaternionMultiply(Qr, Qe);
		Qo = XMQuaternionMultiply(Qo, Qr);
		Qe += Qo;

		XMDUALVECTOR dqRes;
		dqRes.r[0] = Qr;
		dqRes.r[1] = Qe;
		return dqRes;
	}

	inline XMDUALVECTOR XMDualQuaternionRotationTranslation(FXMVECTOR Q, FXMVECTOR T)
	{
		// non-dual part (just copy q0):
		// dual part:
		static const XMVECTORF32 Control = { 0.5f,0.5f,0.5f,.0f };
		XMVECTOR Qe = XMVectorMultiply(T, Control);
		Qe = XMQuaternionMultiply(Q, Qe);
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
		dqRes.r[0] = XMVectorDivide(Dq.r[0], Length);
		dqRes.r[1] = XMVectorDivide(Dq.r[1], Length);
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
		XMDUALVECTOR dvResult(Dq.r[0], XMVectorNegate(Dq.r[1]));
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

	inline XMDUALVECTOR XMDualQuaternionMultipy(CXMDUALVECTOR DQ0, CXMDUALVECTOR DQ1)
	{
		XMDUALVECTOR dvResult;
		dvResult.r[0] = XMQuaternionMultiply(DQ0.r[0], DQ1.r[1]);
		dvResult.r[1] = XMQuaternionMultiply(DQ0.r[0], DQ1.r[1]);
		dvResult.r[1] += XMQuaternionMultiply(DQ0.r[1], DQ1.r[0]);
		return dvResult;
	}

	inline bool XMDualQuaternionDecompose(XMVECTOR* outRotQuat, XMVECTOR* outTrans, CXMDUALVECTOR Dq)
	{
		static const XMVECTORF32 ControlX = { 2.0f,-2.0f,2.0f,-2.0f };
		static const XMVECTORF32 ControlY = { 2.0f,2.0f,-2.0f,-2.0f };
		static const XMVECTORF32 ControlZ = { -2.0f,2.0f,2.0f,-2.0f };

		//vT.x = 2.0f*( Qe.x*Q.w - Qe.y*Q.z + Qe.z*Q.y - Qe.w*Q.x);
		//vT.y = 2.0f*( Qe.x*Q.z + Qe.y*Q.w - Qe.z*Q.x - Qe.w*Q.y);
		//vT.z = 2.0f*(-Qe.x*Q.y + Qe.y*Q.x + Qe.z*Q.w - Qe.w*Q.z);
		XMVECTOR vT;
		XMVECTOR Qr = Dq.r[0];
		XMVECTOR Qe = Dq.r[1];
		Qr = XMQuaternionNormalize(Qr);

		XMVECTOR Q = XMVectorSwizzle<3, 2, 1, 0>(Qr);
		Q = XMVectorMultiply(Qe, Q);
		Q = XMVector4Dot(Q, ControlX.v);
		vT = XMVectorAndInt(Q, g_XMMaskX);

		Q = XMVectorSwizzle<2, 3, 0, 1>(Qr);
		Q = XMVectorMultiply(Qe, Q);
		Q = XMVector4Dot(Q, ControlY.v);
		Q = XMVectorAndInt(Q, g_XMMaskY);
		vT = XMVectorOrInt(vT, Q);

		Q = XMVectorSwizzle<1, 0, 3, 2>(Qr);
		Q = XMVectorMultiply(Qe, Q);
		Q = XMVector4Dot(Q, ControlZ.v);
		Q = XMVectorAndInt(Q, g_XMMaskZ);
		vT = XMVectorOrInt(vT, Q);

		*outRotQuat = Qr;
		*outTrans = vT;
		return true;
	}

	inline XMVECTOR XMVector3Displacement(FXMVECTOR V, FXMVECTOR RotationQuaternion, FXMVECTOR TranslationQuaternion)
	{
		XMVECTOR Qr = RotationQuaternion;
		XMVECTOR Qe = TranslationQuaternion;
		XMVECTOR Dual = XMVector4Dot(Qr, Qe);
		assert(XMVector4NearEqual(Dual, g_XMZero, XMVectorReplicate(0.01f)));

		XMVECTOR vRes = XMVector3Rotate(V, Qr);
		Qr = XMQuaternionConjugate(Qr);
		XMVECTOR vTrans = XMQuaternionMultiply(Qr, Qe);
		//XMVECTOR vTrans = XMVectorSplatW(Qr) * Qe - Qr * XMVectorSplatW(Qe) + XMVector3Cross(Qr,Qe);
		vTrans *= g_XMTwo;

		vRes += vTrans;
		return vRes;
	}

	inline XMVECTOR XMVector3Displacement(FXMVECTOR V, CXMDUALVECTOR TransformDualQuaternion)
	{
		return XMVector3Displacement(V, TransformDualQuaternion.r[0], TransformDualQuaternion.r[1]);
	}

	const float XM_EPSILON = 1.192092896e-7f;

	inline float invsqrt(float x) {
		float xhalf = 0.5f*x;
		int i = *(int*) &x;
		i = 0x5f3759df - (i >> 1);
		x = *(float*) &i;
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

	template <typename _T, size_t _Align_Boundary = std::alignment_of<_T>::value>
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
		inline AlignedAllocator() throw() {}

		template <typename _T2>
		inline  AlignedAllocator(const AlignedAllocator<_T2, _Align_Boundary> &) throw() {}
		inline ~AlignedAllocator() throw() {}

		inline pointer adress(reference r)
		{
			return &r;
		}

		inline const_pointer adress(const_reference r) const
		{
			return &r;
		}

		inline pointer allocate(size_type n)
		{
			return (pointer) _mm_malloc(n*sizeof(value_type), _Align_Boundary);
		}

		inline void deallocate(pointer p, size_type)
		{
			_mm_free(p);
		}

		inline void construct(pointer p, const_reference wert)
		{
			::new(p) value_type(wert);
		}

		inline void destroy(pointer p)
		{ /* C4100 */ p; p->~value_type();
		}

		inline size_type max_size() const throw()
		{
			return size_type(-1) / sizeof(value_type);
		}

		template <typename _T2>
		struct rebind { typedef AlignedAllocator<_T2, _Align_Boundary> other; };
	};

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


			void Normarlize(DualQuaternion& result) const
			{
				XMDUALVECTOR dq = *this;
				dq = XMDualQuaternionNormalize(dq);
				result.Qr = dq.r[0];
				result.Qe = dq.r[1];
			}

			void Normarlize()
			{
				Normarlize(*this);
			}

			void Inverse( DualQuaternion& result) const
			{
				XMDUALVECTOR dq = *this;
				dq = XMDualQuaternionInverse(dq);
				result.Qr = dq.r[0];
				result.Qe = dq.r[1];
			}
			void Inverse()
			{
				Inverse(*this);
			}

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