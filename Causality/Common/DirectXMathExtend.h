#pragma once

#ifndef DX_MATH_EXT_H
#define DX_MATH_EXT_H

#if defined(BT_USE_SSE)
#error Bullet physics can not be include before DirectX Math headers.
#endif

// Disable bullet to overload operators on _m128
#ifndef BT_NO_SIMD_OPERATOR_OVERLOADS
#define BT_NO_SIMD_OPERATOR_OVERLOADS
#endif


#include <DirectXMath.h>
#include <DirectXMathSSE3.h>
#include <DirectXMathSSE4.h>
#include <DirectXMathAVX.h>
#include <DirectXCollision.h>
#include <SimpleMath.h>
#include <smmintrin.h>
#include <type_traits>
#include <boost\operators.hpp>

#ifndef XM_ALIGN16
#define XM_ALIGN16 _MM_ALIGN16
#endif
namespace DirectX
{
	namespace AlignedVector
	{


		XM_ALIGN16
		struct Vector3A : XMFLOAT3A
		{
			Vector3A() : XMFLOAT3A(0.f, 0.f, 0.f) {}
			Vector3A(FXMVECTOR V) { XMStoreFloat3A(this, V); }

			inline operator XMVECTOR() const { return XMLoadFloat3A(this); }

			inline Vector3A& XM_CALLCONV operator=(FXMVECTOR V) { XMStoreFloat3A(this, V); }
		};

		struct Vector4A : XMFLOAT4A
		{
			Vector4A() : XMFLOAT4A(0.f, 0.f, 0.f, 0.f) {}
			Vector4A(FXMVECTOR V) { XMStoreFloat4A(this, V); }

			inline operator XMVECTOR() const { return XMLoadFloat4A(this); }

			inline Vector4A& XM_CALLCONV operator=(FXMVECTOR V) { XMStoreFloat4A(this, V); }
		};
	}

	using SimpleMath::Vector2;
	using SimpleMath::Vector3;

	//#ifndef _Vector4_
	//#define _Vector4_
	using SimpleMath::Vector4;
	//#endif // _Vector4_

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
	XMGLOBALCONST XMVECTORF32 g_XMNegtiveXZ = { -1.0f,1.0f,-1.0f,1.0f };


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

#ifdef _MSC_VER
#pragma region XMDUALVECTOR
#endif

	struct XMDUALVECTOR;
	// Calling convetion

	// First XMDUALVECTOR
#if ( defined(_M_IX86) || defined(_M_ARM) || defined(_XM_VMX128_INTRINSICS_) || _XM_VECTORCALL_ ) && !defined(_XM_NO_INTRINSICS_)
	typedef const XMDUALVECTOR FXMDUALVECTOR;
#else
	typedef const XMDUALVECTOR& FXMDUALVECTOR;
#endif

	// 2nd
#if ( defined(_M_ARM) || defined(_XM_VMX128_INTRINSICS_) || (_XM_VECTORCALL_)) && !defined(_XM_NO_INTRINSICS_)
	typedef const XMDUALVECTOR GXMDUALVECTOR;
#else
	typedef const XMDUALVECTOR& GXMDUALVECTOR;
#endif
	// 3rd
#if ( defined(_XM_VMX128_INTRINSICS_) || _XM_VECTORCALL_ ) && !defined(_XM_NO_INTRINSICS_)
	typedef const XMDUALVECTOR HXMDUALVECTOR;
#else
	typedef const XMDUALVECTOR& HXMDUALVECTOR;
#endif

#if defined(_XM_VMX128_INTRINSICS_) && !defined(_XM_NO_INTRINSICS_)
	typedef const XMDUALVECTOR CXMDUALVECTOR;
#else
	typedef const XMDUALVECTOR& CXMDUALVECTOR;
#endif

#if (defined(_M_IX86) || defined(_M_AMD64) || defined(_M_ARM)) && defined(_XM_NO_INTRINSICS_)
	struct XMDUALVECTOR
#else
	__declspec(align(16)) struct XMDUALVECTOR
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
		XMDUALVECTOR(CXMVECTOR V0, CXMVECTOR V1) { r[0] = V0; r[1] = V1; }
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

		XMDUALVECTOR& XM_CALLCONV operator= (FXMDUALVECTOR DV) { r[0] = DV.r[0]; r[1] = DV.r[1]; return *this; }

		XMDUALVECTOR XM_CALLCONV operator+ () const { return *this; }
		XMDUALVECTOR XM_CALLCONV operator- () const
		{
			XMDUALVECTOR dvResult(r[0], -r[1]);
			return dvResult;
		}


		XMDUALVECTOR& XM_CALLCONV operator+= (FXMDUALVECTOR M)
		{
			r[0] += M.r[0];
			r[1] += M.r[1];
			return *this;
		}

		XMDUALVECTOR& XM_CALLCONV operator-= (FXMDUALVECTOR M)
		{
			r[0] -= M.r[0];
			r[1] -= M.r[1];
			return *this;
		}

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

#if defined(_XM_SSE_INTRINSICS_) 
	template <>
	inline XMVECTOR XM_CALLCONV XMVectorSwizzle<1, 1, 3, 3>(FXMVECTOR v)
	{
		return SSE3::XMVectorSwizzle_1133(v);
	}
	template <>
	inline XMVECTOR XM_CALLCONV XMVectorSwizzle<0, 0, 2, 2>(FXMVECTOR v)
	{
		return SSE3::XMVectorSwizzle_0022(v);
	}
	template <>
	inline XMVECTOR XM_CALLCONV XMVectorSwizzle<0, 0, 1, 1>(FXMVECTOR v)
	{
		return XMVectorMergeXY(v, v);
	}
	template <>
	inline XMVECTOR XM_CALLCONV XMVectorSwizzle<2, 2, 3, 3>(FXMVECTOR v)
	{
		return XMVectorMergeZW(v, v);
	}
#endif

	// Caculate the rotation quaternion base on v1 and v2 (shortest rotation geo-distance in sphere surface)
	inline XMVECTOR XM_CALLCONV XMQuaternionRotationVectorToVector(FXMVECTOR v1, FXMVECTOR v2) {
		assert(!XMVector3Equal(v1, XMVectorZero()));
		assert(!XMVector3Equal(v2, XMVectorZero()));
		XMVECTOR n1 = XMVector3Normalize(v1);
		XMVECTOR n2 = XMVector3Normalize(v2);
		XMVECTOR axias = XMVector3Cross(n1, n2);
		if (XMVector4NearEqual(axias, g_XMZero.v, g_XMEpsilon.v))
		{
			n2 = g_XMIdentityR0.v;
			axias = XMVector3Cross(n1, n2);
			if (XMVector4NearEqual(axias, g_XMZero.v, g_XMEpsilon.v))
			{
				n2 = g_XMIdentityR1.v;
				axias = XMVector3Cross(n1, n2);
			}
		}
		float angle = std::acosf(XMVectorGetX(XMVector3Dot(n1, n2)));
		auto rot = XMQuaternionRotationAxis(axias, angle);
		return rot;
	}

	enum AxisEnum
	{
		AxisX = 0,
		AxisY = 1,
		AxisZ = 2,
	};

	template <unsigned X, unsigned Y, unsigned Z>
	// Caculate Left handed eular angles with 
	// rotation sequence R(X)-R(Y)-R(Z)
	XMVECTOR XM_CALLCONV XMQuaternionEulerAngleABC(FXMVECTOR q);

	template <>
	// Caculate Left handed eular angles with 
	// rotation sequence R(Z)-R(Y)-R(X)
	inline XMVECTOR XM_CALLCONV XMQuaternionEulerAngleABC<2, 1, 0>(FXMVECTOR q)
	{
#if defined(_XM_NO_INTRINSICS_)
		float q0 = q.m128_f32[3], q1 = q.m128_f32[1], q2 = q.m128_f32[2], q3 = q.m128_f32[3];
		XMVECTOR euler;
		euler.m128_f32[0] = atan2(2 * (q0*q1 + q2*q3), 1 - 2 * (q1*q1 + q2*q2));
		euler.m128_f32[1] = asinf(2 * (q0*q2 - q3*q1));
		euler.m128_f32[2] = atan2(2 * (q0*q3 + q2*q1), 1 - 2 * (q2*q2 + q3*q3));
#elif defined(_XM_SSE_INTRINSICS_) 
		XMVECTOR q20 = XMVectorSwizzle<1, 1, 3, 3>(q);
		XMVECTOR q13 = XMVectorSwizzle<0, 0, 2, 2>(q);

		// eular X - Roll
		XMVECTOR X = SSE4::XMVector4Dot(q20, q13);
		XMVECTOR Y = XMVectorMergeXY(q13, q20);
		Y = SSE4::XMVector4Dot(Y, Y);
		Y = XMVectorSubtract(g_XMOne.v, Y);
		XMVECTOR vResult = XMVectorATan2(Y, X);

		// eular Z - X
		q13 = XMVectorSwizzle<2, 2, 0, 0>(q); // now 3311
		X = SSE4::XMVector4Dot(q20, q13);
		Y = XMVectorMergeXY(q13, q20);
		Y = SSE4::XMVector4Dot(Y, Y);
		Y = XMVectorSubtract(g_XMOne.v, Y);
		Y = XMVectorATan2(Y, X);

		// eular Y - Patch
		X = XMVectorSwizzle<2, 3, 0, 1>(q);
		X = XMVectorMultiply(X, g_XMNegtiveXZ.v);
		X = SSE4::XMVector4Dot(X, q);

		// merge result
		Y = XMVectorMergeXY(Y, X);
		vResult = XMVectorSelect(Y, vResult, g_XMSelect1000.v);
		vResult = XMVectorAndInt(vResult, g_XMMask3.v);
		return vResult;
#endif
	}

	namespace Internal
	{
		template <unsigned _X, unsigned _Y, unsigned _Z>
		struct SwizzleInverse
		{
		};
		template <>
		struct SwizzleInverse<0, 1, 2>
		{
			static const unsigned X = 0;
			static const unsigned Y = 1;
			static const unsigned Z = 2;
		};
		template <>
		struct SwizzleInverse<1, 2, 0>
		{
			static const unsigned X = 2;
			static const unsigned Y = 0;
			static const unsigned Z = 1;
		};
		template <>
		struct SwizzleInverse<2, 1, 0>
		{
			static const unsigned X = 2;
			static const unsigned Y = 1;
			static const unsigned Z = 0;
		};
		template <>
		struct SwizzleInverse<1, 0, 2>
		{
			static const unsigned X = 1;
			static const unsigned Y = 0;
			static const unsigned Z = 2;
		};
		template <>
		struct SwizzleInverse<0, 2, 1>
		{
			static const unsigned X = 0;
			static const unsigned Y = 2;
			static const unsigned Z = 1;
		};
		template <>
		struct SwizzleInverse<2, 0, 1>
		{
			static const unsigned X = 1;
			static const unsigned Y = 2;
			static const unsigned Z = 0;
		};
	}

	template <unsigned X, unsigned Y, unsigned Z >
	// Caculate Left handed eular angles with 3 different rotation axis
	// rotation sequence R(X)-R(Y)-R(Z)	
	inline XMVECTOR XM_CALLCONV XMQuaternionEulerAngleABC(FXMVECTOR q)
	{
		static_assert(X <= 2 && Y <= 2 && Z <= 2 && X != Y && Z != Y && X != Z, "X-Y-Z must be different Axis");

		XMVECTOR qs = XMVectorSwizzle<Z, Y, X, 3>(q);
		qs = XMQuaternionEulerAngleABC<2, 1, 0>(qs);
		using InvS = Internal::SwizzleInverse<X, Y, Z>;
		qs = XMVectorSwizzle<InvS::X, InvS::Y, InvS::Z, 3>(qs);
		return qs;
	}

	template <unsigned X, unsigned Y>
	// Caculate Left handed eular angles with 2 different rotation axis
	// rotation sequence R(X)-R(Y)-R(X)	
	inline XMVECTOR XM_CALLCONV XMQuaternionEulerAngleABA(FXMVECTOR q)
	{
	}

	// Left handed eular angles
	// rotation sequence R(Y)-R(X)-R(Z), so called Yaw-Pitch-Roll
	// return : <R(X),R(Y),R(Z),0>
	inline XMVECTOR XM_CALLCONV XMQuaternionEulerAngleYXZ(FXMVECTOR q)
	{
		return XMQuaternionEulerAngleABC<AxisY, AxisX, AxisZ>(q);
	}

	// left handed eular angles : <Pitch,Yaw,Roll,0>, ready with XMQuaternionRotationYawPitchRoll
	// rotation sequence R(Y)-R(X)-R(Z), so called Yaw-Pitch-Roll
	inline XMVECTOR XM_CALLCONV XMQuaternionEulerAngleYawPitchRoll(FXMVECTOR q)
	{
		return XMQuaternionEulerAngleABC<AxisY, AxisX, AxisZ>(q);
	}

	inline XMVECTOR XM_CALLCONV XMQuaternionRotationRollPitchYawRH
	(
		float Pitch,
		float Yaw,
		float Roll
	)
	{
		XMVECTOR Angles = XMVectorSet(Pitch, Yaw, Roll, 0.0f);
		Angles = XMVectorNegate(Angles);
		return XMQuaternionRotationRollPitchYawFromVector(Angles);
	}

	inline XMVECTOR XM_CALLCONV XMQuaternionRotationRollPitchYawFromVectorRH
	(
		FXMVECTOR eular //<Pitch, Yaw, Roll, 0>
	)
	{
		XMVECTOR Angles = XMVectorNegate(eular);
		return XMQuaternionRotationRollPitchYawFromVector(Angles);
	}

	// This function garuntee the extends of the bounding box is sorted from bigger to smaller
	inline void CreateBoundingOrientedBoxFromPoints(_Out_ BoundingOrientedBox& Out, _In_ size_t Count,
		_In_reads_bytes_(sizeof(XMFLOAT3) + Stride*(Count - 1)) const XMFLOAT3* pPoints, _In_ size_t Stride) {
		using namespace std;
		BoundingOrientedBox::CreateFromPoints(Out, Count, pPoints, Stride);
		XMMATRIX rot = XMMatrixIdentity();
		auto& ext = Out.Extents;

		// Sort the demension
		if (ext.x < ext.y)
		{
			swap(ext.x, ext.y);
			swap(rot.r[0], rot.r[1]);
			rot.r[2] = -rot.r[2]; // Flip the other axias to maintain the chirality
		}
		if (ext.y < ext.z)
		{
			swap(ext.y, ext.z);
			swap(rot.r[1], rot.r[2]);
			rot.r[0] = -rot.r[0];
		}
		if (ext.x < ext.y)
		{
			swap(ext.x, ext.y);
			swap(rot.r[0], rot.r[1]);
			rot.r[2] = -rot.r[2];
		}

		//qw = sqrt(1 + m00 + m11 + m22) / 2
		//qx = (m21 - m12) / (4 * qw)
		//qy = (m02 - m20) / (4 * qw)
		//qz = (m10 - m01) / (4 * qw)
		XMVECTOR q = XMQuaternionRotationMatrix(rot);
		XMStoreFloat4(&Out.Orientation, XMQuaternionMultiply(q, XMLoadFloat4(&Out.Orientation)));
		//auto q = XMQuaternionRotationMatrix(rot);
	}

	XMVECTOR XM_CALLCONV XMVector3Displacement(FXMVECTOR V, FXMVECTOR RotationQuaternion, FXMVECTOR TranslationQuaternion);

	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionTranslation(FXMVECTOR T)
	{
		static const XMVECTORF32 Control = { 0.5f,0.5f,0.5f,.0f };
		XMVECTOR Qe = XMVectorMultiply(T, Control);
		XMVECTOR Qr = XMQuaternionIdentity();
		XMDUALVECTOR dqRes;
		dqRes.r[0] = Qr;
		dqRes.r[1] = Qe;
		return dqRes;
	}

	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionRotation(FXMVECTOR Q)
	{
		XMDUALVECTOR dqRes;
		dqRes.r[0] = Q;
		dqRes.r[1] = XMVectorZero();
	}

	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionRigidTransform(FXMVECTOR RotationOrigin, FXMVECTOR RotationQuaternion, FXMVECTOR Translation)
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

	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionRotationTranslation(FXMVECTOR Q, FXMVECTOR T)
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

	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionNormalize(FXMDUALVECTOR Dq)
	{
		XMVECTOR Length = XMQuaternionLength(Dq.r[0]);
		XMDUALVECTOR dqRes;
		dqRes.r[0] = XMVectorDivide(Dq.r[0], Length);
		dqRes.r[1] = XMVectorDivide(Dq.r[1], Length);
		return dqRes;
	}
	inline XMVECTOR XM_CALLCONV XMDualQuaternionNormalizeEst(FXMDUALVECTOR Dq)
	{
		return XMVector4NormalizeEst(Dq.r[0]);
	}
	inline XMVECTOR XM_CALLCONV XMDualQuaternionLength(FXMDUALVECTOR Dq)
	{
		return XMVector4Length(Dq.r[0]);
	}
	inline XMVECTOR XM_CALLCONV XMDualQuaternionLengthSq(FXMDUALVECTOR Dq)
	{
		return XMVector4LengthSq(Dq.r[0]);
	}
	inline XMVECTOR XM_CALLCONV XMDualQuaternionLengthEst(FXMDUALVECTOR Dq)
	{
		return XMVector4LengthEst(Dq.r[0]);
	}
	inline XMDUALVECTOR XM_CALLCONV XMDualVectorConjugate(FXMDUALVECTOR Dq)
	{
		XMDUALVECTOR dvResult(Dq.r[0], XMVectorNegate(Dq.r[1]));
		return dvResult;
	}
	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionConjugate(FXMDUALVECTOR Dq)
	{
		XMDUALVECTOR dvResult;
		dvResult.r[0] = XMQuaternionConjugate(Dq.r[0]);
		dvResult.r[1] = XMQuaternionConjugate(Dq.r[1]);
		return dvResult;
	}

	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionInverse(FXMDUALVECTOR Dq)
	{
		XMDUALVECTOR dvResult = XMDualQuaternionConjugate(Dq);
		dvResult.r[0] = XMQuaternionNormalize(dvResult.r[0]);
	}

	inline XMDUALVECTOR XM_CALLCONV XMDualQuaternionMultipy(FXMDUALVECTOR DQ0, GXMDUALVECTOR DQ1)
	{
		XMDUALVECTOR dvResult;
		dvResult.r[0] = XMQuaternionMultiply(DQ0.r[0], DQ1.r[1]);
		dvResult.r[1] = XMQuaternionMultiply(DQ0.r[0], DQ1.r[1]);
		dvResult.r[1] += XMQuaternionMultiply(DQ0.r[1], DQ1.r[0]);
		return dvResult;
	}

	inline bool XM_CALLCONV XMDualQuaternionDecompose(XMVECTOR* outRotQuat, XMVECTOR* outTrans, FXMDUALVECTOR Dq)
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
		vT = XMVectorAndInt(Q, g_XMMaskX.v);

		Q = XMVectorSwizzle<2, 3, 0, 1>(Qr);
		Q = XMVectorMultiply(Qe, Q);
		Q = XMVector4Dot(Q, ControlY.v);
		Q = XMVectorAndInt(Q, g_XMMaskY.v);
		vT = XMVectorOrInt(vT, Q);

		Q = XMVectorSwizzle<1, 0, 3, 2>(Qr);
		Q = XMVectorMultiply(Qe, Q);
		Q = XMVector4Dot(Q, ControlZ.v);
		Q = XMVectorAndInt(Q, g_XMMaskZ.v);
		vT = XMVectorOrInt(vT, Q);

		*outRotQuat = Qr;
		*outTrans = vT;
		return true;
	}

	inline XMVECTOR XM_CALLCONV XMVector3Displacement(FXMVECTOR V, FXMVECTOR RotationQuaternion, FXMVECTOR TranslationQuaternion)
	{
		XMVECTOR Qr = RotationQuaternion;
		XMVECTOR Qe = TranslationQuaternion;
		XMVECTOR Dual = XMVector4Dot(Qr, Qe);
		assert(XMVector4NearEqual(Dual, XMVectorZero(), XMVectorReplicate(0.01f)));

		XMVECTOR vRes = XMVector3Rotate(V, Qr);
		Qr = XMQuaternionConjugate(Qr);
		XMVECTOR vTrans = XMQuaternionMultiply(Qr, Qe);
		//XMVECTOR vTrans = XMVectorSplatW(Qr) * Qe - Qr * XMVectorSplatW(Qe) + XMVector3Cross(Qr,Qe);
		vTrans *= g_XMTwo.v;

		vRes += vTrans;
		return vRes;
	}

	inline XMVECTOR XM_CALLCONV XMVector3Displacement(FXMVECTOR V, CXMDUALVECTOR TransformDualQuaternion)
	{
		return XMVector3Displacement(V, TransformDualQuaternion.r[0], TransformDualQuaternion.r[1]);
	}

#ifdef _MSC_VER
#pragma endregion
#endif

	const float XM_EPSILON = 1.192092896e-7f;

	inline float XMScalarReciprocalSqrtEst(float x) {
		float xhalf = 0.5f*x;
		int i = *(int*) &x;
		i = 0x5f3759df - (i >> 1);
		x = *(float*) &i;
		x = x*(1.5f - xhalf*x*x);
		return x;
	}

	inline XMMATRIX XMMatrixZero()
	{
		XMMATRIX M;
		M.r[0] = XMVectorZero();
		M.r[1] = XMVectorZero();
		M.r[2] = XMVectorZero();
		M.r[3] = XMVectorZero();
		return M;
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

		template <size_t _Other_Alignment>
		inline bool operator== (const AlignedAllocator<_T, _Other_Alignment> & rhs) const
		{
			return _Align_Boundary == _Other_Alignment;
		}



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
		, boost::multiplicative<DualQuaternion, float>
	{
	public:
		DirectX::Quaternion Qr, Qe;
		DualQuaternion()
			: Qr(), Qe(0.0f, 0.0f, 0.0f, 0.0f)
		{}
		DualQuaternion(const Quaternion& Rotation, const Vector3& Translation)
			: Qr(Rotation)
		{
			Qr.Normalize();
			const float* Q0 = reinterpret_cast<float*>(&Qr);
			const float* T = reinterpret_cast<const float*>(&Translation);
			Qe.w = -0.5f*(T[0] * Q0[0] + T[1] * Q0[1] + T[2] * Q0[2]);
			Qe.x = 0.5f*(T[0] * Q0[3] + T[1] * Q0[2] - T[2] * Q0[1]);
			Qe.y = 0.5f*(-T[0] * Q0[2] + T[1] * Q0[3] + T[2] * Q0[0]);
			Qe.z = 0.5f*(T[0] * Q0[1] - T[1] * Q0[0] + T[2] * Q0[3]);
		}
		DualQuaternion(FXMVECTOR Qr, FXMVECTOR Qe)
			: Qr(Qr), Qe(Qe)
		{}
		DualQuaternion(CXMDUALVECTOR DQ)
			: Qr(DQ.r[0]), Qe(DQ.r[1])
		{}

		explicit DualQuaternion(_In_reads_(8) const float* pArray)
			: Qr(pArray), Qe(pArray + 4)
		{}
		explicit DualQuaternion(_In_reads_(2) const Quaternion* pQArray)
			: Qr(*pQArray), Qe(*(pQArray + 1))
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

		void Inverse(DualQuaternion& result) const
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

		void Conjugate(DualQuaternion& result) const
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
			q1 = XMVector4Dot(q0, q1);
			q0 = XMVectorDivide(q1, len);
			q1 = XMVectorSelect(len, q0, g_XMSelect0101);
			value = q1;
			return value;
		}

		bool IsUnit() const
		{
			XMVECTOR q0 = Qr;
			XMVECTOR q1 = Qe;
			q1 = XMVector4Dot(q0, q1);
			return XMVector4NearEqual(q0, g_XMZero.v, g_XMEpsilon.v);
		}

		bool Decompose(Quaternion& Rotation, Vector3& Translation) const
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
			D = XMQuaternionMultiply(A, D);
			B = XMQuaternionMultiply(B, C);
			Qe = XMVectorAdd(D, B);
			Qr = XMQuaternionMultiply(A, C);
		}

		DualQuaternion& operator*= (float scale)
		{
			Qr *= scale;
			Qr *= scale;
			return *this;
		}

		DualQuaternion& operator/= (float scale)
		{
			float s = 1.0f / scale;
			return (*this) *= s;
		}
	};

	//typedef std::pair<Vector3, Vector3> LineSegement;
	//typedef std::vector<Vector3> LinePath;
	static const float g_INFINITY = 1e12f;

	inline XMMATRIX XM_CALLCONV XMMatrixScalingFromCenter(FXMVECTOR scale, FXMVECTOR center)
	{
		XMMATRIX m = XMMatrixScalingFromVector(scale);
		m.r[3] = XMVectorSubtract(g_XMOne.v, scale);
		m.r[3] *= center;
		m.r[3] = XMVectorSelect(g_XMOne.v, m.r[3], g_XMSelect1110.v);
		return m;
	}

	// No Matrix Multiply inside, significant faster than affine transform
	inline XMMATRIX XM_CALLCONV XMMatrixRigidTransform(FXMVECTOR RotationOrigin, FXMVECTOR RotationQuaterion, FXMVECTOR Translation)
	{
		XMVECTOR VTranslation = XMVector3Rotate(RotationOrigin, RotationQuaterion);
		VTranslation = XMVectorAdd(VTranslation, RotationOrigin);
		VTranslation = XMVectorAdd(VTranslation, VTranslation);
		VTranslation = XMVectorSelect(g_XMSelect1110.v, Translation, g_XMSelect1110.v);
		XMMATRIX M = XMMatrixRotationQuaternion(RotationQuaterion);
		M.r[3] = XMVectorAdd(M.r[3], VTranslation);
		return M;
	}

	// No Matrix Multiply inside, significant faster than affine transform
	inline XMMATRIX XM_CALLCONV XMMatrixRigidTransform(FXMVECTOR RotationQuaterion, FXMVECTOR Translation)
	{
		XMMATRIX M = XMMatrixRotationQuaternion(RotationQuaterion);
		XMVECTOR VTranslation = XMVectorSelect(g_XMSelect1110.v, Translation, g_XMSelect1110.v);
		M.r[3] = XMVectorAdd(M.r[3], VTranslation);
		return M;
	}


	// Composition of Translation and Rotation
	struct RigidTransform
	{
	public:
		Quaternion  Rotation;
		Vector3		Translation;

		RigidTransform()
		{}

		// Extract the Matrix Representation of this rigid transform
		inline XMMATRIX TransformMatrix() const
		{
			XMMATRIX M = XMMatrixRotationQuaternion(Rotation);
			XMVECTOR VTranslation = XMVectorSelect(g_XMSelect1110.v, Translation, g_XMSelect1110.v);
			M.r[3] = XMVectorAdd(M.r[3], VTranslation);
			return M;
		}

		inline void XM_CALLCONV SetFromTransformMatrix(FXMMATRIX transform)
		{
			XMVECTOR scl, rot, tra;
			XMMatrixDecompose(&scl, &rot, &tra, transform);
			Rotation = rot;
			Translation = tra;
		}

		template <typename _TTransform>
		RigidTransform& operator *=(const _TTransform& transform)
		{
			XMMATRIX mat = TransformMatrix();
			XMMATRIX mat2 = transform.TransformMatrix();
			mat *= mat2;
			SetFromTransformMatrix(mat);
			return *this;
		}


		// Extract the Dual-Quaternion Representation of this rigid transform
		inline XMDUALVECTOR TransformDualQuaternion() const
		{
			return XMDualQuaternionRotationTranslation(Rotation, Translation);
		}

		bool NearEqual(const RigidTransform& rhs, float tEpsilon = 0.002f, float rEpsilon = 0.5f) const
		{
			Vector3 PosDiff = Translation - rhs.Translation;
			XMVECTOR RotDiff = Rotation;
			RotDiff = XMQuaternionInverse(RotDiff);
			RotDiff = XMQuaternionMultiply(RotDiff, rhs.Rotation);
			float AngDiff = 2 * acosf(XMVectorGetW(RotDiff));
			return (PosDiff.LengthSquared() <= tEpsilon*tEpsilon && AngDiff <= rEpsilon);
		}
	};

	// Composition of Translation/Rotation/Scale
	struct AffineTransform : public RigidTransform
	{
		static AffineTransform Identity()
		{
			return AffineTransform();
		}

		Vector3 Scale;

		AffineTransform()
			: Scale(1.0f)
		{}

		inline explicit AffineTransform(CXMMATRIX transform)
		{
			SetFromTransformMatrix(transform);
		}

		inline void XM_CALLCONV SetFromTransformMatrix(FXMMATRIX transform)
		{
			XMVECTOR scl, rot, tra;
			XMMatrixDecompose(&scl, &rot, &tra, transform);
			Scale = scl;
			Rotation = rot;
			Translation = tra;
		}

		template <typename _TTransform>
		AffineTransform& operator *=(const _TTransform& transform)
		{
			XMMATRIX mat = TransformMatrix();
			XMMATRIX mat2 = transform.TransformMatrix();
			mat *= mat2;
			SetFromTransformMatrix(mat);
			return *this;
		}

		inline XMMATRIX TransformMatrix() const
		{
			XMMATRIX M = XMMatrixScalingFromVector(Scale);
			M *= XMMatrixRotationQuaternion(Rotation);
			XMVECTOR VTranslation = XMVectorSelect(g_XMSelect1110.v, Translation, g_XMSelect1110.v);
			M.r[3] = XMVectorAdd(M.r[3], VTranslation);
			return M;
		}

		explicit operator XMMATRIX() const
		{
			return TransformMatrix();
		}
	};

	struct MatrixTransform : public Matrix4x4
	{
		using Matrix4x4::operator();
		using Matrix4x4::operator+=;
		using Matrix4x4::operator-=;
		using Matrix4x4::operator*=;
		using Matrix4x4::operator/=;
		using Matrix4x4::operator-;
		using Matrix4x4::operator DirectX::XMMATRIX;

		inline void XM_CALLCONV SetFromTransformMatrix(FXMMATRIX transform)
		{
			*this = transform;
		}

		MatrixTransform& operator=(const Matrix4x4& rhs)
		{
			Matrix4x4::operator=(rhs);
		}

		template <typename _TTransform>
		MatrixTransform& operator *=(const _TTransform& transform)
		{
			XMMATRIX mat = TransformMatrix();
			XMMATRIX mat2 = transform.TransformMatrix();
			mat *= mat2;
			SetFromTransformMatrix(mat);
			return *this;
		}

		inline XMMATRIX TransformMatrix() const
		{
			return XMLoadFloat4x4(this);
		}
	};

	namespace BoundingFrustumExtension
	{
		inline void CreateFromMatrixRH(BoundingFrustum& Out, CXMMATRIX Projection)
		{
			// Corners of the projection frustum in homogenous space.
			static XMVECTORF32 HomogenousPoints[6] =
			{
				{ 1.0f,  0.0f, -1.0f, 1.0f },   // right (at far plane)
				{ -1.0f,  0.0f, -1.0f, 1.0f },   // left
				{ 0.0f,  1.0f, -1.0f, 1.0f },   // top
				{ 0.0f, -1.0f, -1.0f, 1.0f },   // bottom

				{ 0.0f, 0.0f, 1.0f, 1.0f },    // far
				{ 0.0f, 0.0f, 0.0f, 1.0f }     // near
			};

			XMVECTOR Determinant;
			XMMATRIX matInverse = XMMatrixInverse(&Determinant, Projection);

			// Compute the frustum corners in world space.
			XMVECTOR Points[6];

			for (size_t i = 0; i < 6; ++i)
			{
				// Transform point.
				Points[i] = XMVector4Transform(HomogenousPoints[i], matInverse);
			}

			Out.Origin = XMFLOAT3(0.0f, 0.0f, 0.0f);
			Out.Orientation = XMFLOAT4(0.0f, 0.0f, 0.0f, 1.0f);

			// Compute the slopes.
			Points[0] = Points[0] * XMVectorReciprocal(XMVectorSplatZ(Points[0]));
			Points[1] = Points[1] * XMVectorReciprocal(XMVectorSplatZ(Points[1]));
			Points[2] = Points[2] * XMVectorReciprocal(XMVectorSplatZ(Points[2]));
			Points[3] = Points[3] * XMVectorReciprocal(XMVectorSplatZ(Points[3]));

			Out.RightSlope = XMVectorGetX(Points[0]);
			Out.LeftSlope = XMVectorGetX(Points[1]);
			Out.TopSlope = XMVectorGetY(Points[2]);
			Out.BottomSlope = XMVectorGetY(Points[3]);

			// Compute near and far.
			Points[4] = Points[4] * XMVectorReciprocal(XMVectorSplatW(Points[4]));
			Points[5] = Points[5] * XMVectorReciprocal(XMVectorSplatW(Points[5]));

			Out.Near = XMVectorGetZ(Points[4]);
			Out.Far = XMVectorGetZ(Points[5]);
		}
	}

	//Some supporting method

	namespace LineSegmentTest
	{
		inline float XM_CALLCONV Distance(FXMVECTOR p, FXMVECTOR s0, FXMVECTOR s1)
		{
			XMVECTOR s = s1 - s0;
			XMVECTOR v = p - s0;
			auto Ps = XMVector3Dot(v, s);
			//p-s0 is the shortest distance
			if (XMVector4LessOrEqual(Ps, XMVectorZero()))
				return XMVectorGetX(XMVector3Length(v));

			auto Ds = XMVector3LengthSq(s);
			//p-s1 is the shortest distance
			if (XMVector4Greater(Ps, Ds))
				return XMVectorGetX(XMVector3Length(p - s1));
			//find the projection point on line segment U
			return XMVectorGetX(XMVector3Length(v - s * (Ps / Ds)));
		}

		// Takes a space point and space line segment , return the projection point on the line segment
		//  A0  |		A1		  |
		//      |s0-------------s1|
		//      |				  |		A2
		// where p in area of A0 returns s0 , area A2 returns s1 , point in A1 returns the really projection point 
		inline XMVECTOR XM_CALLCONV Projection(FXMVECTOR p, FXMVECTOR s0, FXMVECTOR s1)
		{
			XMVECTOR s = s1 - s0;
			XMVECTOR v = p - s0;
			XMVECTOR Ps = SSE4::XMVector3Dot(v, s);

			//p-s0 is the shortest distance
			//if (Ps<=0.0f) 
			//	return s0;
			if (XMVector4LessOrEqual(Ps, g_XMZero.v))
				return s0;
			XMVECTOR Ds = SSE4::XMVector3LengthSq(s);
			//p-s1 is the shortest distance
			//if (Ps>Ds) 	
			//	return s1;
			if (XMVector4Less(Ds, Ps))
				return s1;
			//find the projection point on line segment U
			return (s * (Ps / Ds)) + s0;
		}

		//inline XMVECTOR XM_CALLCONV Projection(FXMVECTOR p, XMFLOAT3A *path, size_t nPoint)
		//{
		//	const auto N = nPoint;
		//	XMVECTOR vBegin = XMLoadFloat3A(path);
		//	XMVECTOR vEnd = XMLoadFloat3A(path + 1);
		//	XMVECTOR vMinProj = Projection(p, vBegin, vEnd);
		//	XMVECTOR vMinDis = XMVector3LengthSq(p - vMinProj);
		//	vBegin = vEnd;

		//	for (size_t i = 2; i < N - 1; i++)
		//	{
		//		vEnd = XMLoadFloat3A(path + i);
		//		XMVECTOR vProj = Projection(p, vBegin, vEnd);
		//		XMVECTOR vDis = XMVector3LengthSq(p - vProj);
		//		if (XMVector4LessOrEqual(vDis, vMinDis))
		//		{
		//			vMinDis = vDis;
		//			vMinProj = vProj;
		//		}
		//		vBegin = vEnd;
		//	}

		//	return vMinProj;
		//}

		//inline XMVECTOR XM_CALLCONV Projection(FXMVECTOR p, XMFLOAT3 *path, size_t nPoint)
		//{
		//	const auto N = nPoint;
		//	XMVECTOR vBegin = XMLoadFloat3(path);
		//	XMVECTOR vEnd = XMLoadFloat3(path + 1);
		//	XMVECTOR vMinProj = Projection(p, vBegin, vEnd);
		//	XMVECTOR vMinDis = XMVector3LengthSq(p - vMinProj);
		//	vBegin = vEnd;

		//	for (size_t i = 2; i < N - 1; i++)
		//	{
		//		vEnd = XMLoadFloat3(path + i);
		//		XMVECTOR vProj = Projection(p, vBegin, vEnd);
		//		XMVECTOR vDis = XMVector3LengthSq(p - vProj);
		//		if (XMVector4LessOrEqual(vDis, vMinDis))
		//		{
		//			vMinDis = vDis;
		//			vMinProj = vProj;
		//		}
		//		vBegin = vEnd;
		//	}

		//	return vMinProj;
		//}

		template <typename T>
		inline XMVECTOR XM_CALLCONV Projection(FXMVECTOR p, const T *path, size_t nPoint, size_t strideInByte = sizeof(T))
		{
			if (!(strideInByte % 16) && !(reinterpret_cast<intptr_t>(path) & 0x16))
			{
				const auto N = nPoint;
				auto p = reinterpret_cast<const char*>(path) + strideInByte;
				XMVECTOR vBegin = XMLoadFloat3A(reinterpret_cast<const XMFLOAT3A*>(path));
				XMVECTOR vEnd = XMLoadFloat3A(reinterpret_cast<const XMFLOAT3A*>(p));
				XMVECTOR vMinProj = Projection(p, vBegin, vEnd);
				XMVECTOR vMinDis = SSE4::XMVector3LengthSq(p - vMinProj);
				vBegin = vEnd;

				for (size_t i = 2; i < N - 1; i++)
				{
					p += strideInByte;
					vEnd = XMLoadFloat3A(reinterpret_cast<const XMFLOAT3A*>(path));
					XMVECTOR vProj = Projection(p, vBegin, vEnd);
					XMVECTOR vDis = SSE4::XMVector3LengthSq(p - vProj);
					if (XMVector4LessOrEqual(vDis, vMinDis))
					{
						vMinDis = vDis;
						vMinProj = vProj;
					}
					vBegin = vEnd;
				}

				return vMinProj;
			}
			else
			{
				const auto N = nPoint;
				XMVECTOR vBegin = XMLoadFloat3(reinterpret_cast<const XMFLOAT3*>(path));
				XMVECTOR vEnd = XMLoadFloat3(reinterpret_cast<const XMFLOAT3*>(p));
				XMVECTOR vMinProj = Projection(p, vBegin, vEnd);
				XMVECTOR vMinDis = XMVector3LengthSq(p - vMinProj);
				vBegin = vEnd;

				auto p = reinterpret_cast<const char*>(path) + strideInByte;
				for (size_t i = 2; i < N - 1; i++)
				{
					p += strideInByte;
					vEnd = XMLoadFloat3(reinterpret_cast<const XMFLOAT3*>(path));
					XMVECTOR vProj = Projection(p, vBegin, vEnd);
					XMVECTOR vDis = XMVector3LengthSq(p - vProj);
					if (XMVector4LessOrEqual(vDis, vMinDis))
					{
						vMinDis = vDis;
						vMinProj = vProj;
					}
					vBegin = vEnd;
				}

				return vMinProj;
			}
		}
	}
}

// Extending std lib for output
#ifdef _OSTREAM_
#include <iomanip>
#endif

namespace std
{
	inline std::ostream& operator << (std::ostream& lhs, const DirectX::XMFLOAT2& rhs)
	{
		lhs << '(' << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.x
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << rhs.y << ')';
		return lhs;
	};

	inline std::ostream& operator << (std::ostream& lhs, const DirectX::XMFLOAT3& rhs)
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
		axis.Normalize();
		lhs << '(' << axis
			<< "," << std::setw(6) << setiosflags(std::ios::fixed) << std::setprecision(3) << theta << "*Pi)";
		return lhs;
	};
}
#endif