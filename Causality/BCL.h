#pragma once

// STL and std libs
#include <map>
#include <set>
#include <vector>
#include <list>
#include <memory>
#include <functional>
#include <chrono>

// boost and std extension
#include <boost\signals2.hpp>
#include <boost\any.hpp>
//#include <boost\range\any_range.hpp>
#include "Common\stride_range.h"
#include "Common\tree.h"

// Math libraries
#include "Common\DirectXMathExtend.h"
#include <Eigen\Dense>
#include <Eigen\Sparse>

// ComPtr and task
#define NOMINMAX
#include <wrl\client.h>
#include <ppltasks.h>

#ifndef _PCH_CPP_
// Extern template instantiation declearation
extern template class std::vector<int>;
extern template class std::vector<float>;
//extern template class Eigen::Matrix<float,-1,-1>;
//extern template class Eigen::Matrix<float,-1, 1>;
#endif

namespace Causality
{
	using time_seconds = std::chrono::duration<double>;
	typedef uint64_t id_t;
	using std::string;
	using boost::iterator_range;
	using boost::sub_range;
	using boost::any;
	using boost::any_cast;
	//using boost::any_range;
	namespace adaptors = boost::adaptors;
	using stdx::tree_node;
	using stdx::foward_tree_node;
	using stdx::stride_range;
	using stdx::stride_iterator;
	using std::vector;
	using std::map;
	using std::function;
	using std::unique_ptr;
	using std::shared_ptr;
	using std::list;
	using std::weak_ptr;

	using Concurrency::task;

	using DirectX::Vector2;
	using DirectX::Vector3;
	using DirectX::Vector4;
	using DirectX::Quaternion;
	using DirectX::Plane;
	using DirectX::Ray;
	using DirectX::Color;
	using DirectX::Matrix4x4;
	using DirectX::BoundingBox;
	using DirectX::BoundingOrientedBox;
	using DirectX::BoundingFrustum;
	using DirectX::BoundingSphere;

	using VectorX = Eigen::VectorXf;
	using RowVectorX = Eigen::RowVectorXf;
	using MatrixX = Eigen::MatrixXf;
	using Eigen::SparseMatrix;

	template <class T>
	using sptr = std::shared_ptr<T>;

	template <class T>
	using uptr = std::unique_ptr<T>;

	template <class T>
	using cptr = Microsoft::WRL::ComPtr<T>;

	struct Rect
	{
		Vector2 Position;
		Vector2 Size;

		float Top() const { return Position.y; }
		float Left() const { return Position.x; }
		float Bottom() const { return Position.y + Size.y; }
		float Right() const { return Position.x + Size.x; }
		float Width() const { return Size.x; }
		float Height() const { return Size.y; }
	};

	struct StaticPose
	{
	public:
		Quaternion	Orientation;
		Vector3		Position;
	};

	struct DynamicPose
	{
		Quaternion	Orientation;
		Vector3		Position;
		Vector3		AngularVelocity;
		Vector3		Velocity;
		Vector3		AngularAcceleration;
		Vector3		Acceleration;
		double		TimeInSeconds;         // Absolute time of this state sample.
	};

	// Binary operators
	using DirectX::operator+;
	using DirectX::operator-;
	using DirectX::operator/;
	using DirectX::operator*;

	template <class... TArgs>
	using Event = boost::signals2::signal<void(TArgs...)>;

	template <class TSender, class... TArgs>
	using TypedEvent = boost::signals2::signal<void(TSender*, TArgs...)>;

	using EventConnection = boost::signals2::connection;

	template <class TSender, class TCallback>
	auto MakeEventHandler(TCallback memberFuncPointer, TSender sender) -> decltype(std::bind(memberFuncPointer, sender, std::placeholders::_1))
	{
		return std::bind(memberFuncPointer, sender, std::placeholders::_1);
	}

	template <class TArg, class TCallback>
	inline auto operator+=(Event<TArg>& signal, TCallback &&callback) -> decltype(signal.connect(callback))
	{
		return signal.connect(std::move(callback));
	}

	template <class TArg, class TCallback>
	inline void operator-=(Event<TArg>& signal, TCallback &&callback)
	{
		signal.disconnect(std::move(callback));
	}

	//Class template CompositeFlag
	#include "CompositeFlag.h"

	//Vector2 operator* (const Vector2& V1, const Vector2& V2);
	//Vector2 operator* (const Vector2& V, float S);
	//Vector2 operator/ (const Vector2& V1, const Vector2& V2);
	//Vector2 operator* (float S, const Vector2& V);

	//typedef Eigen::Matrix<float, 2, 1, Eigen::DontAlign> Vector2f;
	//typedef Eigen::Matrix<float, 3, 1, Eigen::DontAlign> Vector3f;
	//typedef Eigen::Matrix<float, 4, 1, Eigen::DontAlign> Vector4f;
	// NOTE: The use of Eigen::DontAlign removes the guarantee that the statically-sized
	// Eigen matrix/vector types will be aligned to 16-byte memory boundaries and therefore
	// the corresponding matrix/vector operations aren't guaranteed to be vectorized.
	// See http://eigen.tuxfamily.org/dox/group__TopicUnalignedArrayAssert.html and
	// http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html for more info.

	// geometry storage types
	//typedef float MATH_TYPE;
	//// matrices
	//typedef Eigen::Matrix<MATH_TYPE, 1, 1, Eigen::DontAlign> Matrix1x1;
	//typedef Eigen::Matrix<MATH_TYPE, 2, 2, Eigen::DontAlign> Matrix2x2;
	//typedef Eigen::Matrix<MATH_TYPE, 2, 3, Eigen::DontAlign> Matrix2x3;
	//typedef Eigen::Matrix<MATH_TYPE, 3, 3, Eigen::DontAlign> Matrix3x3;
	//typedef Eigen::Matrix<MATH_TYPE, 3, 2, Eigen::DontAlign> Matrix3x2;
	//typedef Eigen::Matrix<MATH_TYPE, 4, 4, Eigen::DontAlign> Matrix4x4;
	//typedef Eigen::Matrix<MATH_TYPE, Eigen::Dynamic, Eigen::Dynamic> MatrixD;
	////typedef Eigen::Matrix<float, 2, 2, Eigen::DontAlign> Matrix2x2f;
	////typedef Eigen::Matrix<float, 3, 3, Eigen::DontAlign> Matrix3x3f;
	////typedef Eigen::Matrix<float, 4, 4, Eigen::DontAlign> Matrix4x4f;

	//// vectors
	////typedef Eigen::Matrix<MATH_TYPE, 1, 1, Eigen::DontAlign> Vector1;
	////typedef Eigen::Matrix<MATH_TYPE, 2, 1, Eigen::DontAlign> Vector2;
	////typedef Eigen::Matrix<MATH_TYPE, 3, 1, Eigen::DontAlign> Vector3;
	////typedef Eigen::Matrix<MATH_TYPE, 4, 1, Eigen::DontAlign> Vector4;
	//typedef Eigen::Matrix<MATH_TYPE, 5, 1, Eigen::DontAlign> Vector5;
	//typedef Eigen::Matrix<MATH_TYPE, 6, 1, Eigen::DontAlign> Vector6;
	//typedef Eigen::Matrix<MATH_TYPE, 7, 1, Eigen::DontAlign> Vector7;
	//typedef Eigen::Matrix<MATH_TYPE, 8, 1, Eigen::DontAlign> Vector8;
	//typedef Eigen::Matrix<MATH_TYPE, 9, 1, Eigen::DontAlign> Vector9;
	//typedef Eigen::Matrix<MATH_TYPE, 10, 1, Eigen::DontAlign> Vector10;
	//typedef Eigen::Matrix<MATH_TYPE, Eigen::Dynamic, 1> VectorD;

//namespace Eigen{
//	// standard library containers
//	template<typename _T>
//	using vector = std::vector<_T, Eigen::aligned_allocator<_T>>;
//
//	template<typename _T, typename _C = std::less<_T>>
//	using set = std::set<_T, _C, Eigen::aligned_allocator<_T>>;
//
//	template<typename _K, typename _V, typename _C = std::less<_K>>
//	using map = std::map<_K, _V, _C, Eigen::aligned_allocator<std::pair<_K, _V>>>;
//}

//namespace Math {
//	//legacy typedefs
//	template < class T >
//	template<typename _T>
//	using aligned_vector = std::vector<_T, Eigen::aligned_allocator<_T>>;
//	//typedef Eigen::vector<Vector2f> stdvectorV2f;
//	//typedef Eigen::vector<Vector3f> stdvectorV3f;
//	//typedef Eigen::vector<Vector4f> stdvectorV4f;
//}

//Marshaling functions
//NOTE:I really, really tried to make this a template function, but got stuck in template hell and did not
//have time to make it work.  This is actually fairly robust, so we'll use it untill someone more familiar
//with Eigen has the time to sort this out. --WG
//#define ProjectVector(_outDim, data) data.block<_outDim,1>(0,0)
}