#pragma once

// STL and std libs
#include <map>
#include <set>
#include <vector>
#include <list>
#include <memory>
#include <functional>
#include <chrono>
#include <type_traits>

// boost and std extension

#include <boost\signals2.hpp>
#include <boost\any.hpp>
#include <boost\range.hpp>
#include <boost\operators.hpp>
#include <boost\filesystem.hpp>

#include "Common\stride_range.h"
#include "Common\tree.h"

// Math libraries
#include <DirectXMath.h>
#include <DirectXMathSSE3.h>
#include <DirectXMathSSE4.h>
#include <DirectXMathAVX.h>
#include <DirectXCollision.h>

#include <SimpleMath.h>

#include "DirectXMathExtend.h"

#include <Eigen\Dense>
#include <Eigen\Sparse>

// ComPtr and task
#include <wrl/client.h>

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
	using Microsoft::WRL::ComPtr;

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

	// Binary operators
	using DirectX::operator +;
	using DirectX::operator -;
	using DirectX::operator /;
	using DirectX::operator *;
	using DirectX::operator *=;
	using DirectX::operator -=;
	using DirectX::operator +=;
	using DirectX::operator /=;

	template <class... TArgs>
	using Event = boost::signals2::signal<void(TArgs...)>;

	template <class TSender, class... TArgs>
	using TypedEvent = boost::signals2::signal<void(TSender*, TArgs...)>;

	using EventConnection = boost::signals2::connection;

	template <class TSender, class TCallback>
	auto MakeEventHandler(TCallback memberFuncPointer, TSender* sender) 
	{
		return std::bind(memberFuncPointer, sender, std::placeholders::_1);
	}

	template <class TArg, class TCallback>
	inline auto operator+=(Event<TArg>& signal, TCallback &&callback)
	{
		return signal.connect(std::move(callback));
	}

	template <class TArg, class TCallback>
	inline auto operator+=(Event<TArg>& signal, TCallback &callback)
	{
		return signal.connect(callback);
	}

	template <class TArg, class TCallback>
	inline void operator-=(Event<TArg>& signal, TCallback &&callback)
	{
		signal.disconnect(std::move(callback));
	}

	//Class template CompositeFlag
#include "CompositeFlag.h"
}