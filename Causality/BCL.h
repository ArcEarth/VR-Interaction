#pragma once

// STL and std libs
#include <map>
#include <set>
#include <vector>
#include <list>
#include <memory>
#include <functional>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <type_traits>

// Guildline Support Library
#include <gsl.h>

//// boost and std extension
//#include <boost\signals2.hpp>
//#include <boost\any.hpp>
#include <boost\range.hpp>
#include <boost\operators.hpp>
#include <boost\format.hpp>
//#include <boost\filesystem.hpp>

#include "Common\stride_range.h"
#include "Common\tree.h"

// Math libraries
#include <DirectXMath.h>
#if defined(__SSE3__)
#include <DirectXMathSSE3.h>
#endif
#if defined(__SSE4__)
#include <DirectXMathSSE4.h>
#endif
#if defined(__AVX__)
#include <DirectXMathAVX.h>
#endif

#include <DirectXCollision.h>

#include <SimpleMath.h>

#include "DirectXMathExtend.h"

#if defined __AVX__
#undef __AVX__ //#error Eigen have problem with AVX now
#endif

#define EIGEN_HAS_CXX11_MATH 1
#define EIGEN_HAS_STD_RESULT_OF 1
#define EIGEN_HAS_VARIADIC_TEMPLATES 1
#include <Eigen\Dense>

// For ComPtr 
#include <wrl/client.h>

namespace Causality
{
	using time_seconds = std::chrono::duration<double>;
	typedef uint64_t id_t;

	using std::string;
	using boost::iterator_range;
	using boost::sub_range;

	namespace adaptors = boost::adaptors;

	//using boost::any;
	//using boost::any_cast;
	//using boost::any_range;

	using gsl::owner;
	using gsl::array_view;
	using gsl::byte;
	using gsl::string_view;
	using gsl::not_null;

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
	using DirectX::BoundingGeometry;

	using Microsoft::WRL::ComPtr;

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
}