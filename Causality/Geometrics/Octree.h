#pragma once
#include "DirectXMathExtend.h"

namespace Geometrics {

	XM_ALIGNATTR
	struct Aabb : public DirectX::AlignedNew<Aabb>
	{
		DirectX::Vector4 Min;
		DirectX::Vector4 Max;
	};

	template <class _T, class _AabbT>
	class Octree
	{
		size_t		_depth;
		Octree*[8]	_children;

		int GetIntersectedObjects();
	};
}