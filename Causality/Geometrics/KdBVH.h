#pragma once
#include "DirectXMathExtend.h"
#include <vector>
#include <algorithm>
#include <Eigen\Core>

namespace Geometrics {

	namespace internal {
		//internal pair class for the BVH--used instead of std::pair because of alignment
		template<typename Scalar, int Dim>
		struct vector_int_pair
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF_VECTORIZABLE_FIXED_SIZE(Scalar, Dim)
				typedef Matrix<Scalar, Dim, 1> VectorType;

			vector_int_pair(const VectorType &v, int i) : first(v), second(i) {}

			VectorType first;
			int second;
		};

		//these templates help the tree initializer get the bounding boxes either from a provided
		//iterator range or using bounding_box in a unified way
		template<typename ObjectList, typename VolumeList, typename BoxIter>
		struct get_boxes_helper {
			void operator()(const ObjectList &objects, BoxIter boxBegin, BoxIter boxEnd, VolumeList &outBoxes)
			{
				outBoxes.insert(outBoxes.end(), boxBegin, boxEnd);
				eigen_assert(outBoxes.size() == objects.size());
			}
		};

		template<typename ObjectList, typename VolumeList>
		struct get_boxes_helper<ObjectList, VolumeList, int> {
			void operator()(const ObjectList &objects, int, int, VolumeList &outBoxes)
			{
				outBoxes.reserve(objects.size());
				for (int i = 0; i < (int)objects.size(); ++i)
					outBoxes.push_back(bounding_box(objects[i]));
			}
		};
	} // end namespace internal


	template <typename _TScale, size_t _Dim>
	struct Aabb /*: public DirectX::AlignedNew<Aabb>*/
	{
		DirectX::Vector4 Min;
		DirectX::Vector4 Max;
	};

	const int boxSize = sizeof(DirectX::BoundingBox);
	const int k = sizeof(std::vector<DirectX::BoundingGeometry>);

	template <typename _TScale, size_t _Dim, typename _TObject, class _TAabb = Aabb<_TScale, _Dim>>
	class AabbTree 
	{
	public:
		using typename DirectX::BoundingBox;
		using typename DirectX::XMAllocator;

		typedef int Index;
		typedef _TAabb AabbType;
		typedef _TObject ObjectType;
		typedef AabbType Volume;
		typedef ObjectType Object;

		typedef std::vector<AabbType,	DirectX::XMAllocator> VolumeList;
		typedef std::vector<ObjectType, Eigen::aligned_allocator<ObjectType>> ObjectList;

	private:
		std::vector<Index> m_Children; //children of x are children[2x] and children[2x+1], indices bigger than boxes.size() index into objects.
		VolumeList m_Boxes;
		ObjectList m_Objects;

	public:
		inline bool isVolumeObject(Index index) const
		{

		}

		inline const Object& getVolumeObject(Index index) const
		{

		}

		inline const Volume &getVolume(Index index) const
		{
			return m_Boxes[index];
		}



	public:
		void Build(int begin, int end, int dim);

		int GetIntersectedObjects();
	};

	template<class _T, class _AabbT>
	inline void Octree<_T, _AabbT>::Build(int begin, int end, int dim)
	{
		if (end - begin == 1)
			return;
		else if (end - begin == 2)
		{

		}
		else
		{
			k = begin + (end - begin) / 2;
			auto bitr = m_Objects.begin() + begin;
			std::nth_element(bitr + begin, bitr + k, bitr + end,[]())
		}
	}
}