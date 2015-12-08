#include <vector>
#include <cassert>
#include "csg.h"

namespace Geometrics
{
	using std::vector;
	class Extrusion
	{
		Patch* m_top, m_bottom;
		Curve* m_path;
		vector<Vertex> m_vertices;
		vector<Triangle<uint16_t>> m_triangles;

	public:
		bool intersect(const Ray, float* distance)
		{}

		void trianglize(size_t pathSubdivition)
		{
			m_vertices.clear();
			m_triangles.clear();
			assert(m_bottom != nullptr);
			auto top = m_top->boundry();
			auto bottom = m_bottom->boundry();

			top->resample(bottom->size());
			centralize(top);
			centralize(bottom);
			alignCurve(top, bottom);

			auto&path = *m_path;
			auto length = path.length();
			auto interval = length / pathSubdivition;
			m_vertices.reserve(pathSubdivition * bottom.size());
			m_indices
				for (float t = .0f; t <= length; t += interval)
				{
					auto trans = getTransform(
						path.position(.0f),
						path.tangent(.0f)
						);

					for (int i = 0; i < bottom.size(); ++i)
					{
						auto v = bottom[i] * (1 - t) + top[i] * t;
						m_vertices.push_back(v);
					}
				}

			m_bottom->transform();
		}


	};
}