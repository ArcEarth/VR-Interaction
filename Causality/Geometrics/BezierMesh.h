#pragma once
#include "BezierClip.h"
#include <vector>
#include <list>

namespace Geometrics
{
	namespace Bezier
	{
		namespace Mesh
		{
			typedef float PointType;

			// Use Cubic-bezier patch as patch type
			typedef BezierClipping<PointType, 3>  CurveType;
			typedef BezierPatch<PointType, 3>	  PatchType;

			class BezierMesh;

			class HalfEdge;

			class Face;

			struct Vertex
			{
				PointType position;

				BezierMesh* mesh;

				std::list<HalfEdge*> in_edges;
				std::list<HalfEdge*> out_edges;
			};

			struct HalfEdge
			{
				CurveType curve;

				// owner mesh
				BezierMesh* mesh;

				// incident vertex
				Vertex* vertex;
				// incident face
				Face*	face;

				// opposite half edge
				HalfEdge* opposite;
				// previous half edge
				HalfEdge* prev;
				// next half edge
				HalfEdge* next;

			};

			struct Face
			{
				PatchType patch;

				HalfEdge* boundry_edge;
			};

			class BezierMesh
			{
			public:
				std::vector<Vertex> Vertices;
			};
		}
	}
}