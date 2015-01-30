#ifndef SURFACE_MESH_H
#define SURFACE_MESH_H
#pragma once

#include "MathHelper.h"
#include <vector>

namespace Geometrics
{
	namespace TriangleMesh
	{
		const unsigned int nNoEd  = 2;	//!< ê¸ï™Ç…Ç¢Ç≠ÇÁì_Ç™Ç†ÇÈÇ©

		const unsigned int nNoTri = 3;	//!< ÇRäpå`Ç…Ç¢Ç≠ÇÁí∏ì_Ç™Ç†ÇÈÇ©
		const unsigned int nEdTri = 3;	//!< ÇRäpå`Ç…Ç¢Ç≠ÇÁï”Ç™Ç†ÇÈÇ©
		//! ÇRäpå`ÇÃäeï”ÇÃí∏ì_î‘çÜ
		const unsigned int noelTriEdge[nEdTri][nNoEd] = {	
			{ 1, 2 }, //edge 0
			{ 2, 0 }, //edge 1
			{ 0, 1 }, //edge 2
		};
		//! ÇRäpå`ÇÃó◊ê⁄ä÷åW
		const unsigned int relTriTri[3][3] = {
			{ 0, 2, 1 }, //  0
			{ 2, 1, 0 }, //  1 
			{ 1, 0, 2 }, //  2
		};
		template <typename IndexType>
		struct Triangle
		{
			IndexType v[3];
			Triangle() {}
			Triangle(const IndexType &v0, const IndexType &v1, const IndexType &v2)
				{ v[0] = v0; v[1] = v1; v[2] = v2; }
			Triangle(const IndexType *v_)
				{ v[0] = v_[0]; v[1] = v_[1]; v[2] = v_[2]; }
			template <class S> explicit Triangle(const S &x)
				{ v[0] = x[0];  v[1] = x[1];  v[2] = x[2]; }
			IndexType &operator[] (int i) { return v[i]; }
			const IndexType &operator[] (int i) const { return v[i]; }
			operator const IndexType * () const { return &(v[0]); }
			operator IndexType * () { return &(v[0]); }
			int indexof(IndexType v_) const
			{
				return (v[0] == v_) ? 0 :
					   (v[1] == v_) ? 1 :
					   (v[2] == v_) ? 2 : -1;
			}
		};

		template <typename T>
		class MeshVertex
		{
		public:
			MeshVertex();
			MeshVertex(const T&);

			T& operator * ();
			const T& operator * ()const;
			T* operator -> ();
			const T* operator -> ()const;

			T VertexData;
		};


		template <typename VertexType , typename FaceType = Triangle<uint16_t>>
		class TriangleMesh
		{
		public:
			std::vector<VertexType> Vertecis;
			std::vector<FaceType> Faces;
		};



		//! ÇQéüå≥ÇRäpå`óvëfç\ë¢ëÃ
		class STri2D{
		public:
			STri2D()
			{
				r2[0] = -1;
				r2[1] = -1;
				r2[2] = -1;
			}

			unsigned int v[3];	//!< í∏ì_Index
			int g2[3];			//!< -2 the ith edge have neighboring face; -1 the ithe edge of this face is a boundary edge, and doesn't have adjacent faces
			unsigned int s2[3];	//!< index of face that is adjacent to ith edge; The 0th edge is the edge facing 0th vertex
			int r2[3];	//!< relationship of vertex index between two adjacent faces
		};

		/*!
		@brief ÇQéüå≥ì_ÉNÉâÉX
		(CPoint2D.e!=-1)Ç»ÇÁ(aTri[e].no[d])Ç™Ç±ÇÃì_ÇÃëSëÃêﬂì_î‘çÜÇ≈Ç†ÇÈÇÕÇ∏
		*/
		class CVertex3D{
		public:
			CVertex3D(){}
			CVertex3D( const CVertex3D& rhs )
				: p(rhs.p), n(rhs.n), e(rhs.e), d(rhs.d),color(rhs.color)
				//		mJointWeights(rhs.mJointWeights) 
			{}
			CVertex3D(float x, float y, float z, int ielem, unsigned int idir)
				: p(x,y,z), e(ielem), d(idir){}
		public:
			DirectX::Vector3 p;   //!< ì_ÇÃç¿ïW
			DirectX::Vector3 n;
			int e;              //!< ì_ÇàÕÇﬁóvëfÇÃÇ§ÇøàÍÇ¬ÇÃî‘çÜ(å«óßÇµÇƒÇ¢ÇÈì_Ç»ÇÁ-1Ç™ì¸ÇÈ)
			unsigned int d;     //!< ì_ÇàÕÇﬁóvëfeÇÃóvëfì‡êﬂì_î‘çÜ
			//	std::vector<float> mJointWeights;
			DirectX::XMFLOAT4 color;
		};



		bool MakePointSurTri( const std::vector<STri2D>& aTri, const unsigned int npoin, 
			unsigned int* const elsup_ind, unsigned int& nelsup, unsigned int*& elsup );

		bool CheckTri( const std::vector<STri2D>& aTri );

		bool CheckTri(
			const std::vector<CVertex3D>& po,
			const std::vector<STri2D>& tri );


		bool MakeInnerRelationTri( std::vector<STri2D>& aTri, const unsigned int npoin, 
			const unsigned int* elsup_ind, const unsigned int nelsup, const unsigned int* elsup );


		bool InsertPoint_ElemEdge( const unsigned int ipo_ins,    //the index of the new point
			const unsigned int itri_ins,  //triangle index
			const unsigned int ied_ins,  //edge index
			std::vector<CVertex3D>& po, std::vector<STri2D>& tri );

		void GetTriAryAroundPoint(
			unsigned int ipoin,
			std::vector< std::pair<unsigned int,unsigned int> >& aTriSurPo,
			const std::vector<CVertex3D>& aPo, const std::vector<STri2D>& aTri);

		// How to project one point to a triangle plane: http://www-compsci.swan.ac.uk/~csmark/PDFS/dist.pdf
		DirectX::Vector3 ProjectPointOnTriangle(const DirectX::Vector3 &p0,
			const DirectX::Vector3 &tri_p1, const DirectX::Vector3 &tri_p2, const DirectX::Vector3 &tri_p3);

		// How to decide if a point is inside a triangle: http://www.blackpawn.com/texts/pointinpoly/default.html
		bool IsPointInsideTriangle(const DirectX::Vector3 &p0,
			const DirectX::Vector3 &tri_p1, const DirectX::Vector3 &tri_p2, const DirectX::Vector3 &tri_p3);
		bool IsPointSameSide(const DirectX::Vector3 &p0, const DirectX::Vector3 &p1,
			const DirectX::Vector3 &line_p0, const DirectX::Vector3 &line_p1);
		// http://www.softsurfer.com/Archive/algorithm_0105/algorithm_0105.htm
		bool IsRayIntersectingTriangle(const DirectX::Vector3 &line0, const DirectX::Vector3 &line1,
			const DirectX::Vector3 &tri0, const DirectX::Vector3 &tri1, const DirectX::Vector3 &tri2,
			DirectX::Vector3 &intersectionPoint);

		bool FindRayTriangleMeshIntersections(
			const DirectX::Vector3 &line0,
			const DirectX::Vector3 &line1,
			const std::vector<STri2D>& aTri,
			const std::vector<CVertex3D> &aPoint3D,
			std::vector<DirectX::Vector3> &intersectionPoints);

		bool FindRayTriangleMeshIntersectionClosestToPoint(
			const DirectX::Vector3 &line0,
			const DirectX::Vector3 &line1,
			const std::vector<STri2D> &aTri,
			const std::vector<CVertex3D> &aPoint3D,
			const DirectX::Vector3 &targetPoint,
			DirectX::Vector3 &intersectionPoint);

		bool IsPointInsideTriangleMesh(
			const DirectX::Vector3 &point,
			const std::vector<STri2D> &aTri,
			const std::vector<CVertex3D> &aPoint3D);

		bool Collapse_ElemEdge(const unsigned int itri_del, const unsigned int ied_del,
			std::vector<CVertex3D>& aPo, std::vector<STri2D>& aTri,
			int &remain_vert_index);

		bool DeleteTri(unsigned int itri_to, std::vector<CVertex3D>& aPo, std::vector<STri2D>& aTri);

		bool DeletePoint(unsigned int ipo_del, std::vector<CVertex3D>& aPo, std::vector<STri2D>& aTri);

		void FindSharedTriEdgeIndex(
			int p0Idx,
			int p1Idx,
			int &triIndex,
			int &edgeIndex,
			const std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles);

		void FindRotationAngleAxisBetweenTwoVectors(const DirectX::Vector3 &v0, const DirectX::Vector3 &v1,
			float &angle, DirectX::Vector3 &axis);

		void RotateVectorAroundAxisByAngle(
			DirectX::Vector3 &vector,
			const DirectX::Vector3 &axis,
			const float &angle);

		bool AreTwoVerticesConnected(const unsigned int &vert1Index,
			const unsigned int &vert2Index,
			const std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles);

		void FindOneRingVerts(
			const unsigned int vertIndex,
			const std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles,
			std::vector<int> &oneRingVertIndices);

		// http://stackoverflow.com/questions/7565748/3d-orthogonal-projection-on-a-plane
		DirectX::Vector3 ProjectPointOnPlane(const DirectX::Vector3 &point,
			const DirectX::Vector3 &planeNormal, const DirectX::Vector3 &planePoint);

		void UpdateNormals(const std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles);

		void Smooth(const std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles);

		void InsertVerticesOnRefreshedMesh(std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			std::vector<STri2D> &triangles);

		void Remesh(std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			std::vector<STri2D> &triangles);
	}
}
#endif // #endif SURFACE_MESH_H