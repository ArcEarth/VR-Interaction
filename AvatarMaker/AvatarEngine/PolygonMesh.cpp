#include "stdafx.h"
#include "PolygonMesh.h"
#include <set>
#include <algorithm>
#include <iostream>

#pragma warning(push)
#pragma warning(disable : 4244)
using namespace DirectX;
using namespace std;
using namespace Geometrics::TriangleMesh;

namespace Geometrics
{
	namespace TriangleMesh
	{
		inline Vector3 Cross(const Vector3& A, const Vector3& B)
		{
			return A^B;
		}
		inline float Dot(const Vector3& A, const Vector3& B)
		{
			return A*B;
		}
		inline float Length(const Vector3& A)
		{
			return A.Length();
		}
		inline float Distance(const Vector3& A, const Vector3& B)
		{
			return (A-B).Length();
		}

		static const unsigned int invRelTriTri[3] = {
			0, 1, 2
		};

		// (node index opp to 0)*3+(node index opp to 1) -> relation index
		static const int noel2RelTriTri[9] = {
			-1,	// 0 00
			-1,	// 1 01
			0,	// 2 02
			2, // 3 10
			-1, // 4 11
			-1,	// 5 12
			-1,	// 6 20
			1,	// 7 21
			-1, // 8 22
		};

		// (edge index)*3+(opp edge index) -> relation index
		static const unsigned int ed2RelTriTri[9] = {
			0,	// 0 00
			2,	// 1 01
			1,	// 2 02
			2,	// 3 10
			1,	// 4 11
			0,	// 5 12
			1,	// 6 20
			0,	// 7 21
			2,	// 8 22
		}; 

		static const unsigned int indexRot3[3][3] = {
			{ 0, 1, 2 },
			{ 1, 2, 0 },
			{ 2, 0, 1 },
		};

		bool CheckTri( const std::vector<STri2D>& aTri )
		{
			const unsigned int ntri = aTri.size();
			for(unsigned int itri=0;itri<ntri;itri++){
				const STri2D& ref_tri = aTri[itri];
				/*		for(int inotri=0;inotri<nNoTri;inotri++){
				assert( ref_tri.v[inotri] >= 0 );
				}*/
				for(unsigned int iedtri=0;iedtri<nEdTri;iedtri++){
					if( ref_tri.g2[iedtri] == -2 || ref_tri.g2[iedtri] == -3 ){
						const unsigned int itri_s = ref_tri.s2[iedtri];
						const unsigned int irel = ref_tri.r2[iedtri];
						assert( itri_s < ntri );
						assert( irel < 3 );
						// check sorounding
						{
							const unsigned int noel_dia = relTriTri[irel][iedtri];
							assert( aTri[itri_s].s2[noel_dia] == itri );
							//					std::cout << itri << " " << itri_s << std::endl;
						}
						// check relation 
						for(unsigned int inoed=0;inoed<nNoEd;inoed++){
							const unsigned int inoel = noelTriEdge[iedtri][inoed];
							if( ref_tri.v[inoel] != aTri[itri_s].v[ (int)relTriTri[irel][inoel] ] ){
								std::cout << itri << " " << iedtri << " " << itri_s << " " << ref_tri.v[inoel] << " " << aTri[itri_s].v[ (int)relTriTri[irel][inoel] ] << std::endl;
							}
							assert( ref_tri.v[inoel] == aTri[itri_s].v[ (int)relTriTri[irel][inoel] ] );
						}
					}
				}
			}

			return true;
		}

		inline float TriArea(const Vector3& v1, const Vector3& v2, const Vector3& v3)
		{
			float x, y, z;
			x = ( v2.y - v1.y )*( v3.z - v1.z ) - ( v3.y - v1.y )*( v2.z - v1.z );
			y = ( v2.z - v1.z )*( v3.x - v1.x ) - ( v3.z - v1.z )*( v2.x - v1.x );
			z = ( v2.x - v1.x )*( v3.y - v1.y ) - ( v3.x - v1.x )*( v2.y - v1.y );
			return 0.5*sqrt( x*x + y*y + z*z );
		}

		bool CheckTri(
			const std::vector<CVertex3D>& po,
			const std::vector<STri2D>& tri )
		{
			//	std::cout << "Check Tri" << std::endl;

			const unsigned int npo = po.size();
			const unsigned int ntri = tri.size();

			////////////////////////////////
			// óvëfIndexÇÃÉ`ÉFÉbÉN

			for(unsigned int itri=0;itri<ntri;itri++){
				const STri2D& ref_tri = tri[itri];
				assert( ref_tri.v[0] != ref_tri.v[1] );
				assert( ref_tri.v[1] != ref_tri.v[2] );    
				if( ref_tri.v[2] == ref_tri.v[0] ){
					std::cout << itri << " " << ref_tri.v[0] << " " << ref_tri.v[1] << " " << ref_tri.v[2] << std::endl;
				}
				assert( ref_tri.v[2] != ref_tri.v[0] );        
				/////
				assert( ref_tri.s2[0] != ref_tri.s2[1] );
				assert( ref_tri.s2[1] != ref_tri.s2[2] );    
				assert( ref_tri.s2[2] != ref_tri.s2[0] );        
				for(unsigned int inotri=0;inotri<nNoTri;inotri++){
					assert( ref_tri.v[inotri] < npo );
				}
				for(unsigned int iedtri=0;iedtri<nEdTri;iedtri++){
					if( ref_tri.g2[iedtri] == -2 || ref_tri.g2[iedtri] == -3 ){
						const unsigned int itri_s = ref_tri.s2[iedtri];
						const unsigned int irel = ref_tri.r2[iedtri];
						assert( itri_s < ntri );
						assert( irel < 3 );
						// check sorounding
						{
							const unsigned int noel_dia = relTriTri[irel][iedtri];
							assert( noel_dia < 3 );
							if( tri[itri_s].s2[noel_dia] != itri ){
								std::cout << itri << " " << iedtri << std::endl;
							}
							assert( tri[itri_s].s2[noel_dia] == itri );
						}
						// check relation 
						for(unsigned int inoed=0;inoed<nNoEd;inoed++){
							const unsigned int inoel = noelTriEdge[iedtri][inoed];
							if( ref_tri.v[inoel] != tri[itri_s].v[ (int)relTriTri[irel][inoel] ] ){
								std::cout << itri << " " << iedtri << std::endl;
							}
							assert( ref_tri.v[inoel] == tri[itri_s].v[ (int)relTriTri[irel][inoel] ] );
						}
					}
				}
				/*		{
				if( ref_tri.g2[0]==-1 && ref_tri.g2[1]==-1 && ref_tri.g2[2]==-1 ){
				std::cout << "Isolated Triangle " << itri << std::endl;
				}
				}*/
			}

			////////////////////////////////
			// í∏ì_-óvëfä‘ÇÃàÍä—ê´ÇÃÉ`ÉFÉbÉN

			for(unsigned int ipoin=0;ipoin<npo;ipoin++){
				const int itri0 = po[ipoin].e;
				const int inoel0 = po[ipoin].d;
				if( po[ipoin].e >= 0 ){
					assert( po[ipoin].d >= 0 && po[ipoin].d < 3 );
					if( tri[itri0].v[inoel0] != ipoin ){
						//				std::cout << itri0 << " " << inoel0 << "   " << tri[itri0].v[inoel0] << " " << ipoin << std::endl;
					}
					assert( tri[itri0].v[inoel0] == ipoin );
				}
			}

			////////////////////////////////
			// GeometryÇÃÉ`ÉFÉbÉN

			for(unsigned int itri=0;itri<ntri;itri++){
				const STri2D& ref_tri = tri[itri];
				{
					float area = TriArea( po[ref_tri.v[0]].p, po[ref_tri.v[1]].p, po[ref_tri.v[2]].p);
					if( area < 1.0e-10 ){
						//				std::cout << "Negative Volume : " << itri << " " << area << std::endl;
					}
				}
				/*		{	
				CVector2D v0 = po[ref_tri.v[0]].p;
				CVector2D v1 = po[ref_tri.v[1]].p;
				CVector2D v2 = po[ref_tri.v[2]].p;

				float area = TriArea( v0, v1, v2 );
				const float tmp1 = 0.5 / area;

				float const_term[3];
				const_term[0] = tmp1*(v1.x*v2.y-v2.x*v1.y);
				const_term[1] = tmp1*(v2.x*v0.y-v0.x*v2.y);
				const_term[2] = tmp1*(v0.x*v1.y-v1.x*v0.y);

				float dldx[2][2];
				dldx[0][0] = tmp1*(v1.y-v2.y);
				dldx[1][0] = tmp1*(v2.y-v0.y);
				dldx[2][0] = tmp1*(v0.y-v1.y);

				dldx[0][1] = tmp1*(v2.x-v1.x);
				dldx[1][1] = tmp1*(v0.x-v2.x);
				dldx[2][1] = tmp1*(v1.x-v0.x);

				assert( fabs( dldx[0][0]+dldx[1][0]+dldx[2][0] ) < 1.0e-15 );
				assert( fabs( dldx[0][1]+dldx[1][1]+dldx[2][1] ) < 1.0e-15 );

				assert( fabs( const_term[0]+dldx[0][0]*v0.x+dldx[0][1]*v0.y - 1.0 ) < 1.0e-10 );
				assert( fabs( const_term[0]+dldx[0][0]*v1.x+dldx[0][1]*v1.y ) < 1.0e-10 );
				assert( fabs( const_term[0]+dldx[0][0]*v2.x+dldx[0][1]*v2.y ) < 1.0e-10 );

				assert( fabs( const_term[1]+dldx[1][0]*v0.x+dldx[1][1]*v0.y ) < 1.0e-10 );
				assert( fabs( const_term[1]+dldx[1][0]*v1.x+dldx[1][1]*v1.y - 1.0 ) < 1.0e-10 );
				assert( fabs( const_term[1]+dldx[1][0]*v2.x+dldx[1][1]*v2.y ) < 1.0e-10 );

				assert( fabs( const_term[2]+dldx[2][0]*v0.x+dldx[2][1]*v0.y ) < 1.0e-10 );
				assert( fabs( const_term[2]+dldx[2][0]*v1.x+dldx[2][1]*v1.y ) < 1.0e-10 );
				assert( fabs( const_term[2]+dldx[2][0]*v2.x+dldx[2][1]*v2.y - 1.0 ) < 1.0e-10 );
				}*/
			}

			return true;
		}


		bool DeleteTri(unsigned int itri_to, std::vector<CVertex3D>& aPo, std::vector<STri2D>& aTri)
		{
			if( itri_to >= aTri.size() ) return true;
			{
				assert( aTri[itri_to].g2[0] == -1 );
				assert( aTri[itri_to].g2[1] == -1 );
				assert( aTri[itri_to].g2[2] == -1 );
			}  
			const unsigned int itri_from = aTri.size()-1;
			if( itri_to == itri_from ){
				aTri.resize( aTri.size()-1 );
				return true;
			}
			//  std::cout << "delete : " << itri_to << " " << itri_from << std::endl;
			aTri[itri_to] = aTri[itri_from];
			aTri.resize( aTri.size()-1 );
			for(unsigned int iedtri=0;iedtri<nEdTri;iedtri++){
				if( aTri[itri_to].g2[iedtri] != -2 ) continue;
				const unsigned int itri_adj = aTri[itri_to].s2[iedtri];
				assert( itri_adj < aTri.size() );
				const unsigned int* rel = relTriTri[ (int)aTri[itri_to].r2[iedtri] ];
				const unsigned int iedtri_adj = rel[iedtri];
				//    std::cout << " sur : " << itri_to << " " << iedtri << " " << itri_adj << " " << iedtri_adj << std::endl;
				assert( aTri[itri_adj].g2[iedtri_adj] == -2 );
				assert( aTri[itri_adj].s2[iedtri_adj] == itri_from );
				aTri[itri_adj].s2[iedtri_adj] = itri_to;
			}
			for(unsigned int inotri=0;inotri<nNoTri;inotri++){
				const unsigned int ipo0 = aTri[itri_to].v[inotri];
				aPo[ipo0].e = itri_to;
				aPo[ipo0].d = inotri;
			}
			return true;
		}


		bool DeletePoint(unsigned int ipo_del, std::vector<CVertex3D>& aPo, std::vector<STri2D>& aTri)
		{
			if( ipo_del >= aPo.size() ) return true;
			{
				assert( aPo[ipo_del].e == -1 );
			}  
			const unsigned int ipo_from = aPo.size()-1;
			if( ipo_del == ipo_from ){
				aPo.resize( aPo.size()-1 );
				return true;
			}
			//  std::cout << "delete : " << itri_to << " " << itri_from << std::endl;

			std::vector< std::pair<unsigned int,unsigned int> > aTriSurPo;
			GetTriAryAroundPoint(ipo_from, aTriSurPo, aPo,aTri);
			aPo[ipo_del] = aPo[ipo_from];
			aPo.resize( aPo.size()-1 );

			for(unsigned int itrisp=0;itrisp<aTriSurPo.size();itrisp++){
				unsigned int itri = aTriSurPo[itrisp].first;
				unsigned int iedtri = aTriSurPo[itrisp].second;
				assert( aTri[itri].v[iedtri] == ipo_from );
				aTri[itri].v[iedtri] = ipo_del;
			}
			return true;
		}

		bool Collapse_ElemEdge(const unsigned int itri_del, const unsigned int ied_del,
			std::vector<CVertex3D>& aPo, std::vector<STri2D>& aTri,
			int &remain_vert_index)
		{  
			assert( itri_del < aTri.size() );  

			if( aTri[itri_del].g2[ied_del] != -2 ){
				std::cout << "Error!-->Not Implemented: Mesh with hole" << std::endl;
				assert(0);
			}

			const unsigned int itri_adj = aTri[itri_del].s2[ied_del];
			const unsigned int ied_adj  = (unsigned int)relTriTri[ (int)aTri[itri_del].r2[ied_del] ][ied_del];
			assert( itri_adj < aTri.size() );
			assert( ied_adj < 3 );  
			assert( aTri[itri_adj].s2[ied_adj] == itri_del );

			const unsigned int itri0 = itri_del;
			const unsigned int itri1 = itri_adj;
			const unsigned int itri2 = aTri[itri0].s2[ noelTriEdge[ied_del][0] ];  
			const unsigned int itri3 = aTri[itri0].s2[ noelTriEdge[ied_del][1] ];
			const unsigned int itri4 = aTri[itri1].s2[ noelTriEdge[ied_adj][0] ];
			const unsigned int itri5 = aTri[itri1].s2[ noelTriEdge[ied_adj][1] ];  

			const unsigned int ino0_0 = ied_del;
			const unsigned int ino1_0 = noelTriEdge[ino0_0][0];
			const unsigned int ino2_0 = noelTriEdge[ino0_0][1];

			const unsigned int ino0_1 = ied_adj;
			const unsigned int ino1_1 = noelTriEdge[ino0_1][0];
			const unsigned int ino2_1 = noelTriEdge[ino0_1][1];  

			const unsigned int ino0_2 = (unsigned int)relTriTri[ (int)aTri[itri0].r2[ino1_0] ][ino1_0];  
			const unsigned int ino1_2 = noelTriEdge[ino0_2][0];
			const unsigned int ino2_2 = noelTriEdge[ino0_2][1];    

			const unsigned int ino0_3 = (unsigned int)relTriTri[ (int)aTri[itri0].r2[ino2_0] ][ino2_0];
			const unsigned int ino1_3 = noelTriEdge[ino0_3][0];
			const unsigned int ino2_3 = noelTriEdge[ino0_3][1];    

			const unsigned int ino0_4 = (unsigned int)relTriTri[ (int)aTri[itri1].r2[ino1_1] ][ino1_1];
			const unsigned int ino1_4 = noelTriEdge[ino0_4][0];
			const unsigned int ino2_4 = noelTriEdge[ino0_4][1];  

			const unsigned int ino0_5 = (unsigned int)relTriTri[ (int)aTri[itri1].r2[ino2_1] ][ino2_1];
			const unsigned int ino1_5 = noelTriEdge[ino0_5][0];
			const unsigned int ino2_5 = noelTriEdge[ino0_5][1];    

			if( aTri[itri2].s2[ino2_2] == itri3 ) return false;
			if( aTri[itri3].s2[ino1_3] == itri2 ) return false;
			if( aTri[itri4].s2[ino2_4] == itri5 ) return false;
			if( aTri[itri5].s2[ino1_5] == itri4 ) return false;
			if( itri2 == itri5 && itri3 == itri4 ) return false;

			const STri2D old0 = aTri[itri0];
			const STri2D old1 = aTri[itri1];

			unsigned int ipo0 = old0.v[ino0_0];
			unsigned int ipo1 = old0.v[ino1_0];  
			unsigned int ipo2 = old1.v[ino0_1];
			unsigned int ipo3 = old1.v[ino1_1];  // to be delete

			{
				std::vector<int> ring1;
				{ // set triangle index from point 0 to point 1
					unsigned int jtri = itri5;
					unsigned int jnoel_c = ino1_5;
					unsigned int jnoel_b = noelTriEdge[jnoel_c][0];
					for(;;){
						assert( jtri < aTri.size() );
						assert( jnoel_c < 3 );
						assert( aTri[jtri].v[jnoel_c] == ipo3 );
						{
							unsigned int jpo = aTri[jtri].v[ noelTriEdge[jnoel_c][1] ];
							ring1.push_back(jpo);
						}        
						assert( aTri[jtri].g2[jnoel_b] == -2 );
						unsigned int ktri = aTri[jtri].s2[jnoel_b];
						const unsigned int rel01 = aTri[jtri].r2[jnoel_b];
						const unsigned int knoel_c = relTriTri[rel01][jnoel_c];
						const unsigned int knoel_b = relTriTri[rel01][ noelTriEdge[jnoel_c][1] ];
						assert( itri1 < aTri.size() );
						assert( aTri[ktri].s2[ relTriTri[rel01][jnoel_b] ] == jtri );
						if( ktri == itri2 ) break;
						jtri = ktri;
						jnoel_c = knoel_c;
						jnoel_b = knoel_b;
					}    
				}
				std::vector<int> ring2;
				{
					// set triangle index from point 0 to point 1
					unsigned int jtri = itri3;
					unsigned int jnoel_c = ino1_3;
					unsigned int jnoel_b = noelTriEdge[jnoel_c][0];
					for(;;) {
						assert( jtri < aTri.size() );
						assert( jnoel_c < 3 );
						assert( aTri[jtri].v[jnoel_c] == ipo1 );
						{
							unsigned int jpo = aTri[jtri].v[ noelTriEdge[jnoel_c][1] ];
							ring2.push_back(jpo);
						}
						assert( aTri[jtri].g2[jnoel_b] == -2 );
						unsigned int ktri = aTri[jtri].s2[jnoel_b];
						const unsigned int rel01 = aTri[jtri].r2[jnoel_b];
						const unsigned int knoel_c = relTriTri[rel01][jnoel_c];
						const unsigned int knoel_b = relTriTri[rel01][ noelTriEdge[jnoel_c][1] ];
						assert( itri1 < aTri.size() );
						assert( aTri[ktri].s2[ relTriTri[rel01][jnoel_b] ] == jtri );
						if( ktri == itri4 ) break;
						jtri = ktri;
						jnoel_c = knoel_c;
						jnoel_b = knoel_b;
					}    
				}    
				sort(ring1.begin(),ring1.end());
				sort(ring2.begin(),ring2.end());
				std::vector<int> insc(ring1.size());
				std::vector<int>::iterator it = set_intersection (ring1.begin(), ring1.end(), ring2.begin(), ring2.end(), insc.begin());
				if( it != insc.begin() ){ return  false; }
			}


			//  std::cout << std::endl;
			//  std::cout << "stt" << std::endl;
			//  std::cout << "tris : " << itri0 << " " << itri1 << " " << itri2 << " " << itri3 << " " << itri4 << " " << itri5 << std::endl;
			//  std::cout << "vtxs : " << ipo0 << " " << ipo1 << " " << ipo2 << " " << ipo3 << std::endl;

			assert( old0.v[ino1_0] == old1.v[ino2_1] );
			assert( old0.v[ino2_0] == old1.v[ino1_1] );
			assert( old0.s2[ino0_0 ] == itri1 );
			assert( old1.s2[ino0_1 ] == itri0 );

			{
				aPo[ ipo0 ].e = itri2;	aPo[ ipo0 ].d = ino1_2;
				aPo[ ipo2 ].e = itri4;	aPo[ ipo2 ].d = ino1_4;
				aPo[ ipo1 ].e = itri3;	aPo[ ipo1 ].d = ino1_3;
				aPo[ ipo3 ].e = -1;
			}

			{ // change itri2
				STri2D& tri = aTri[itri2];  
				tri.g2[ino0_2] = old0.g2[ino2_0];
				tri.s2[ino0_2] = old0.s2[ino2_0];
				if( old0.g2[ino2_0] == -2 || old0.g2[ino2_0] == -3 ){
					assert( old0.r2[ino2_0] < 3 );
					assert( old0.s2[ino2_0] < aTri.size() );      
					tri.r2[ino0_2] = ed2RelTriTri[ ino0_2*3 + ino0_3];
					assert( tri.r2[ino0_2] >= 0 && tri.r2[ino0_2] < 3 );
					aTri[ itri3 ].s2[ ino0_3 ] = itri2;
					aTri[ itri3 ].r2[ ino0_3 ] = invRelTriTri[ tri.r2[ino0_2] ];
				}    
			}

			{ // change itri3
				STri2D& tri = aTri[itri3];
				tri.g2[ino0_3] = old0.g2[ino1_0];
				tri.s2[ino0_3] = old0.s2[ino1_0];
				if( old0.g2[ino1_0] == -2 || old0.g2[ino1_0] == -3 ){
					assert( old0.r2[ino1_0] < 3 );
					assert( old0.s2[ino1_0] < aTri.size() );      
					tri.r2[ino0_3] = ed2RelTriTri[ ino0_3*3 + ino0_2 ];
					assert( tri.r2[ino0_3] >= 0 && tri.r2[ino0_3] < 3 );
					aTri[ itri2 ].s2[ ino0_2 ] = itri3;
					aTri[ itri2 ].r2[ ino0_2 ] = invRelTriTri[ tri.r2[ino0_3] ];
				}    
			}

			{ // change itri4
				STri2D& tri = aTri[itri4];  
				tri.g2[ino0_4] = old1.g2[ino2_1];
				tri.s2[ino0_4] = old1.s2[ino2_1];
				if( old1.g2[ino2_1] == -2 || old1.g2[ino2_1] == -3 ){
					assert( old1.r2[ino2_1] < 3 );      
					assert( old1.s2[ino2_1] < aTri.size() );      
					tri.r2[ino0_4] = ed2RelTriTri[ ino0_4*3 + ino0_5];
					assert( tri.r2[ino0_4] >= 0 && tri.r2[ino0_4] < 3 );
					aTri[ itri5 ].s2[ ino0_5 ] = itri4;
					aTri[ itri5 ].r2[ ino0_5 ] = invRelTriTri[ tri.r2[ino0_4] ];
					assert( relTriTri[ (int)aTri[itri4].r2[ino0_4] ][ino0_4] == ino0_5 );
					assert( relTriTri[ (int)aTri[itri5].r2[ino0_5] ][ino0_5] == ino0_4 );    
				}    
			}

			{ // change itri5
				STri2D& tri = aTri[itri5];
				tri.g2[ino0_5] = old1.g2[ino1_1];
				tri.s2[ino0_5] = old1.s2[ino1_1];
				if( old1.g2[ino1_1] == -2 || old1.g2[ino1_1] == -3 ){
					assert( old1.r2[ino1_1] < 3 );
					assert( old1.s2[ino1_1] < aTri.size() );      
					tri.r2[ino0_5] = ed2RelTriTri[ ino0_5*3 + ino0_4 ];
					assert( tri.r2[ino0_5] >= 0 && tri.r2[ino0_5] < 3 );
					aTri[ itri4 ].s2[ ino0_4 ] = itri5;
					aTri[ itri4 ].r2[ ino0_4 ] = invRelTriTri[ tri.r2[ino0_5] ];
					assert( relTriTri[ (int)aTri[itri5].r2[ino0_5] ][ino0_5] == ino0_4 );    
					assert( relTriTri[ (int)aTri[itri4].r2[ino0_4] ][ino0_4] == ino0_5 );
				}    
			}

			{ // set triangle index from point 0 to point 1
				unsigned int jtri = itri5;
				unsigned int jnoel_c = ino1_5;
				unsigned int jnoel_b = noelTriEdge[jnoel_c][0];
				for(;;){
					assert( jtri < aTri.size() );
					assert( jnoel_c < 3 );
					assert( aTri[jtri].v[jnoel_c] == ipo3 );
					//      std::cout << " fan : " << jtri << "     " << aTri[jtri].v[0] << " " << aTri[jtri].v[1] << " " << aTri[jtri].v[2] << std::endl;
					aTri[jtri].v[jnoel_c] = ipo1;
					assert( aTri[jtri].g2[jnoel_b] == -2 );
					unsigned int ktri = aTri[jtri].s2[jnoel_b];
					const unsigned int rel01 = aTri[jtri].r2[jnoel_b];
					const unsigned int knoel_c = relTriTri[rel01][jnoel_c];
					const unsigned int knoel_b = relTriTri[rel01][ noelTriEdge[jnoel_c][1] ];
					assert( itri1 < aTri.size() );
					assert( aTri[ktri].s2[ relTriTri[rel01][jnoel_b] ] == jtri );
					if( ktri == itri3 || ktri == itri4 ) break;
					jtri = ktri;
					jnoel_c = knoel_c;
					jnoel_b = knoel_b;
				}    
			}

			{	// isolate two triangles to be deleted
				aTri[itri0].g2[0] = -1;  aTri[itri0].g2[1] = -1;  aTri[itri0].g2[2] = -1;
				aTri[itri1].g2[0] = -1;  aTri[itri1].g2[1] = -1;  aTri[itri1].g2[2] = -1;
				const unsigned int itri_1st = ( itri0 > itri1 ) ? itri0 : itri1;
				const unsigned int itri_2nd = ( itri0 < itri1 ) ? itri0 : itri1;
				DeleteTri(itri_1st,aPo,aTri);
				DeleteTri(itri_2nd,aPo,aTri);
			}
			DeletePoint(ipo3,aPo,aTri);

			// Output the remaining vertex index
			remain_vert_index = ipo1;
			return true;
		}



		bool InsertPoint_ElemEdge( const unsigned int ipo_ins, 
			const unsigned int itri_ins, const unsigned int ied_ins,
			std::vector<CVertex3D>& po, std::vector<STri2D>& tri )
		{
			assert( itri_ins < tri.size() );
			assert( ipo_ins < po.size() );

			if( tri[itri_ins].g2[ied_ins] != -2 ){
				//		std::cout << "ñ¢é¿ëï" << std::endl;
				assert(0);
			}

			const unsigned int itri_adj = tri[itri_ins].s2[ied_ins];
			const unsigned int ied_adj  = (unsigned int)relTriTri[ (int)tri[itri_ins].r2[ied_ins] ][ied_ins];
			assert( itri_adj < tri.size() );
			assert( ied_ins < 3 );

			const unsigned int itri0 = itri_ins;
			const unsigned int itri1 = itri_adj;
			const unsigned int itri2 = tri.size();
			const unsigned int itri3 = tri.size()+1;

			tri.resize( tri.size()+2 );

			STri2D old0 = tri[itri_ins];
			STri2D old1 = tri[itri_adj];

			const unsigned int ino0_0 = ied_ins;
			const unsigned int ino1_0 = noelTriEdge[ied_ins][0];
			const unsigned int ino2_0 = noelTriEdge[ied_ins][1];

			const unsigned int ino0_1 = ied_adj;
			const unsigned int ino1_1 = noelTriEdge[ied_adj][0];
			const unsigned int ino2_1 = noelTriEdge[ied_adj][1];

			assert( old0.v[ino1_0] == old1.v[ino2_1] );
			assert( old0.v[ino2_0] == old1.v[ino1_1] );
			assert( old0.s2[ino0_0 ] == itri1 );
			assert( old1.s2[ino0_1 ] == itri0 );

			po[ipo_ins].e = itri0;			po[ipo_ins].d = 0;
			po[ old0.v[ino2_0] ].e = itri0;	po[ old0.v[ino2_0] ].d = 1;
			po[ old0.v[ino0_0] ].e = itri1;	po[ old0.v[ino0_0] ].d = 1;
			po[ old1.v[ino2_1] ].e = itri2;	po[ old1.v[ino2_1] ].d = 1;
			po[ old1.v[ino0_1] ].e = itri3;	po[ old1.v[ino0_1] ].d = 1;

			{
				STri2D& ref_tri = tri[itri0];
				////////////////
				ref_tri.v[0]  = ipo_ins;			    ref_tri.v[1]  = old0.v[ino2_0];	ref_tri.v[2]  = old0.v[ino0_0];
				ref_tri.g2[0] = old0.g2[ino1_0];	ref_tri.g2[1] = -2;				      ref_tri.g2[2] = -2;
				ref_tri.s2[0] = old0.s2[ino1_0];	ref_tri.s2[1] = itri1;			    ref_tri.s2[2] = itri3;
				////////////////
				if( old0.g2[ino1_0] == -2 || old0.g2[ino1_0] == -3 ){
					assert( old0.r2[ino1_0] < 3 );
					const unsigned int* rel = relTriTri[ old0.r2[ino1_0] ];
					ref_tri.r2[0] = noel2RelTriTri[ rel[ino1_0]*3 + rel[ino2_0] ];
					assert( ref_tri.r2[0] >= 0 && ref_tri.r2[0] < 3 );
					assert( old0.s2[ino1_0] < tri.size() );
					tri[ old0.s2[ino1_0] ].s2[ rel[ino1_0] ] = itri0;
					tri[ old0.s2[ino1_0] ].r2[ rel[ino1_0] ] = invRelTriTri[ ref_tri.r2[0] ];
				}
				ref_tri.r2[1] = 0;
				ref_tri.r2[2] = 0;
			}
			{
				STri2D& ref_tri = tri[itri1];
				////////////////
				ref_tri.v[0]  = ipo_ins;			ref_tri.v[1]  = old0.v[ino0_0];	ref_tri.v[2]  = old0.v[ino1_0];
				ref_tri.g2[0] = old0.g2[ino2_0];	ref_tri.g2[1] = -2;				ref_tri.g2[2] = -2;
				ref_tri.s2[0] = old0.s2[ino2_0];	ref_tri.s2[1] = itri2;			ref_tri.s2[2] = itri0;
				////////////////
				if( old0.g2[ino2_0] == -2 || old0.g2[ino2_0] == -3 ){
					assert( old0.r2[ino2_0] < 3 );
					const unsigned int* rel = relTriTri[ old0.r2[ino2_0] ];
					ref_tri.r2[0] = noel2RelTriTri[ rel[ino2_0]*3 + rel[ino0_0] ];
					assert( ref_tri.r2[0] >= 0 && ref_tri.r2[0] < 3 );
					assert( old0.s2[ino2_0] < tri.size() );
					tri[ old0.s2[ino2_0] ].s2[ rel[ino2_0] ] = itri1;
					tri[ old0.s2[ino2_0] ].r2[ rel[ino2_0] ] = invRelTriTri[ ref_tri.r2[0] ];
				}
				ref_tri.r2[1] = 0;
				ref_tri.r2[2] = 0;
			}
			{
				STri2D& ref_tri = tri[itri2];
				////////////////
				ref_tri.v[0]  = ipo_ins;			ref_tri.v[1]  = old1.v[ino2_1];	ref_tri.v[2]  = old1.v[ino0_1];
				ref_tri.g2[0] = old1.g2[ino1_1];	ref_tri.g2[1] = -2;				ref_tri.g2[2] = -2;
				ref_tri.s2[0] = old1.s2[ino1_1];	ref_tri.s2[1] = itri3;			ref_tri.s2[2] = itri1;
				////////////////
				if( old1.g2[ino1_1] == -2 || old0.g2[ino2_0] == -3 ){
					assert( old1.r2[ino1_1] < 3 );
					const unsigned int* rel = relTriTri[ old1.r2[ino1_1] ];
					ref_tri.r2[0] = noel2RelTriTri[ rel[ino1_1]*3 + rel[ino2_1] ];
					assert( ref_tri.r2[0] >= 0 && ref_tri.r2[0] < 3 );
					assert( old1.s2[ino1_1] < tri.size() );
					tri[ old1.s2[ino1_1] ].s2[ rel[ino1_1] ] = itri2;
					tri[ old1.s2[ino1_1] ].r2[ rel[ino1_1] ] = invRelTriTri[ ref_tri.r2[0] ];
				}
				ref_tri.r2[1] = 0;
				ref_tri.r2[2] = 0;
			}
			{
				STri2D& ref_tri = tri[itri3];
				ref_tri.v[0]  = ipo_ins;			ref_tri.v[1]  = old1.v[ino0_1];	ref_tri.v[2]  = old1.v[ino1_1];
				ref_tri.g2[0] = old1.g2[ino2_1];	ref_tri.g2[1] = -2;				ref_tri.g2[2] = -2;
				ref_tri.s2[0] = old1.s2[ino2_1];	ref_tri.s2[1] = itri0;			ref_tri.s2[2] = itri2;
				if( old1.g2[ino2_1] == -2 || old1.g2[ino2_1] == -3 ){
					assert( old1.r2[ino2_1] < 3 );
					const unsigned int* rel = relTriTri[ old1.r2[ino2_1] ];
					ref_tri.r2[0] = noel2RelTriTri[ rel[ino2_1]*3 + rel[ino0_1] ];
					assert( ref_tri.r2[0] >= 0 && ref_tri.r2[0] < 3 );
					assert( old1.s2[ino2_1] < tri.size() );
					tri[ old1.s2[ino2_1] ].s2[ rel[ino2_1] ] = itri3;
					tri[ old1.s2[ino2_1] ].r2[ rel[ino2_1] ] = invRelTriTri[ ref_tri.r2[0] ];
				}
				ref_tri.r2[1] = 0;
				ref_tri.r2[2] = 0;
			}
			return true;
		}


		bool MakeInnerRelationTri( std::vector<STri2D>& aTri, const unsigned int npoin, 
			const unsigned int* elsup_ind, const unsigned int nelsup, const unsigned int* elsup )
		{
			const unsigned int EdEd2Rel[nEdTri][nEdTri] = {
				{ 0, 2, 1 },
				{ 2, 1, 0 },
				{ 1, 0, 2 } };

			unsigned int* tmp_poin = new unsigned int [npoin];
			for(unsigned int ipoin=0;ipoin<npoin;ipoin++){ tmp_poin[ipoin] = 0; }
			unsigned int inpofa[2];

			const unsigned int nTri = aTri.size();
			for(unsigned int itri=0;itri<nTri;itri++){
				for(unsigned int iedtri=0;iedtri<nEdTri;iedtri++){
					for(unsigned int ipoed=0;ipoed<nNoEd;ipoed++){
						inpofa[ipoed] = aTri[itri].v[ noelTriEdge[iedtri][ipoed] ];
						tmp_poin[ inpofa[ipoed] ] = 1;
					}
					const unsigned int ipoin0= inpofa[0];
					bool iflg = false;
					for(unsigned int ielsup=elsup_ind[ipoin0];ielsup<elsup_ind[ipoin0+1];ielsup++){
						const unsigned int jtri0 = elsup[ielsup];
						if( jtri0 == itri ) continue;
						for(unsigned int jedtri=0;jedtri<nEdTri;jedtri++){
							iflg = true;
							for(unsigned int jpoed=0;jpoed<nNoEd;jpoed++){
								const unsigned int jpoin0 =  aTri[jtri0].v[ noelTriEdge[jedtri][jpoed] ];
								if( tmp_poin[ jpoin0 ] == 0 ){ iflg = false; break; }
							}
							if( iflg ){
								aTri[itri].g2[iedtri] = -2;
								aTri[itri].s2[iedtri] = jtri0;
								aTri[itri].r2[iedtri] = EdEd2Rel[iedtri][jedtri];
								break;
							}
						}
						if( iflg ) break;
					}
					if( !iflg ){ 
						aTri[itri].g2[iedtri] = -1;
					}
					for(unsigned int ipofa=0;ipofa<nNoEd;ipofa++){
						tmp_poin[ inpofa[ipofa] ] = 0;
					}
				}
			}

			delete[] tmp_poin;
			return true;
		}


		bool MakePointSurTri( const std::vector<STri2D>& aTri, const unsigned int npoin, 
			unsigned int* const elsup_ind, unsigned int& nelsup, unsigned int*& elsup ){

				const unsigned int nnotri = 3;

				for(unsigned int ipoin=0;ipoin<npoin+1;ipoin++){
					elsup_ind[ipoin] = 0;
				}
				for(unsigned int itri=0;itri<aTri.size();itri++){
					for(unsigned int inotri=0;inotri<nnotri;inotri++){
						elsup_ind[ aTri[itri].v[inotri]+1 ]++;
					}
				}
				for(unsigned int ipoin=0;ipoin<npoin;ipoin++){
					elsup_ind[ipoin+1] += elsup_ind[ipoin];
				}
				nelsup = elsup_ind[npoin];
				elsup = new unsigned int [nelsup];
				for(unsigned int itri=0;itri<aTri.size();itri++){
					for(unsigned int inotri=0;inotri<nnotri;inotri++){
						const unsigned int ipoin0 = aTri[itri].v[inotri];
						const unsigned int ielsup = elsup_ind[ipoin0];
						elsup[ielsup] = itri;
						elsup_ind[ipoin0]++;
					}
				}
				for(int ipoin=npoin;ipoin>0;ipoin--){
					elsup_ind[ipoin] = elsup_ind[ipoin-1];
				}
				elsup_ind[0] = 0;
				/*
				for(unsigned int ipoin=0;ipoin<npoin;ipoin++){
				std::cout << ipoin << " ";
				for(unsigned int ielsup=elsup_ind[ipoin];ielsup<elsup_ind[ipoin+1];ielsup++){
				std::cout << elsup[ielsup] << " ";
				}
				std::cout << std::endl;
				}
				*/
				return true;
		}

		int SignofNumber(float a)
		{
			if(a > 0)
				return 1;
			if(a < 0)
				return -1;
			return 0;
		}


		DirectX::Vector3 ProjectPointOnTriangle(const DirectX::Vector3 &p0,
			const DirectX::Vector3 &tri_p1, const DirectX::Vector3 &tri_p2, const DirectX::Vector3 &tri_p3)
		{
			DirectX::Vector3 normal = Cross(tri_p2 - tri_p1, tri_p3 - tri_p1);
			float cosAlpha = Dot(p0 - tri_p1, normal) / (Length(p0 - tri_p1) * Length(normal));
			float lenP0ProjectedP0 = Length(tri_p1 - p0) * cosAlpha;
			DirectX::Vector3 p0ProjectedP0 = -1 * lenP0ProjectedP0 * normal / Length(normal);

			return p0 + p0ProjectedP0;
		}

		DirectX::Vector3 ProjectPointOnPlane(const DirectX::Vector3 &point,
			const DirectX::Vector3 &planeNormal, const DirectX::Vector3 &planePoint)
		{
			float d = -planeNormal.x * planePoint.x - planeNormal.y * planePoint.y - planeNormal.z * planePoint.x;
			float t = (Dot(point, planeNormal) + d) / Dot(planeNormal, planeNormal);

			return point - planeNormal * t;
		}

		bool FindRayTriangleMeshIntersectionClosestToPoint(const DirectX::Vector3 &line0,
			const DirectX::Vector3 &line1,
			const std::vector<STri2D>& aTri,
			const std::vector<CVertex3D> &aPoint3D,
			const DirectX::Vector3 &targetPoint,
			DirectX::Vector3 &intersectionPoint)
		{
			std::vector<DirectX::Vector3> intersectionPoints;
			if (!FindRayTriangleMeshIntersections(line0, line1, aTri, aPoint3D, intersectionPoints))
			{
				return false;
			}

			// Find the point that is the closest to the target point
			float minSquareDistance = 1.0e16f;
			for (unsigned int i = 0; i < intersectionPoints.size(); i++)
			{
				float currSquareDistance = 
					(intersectionPoints[i].x - targetPoint.x) * (intersectionPoints[i].x - targetPoint.x) +
					(intersectionPoints[i].y - targetPoint.y) * (intersectionPoints[i].y - targetPoint.y) +
					(intersectionPoints[i].z - targetPoint.z) * (intersectionPoints[i].z - targetPoint.z);
				if (currSquareDistance < minSquareDistance)
				{
					intersectionPoint = intersectionPoints[i];
					minSquareDistance = currSquareDistance;
				}
			}

			return true;
		}

		bool FindRayTriangleMeshIntersections(const DirectX::Vector3 &line0,
			const DirectX::Vector3 &line1,
			const std::vector<STri2D>& aTri,
			const std::vector<CVertex3D> &aPoint3D,
			std::vector<DirectX::Vector3> &intersectionPoints)
		{
			intersectionPoints.clear();

			// Find all the intersection points between this ray and all triangles in the mesh
			for (unsigned int i = 0; i < aTri.size(); i++)
			{
				DirectX::Vector3 intersectionPoint;
				if (IsRayIntersectingTriangle(line0, line1,
					aPoint3D[aTri[i].v[0]].p,
					aPoint3D[aTri[i].v[1]].p,
					aPoint3D[aTri[i].v[2]].p,
					intersectionPoint))
				{
					intersectionPoints.push_back(intersectionPoint);
				}
			}

			if (intersectionPoints.empty())
			{
				return false;
			} else {
				return true;
			}

		}

		bool IsRayIntersectingTriangle(const DirectX::Vector3 &line0, const DirectX::Vector3 &line1,
			const DirectX::Vector3 &tri0, const DirectX::Vector3 &tri1, const DirectX::Vector3 &tri2,
			DirectX::Vector3 &intersectionPoint)
		{
			DirectX::Vector3 normal = Cross(tri1 - tri0, tri2 - tri0);

			// The ray is parallel to the triangle plane
			if (Dot(normal, line1 - line0) == 0)
			{
				return false;
			}

			float r = Dot(normal, tri0 - line0) / Dot(normal, line1 - line0);

			// The ray does not intersect the triangle plane
			if (r < 0)
			{
				return false;
			}

			// Find the intersection point
			intersectionPoint = line0 + r * (line1 - line0);

			if (!IsPointInsideTriangle(intersectionPoint,
				tri0, tri1, tri2))
			{
				return false;
			}

			return true;
		}

		bool IsPointInsideTriangle(const DirectX::Vector3 &p0,
			const DirectX::Vector3 &tri_p1, const DirectX::Vector3 &tri_p2, const DirectX::Vector3 &tri_p3)
		{
			if (IsPointSameSide(p0, tri_p1, tri_p2, tri_p3)
				&& IsPointSameSide(p0, tri_p2, tri_p1, tri_p3)
				&& IsPointSameSide(p0, tri_p3, tri_p1, tri_p2))
			{
				return true;
			} else {
				return false;
			}
		}

		bool IsPointSameSide(const DirectX::Vector3 &p0, const DirectX::Vector3 &p1,
			const DirectX::Vector3 &line_p0, const DirectX::Vector3 &line_p1)
		{
			DirectX::Vector3 crossProd1 = Cross(line_p1 - line_p0, p0 - line_p0);
			DirectX::Vector3 crossProd2 = Cross(line_p1 - line_p0, p1 - line_p0);

			if (Dot(crossProd1, crossProd2) >= 0)
			{
				return true;
			} else {
				return false;
			}
		}

		void GetTriAryAroundPoint(unsigned int ipoin,
			std::vector< std::pair<unsigned int,unsigned int> >& aTriSurPo,
			const std::vector<CVertex3D>& aPo, const std::vector<STri2D>& aTri)
		{
			const unsigned int itri_ini = aPo[ipoin].e;
			const unsigned int inoel_c_ini = aPo[ipoin].d;
			assert( itri_ini < aTri.size() );
			assert( inoel_c_ini < 3 );
			assert( aTri[itri_ini].v[inoel_c_ini] == ipoin );
			unsigned int itri0= itri_ini;
			unsigned int inoel_c0 = inoel_c_ini;
			unsigned int inoel_b0 = noelTriEdge[inoel_c0][0];
			bool is_bound_flg = false;
			for(;;){
				assert( itri0 < aTri.size() );
				assert( inoel_c0 < 3 );
				assert( aTri[itri0].v[inoel_c0] == ipoin );
				{
					aTriSurPo.push_back( std::make_pair(itri0,inoel_c0) );
				}
				assert( aTri[itri0].g2[inoel_b0] == -2 );
				unsigned int itri1 = aTri[itri0].s2[inoel_b0];
				const unsigned int rel01 = aTri[itri0].r2[inoel_b0];
				const unsigned int inoel_c1 = relTriTri[rel01][inoel_c0];
				const unsigned int inoel_b1 = relTriTri[rel01][ noelTriEdge[inoel_c0][1] ];
				assert( itri1 < aTri.size() );
				assert( aTri[itri1].s2[ relTriTri[rel01][inoel_b0] ] == itri0 );
				assert( aTri[itri1].v[inoel_c1] == ipoin );
				if( itri1 == itri_ini ) break;
				itri0 = itri1;
				inoel_c0 = inoel_c1;
				inoel_b0 = inoel_b1;
			}
		}

		void FindSharedTriEdgeIndex(
			int p0Idx,
			int p1Idx,
			int &triIndex,
			int &edgeIndex,
			const std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles)
		{
			triIndex = -1;
			edgeIndex = -1;

			std::vector< std::pair<unsigned int,unsigned int>> p0NeighborTriList;
			GetTriAryAroundPoint(p0Idx, p0NeighborTriList, vertices, triangles);
			std::vector< std::pair<unsigned int,unsigned int>> p1NeighborTriList;
			GetTriAryAroundPoint(p1Idx, p1NeighborTriList, vertices, triangles);

			for(unsigned int i = 0; i < p0NeighborTriList.size(); i++)
			{
				for(unsigned int j = 0; j < p1NeighborTriList.size(); j++)
				{
					// Find the triangle the two vertices share
					if(p0NeighborTriList[i].first == p1NeighborTriList[j].first)
					{
						triIndex = p0NeighborTriList[i].first;

						// Find the edge in the triangle the two vertices share
						for(int itre=0; itre<3; itre++)
						{
							if ((p0Idx == triangles[triIndex].v[noelTriEdge[itre][0]]
							|| p0Idx == triangles[triIndex].v[noelTriEdge[itre][1]])
								&& (p1Idx == triangles[triIndex].v[noelTriEdge[itre][0]]
							|| p1Idx == triangles[triIndex].v[noelTriEdge[itre][1]]))
							{
								edgeIndex = itre;

								return;
							}
						}
					}
				}
			}

			return;
		}

		// Shoot a random ray from this point and see how many triangles
		// it intersects. If there are an odd number of intersections,
		// this point is inside the mesh. If even, this point is outside.
		bool IsPointInsideTriangleMesh(
			const DirectX::Vector3 &point,
			const std::vector<STri2D> &aTri,
			const std::vector<CVertex3D> &aPoint3D)
		{
			DirectX::Vector3 origin(0.0, 0.0, 0.0);
			std::vector<DirectX::Vector3> intersectionPoints;
			if (FindRayTriangleMeshIntersections(point, origin, aTri, aPoint3D, intersectionPoints))
			{
				if (intersectionPoints.size() % 2 == 0)
				{
					return false;
				} else {
					return true;
				}
			} else {
				return false;
			}
		}

		void FindRotationAngleAxisBetweenTwoVectors(const DirectX::Vector3 &v0, const DirectX::Vector3 &v1,
			float &angle, DirectX::Vector3 &axis)
		{
			axis = Cross(v0, v1);
			axis.Normalize();

			float cosAngle = Dot(v0, v1);
			angle = acos(cosAngle);
		}

		void RotateVectorAroundAxisByAngle(
			DirectX::Vector3 &vector,
			const DirectX::Vector3 &axis,
			const float &angle)
		{
			vector =
				vector * cos(angle)
				+ Cross(axis, vector) * sin(angle)
				+ axis * Dot(axis, vector) * (1.0 - cos(angle));
		}

		bool AreTwoVerticesConnected(const unsigned int &vert1Index,
			const unsigned int &vert2Index,
			const std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles)
		{
			// Find the one ring triangles of vert 1
			std::vector<int> oneRingVert1Indices;
			FindOneRingVerts(vert1Index, vertices, triangles, oneRingVert1Indices);

			if (std::find(oneRingVert1Indices.begin(), oneRingVert1Indices.end(), vert2Index)
				== oneRingVert1Indices.end())
			{
				return false;
			}
			else 
			{
				return true;
			}
		}

		void FindOneRingVerts(
			const unsigned int vertIndex,
			const std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles,
			std::vector<int> &oneRingVertIndices)
		{
			assert(vertIndex >= 0 && vertIndex < vertices.size());
			oneRingVertIndices.clear();

			std::vector<std::pair<unsigned int, unsigned int>> triEdgeList;
			GetTriAryAroundPoint(vertIndex, triEdgeList, vertices, triangles);
			for (unsigned int iTri = 0; iTri < triEdgeList.size(); iTri++)
			{
				for (unsigned int iVert = 0; iVert < nNoTri; iVert++)
				{
					if (triangles[triEdgeList[iTri].first].v[iVert] != vertIndex)
					{
						oneRingVertIndices.push_back(triangles[triEdgeList[iTri].first].v[iVert]);
					}
				}
			}

			// Remove duplicates
			std::set<int> oneRingVertIndexSet(oneRingVertIndices.begin(),
				oneRingVertIndices.end());
			oneRingVertIndices.assign(oneRingVertIndexSet.begin(),
				oneRingVertIndexSet.end());
		}

		void UpdateNormals(const std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles)
		{
			// Create normal buffer for all eVertices
			std::vector<DirectX::Vector3> *pNormalBuffer =
				new std::vector<DirectX::Vector3>[updatedVertexIndices.size()];

			// Compute the surface normals and store them at all vertices
			for (unsigned int i = 0; i < updatedVertexIndices.size(); i ++)
			{
				std::vector< std::pair<unsigned int,unsigned int> > neighborTriList;
				GetTriAryAroundPoint(updatedVertexIndices[i], neighborTriList, vertices, triangles);

				for (unsigned int j = 0; j < neighborTriList.size(); j++)
				{
					const DirectX::Vector3 vertex0 = vertices.at(triangles.at(neighborTriList[j].first).v[0]).p;
					const DirectX::Vector3 vertex1 = vertices.at(triangles.at(neighborTriList[j].first).v[1]).p;
					const DirectX::Vector3 vertex2 = vertices.at(triangles.at(neighborTriList[j].first).v[2]).p;

					const DirectX::Vector3 edge1 = vertex1 - vertex2;
					const DirectX::Vector3 edge2 = vertex0 - vertex2;

					DirectX::Vector3 normal = Cross(edge1, edge2);

					pNormalBuffer[i].push_back(normal);
				}
			}

			// Average the normals at each vertex
			// and update the normal info in the triangle mesh
			for (unsigned int i = 0; i < updatedVertexIndices.size(); i++)
			{
				DirectX::Vector3 avgNormal(0.0, 0.0, 0.0);
				for (unsigned int j = 0; j < pNormalBuffer[i].size(); j++)
				{
					avgNormal += pNormalBuffer[i][j];
				}
				avgNormal /= pNormalBuffer[i].size();
				avgNormal.Normalize();

				vertices.at(updatedVertexIndices[i]).n = avgNormal;
			}

			// Cleaning up
			for (unsigned int i = 0; i < pNormalBuffer->size(); i++)
			{
				pNormalBuffer[i].clear();
			}
			delete[] pNormalBuffer;
		}

		//smooth the deformed vertices
		void Smooth(const std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			const std::vector<STri2D> &triangles)
		{
			for(unsigned int ip = 0; ip < updatedVertexIndices.size(); ip++)
			{
				int ipo = updatedVertexIndices[ip];

				std::vector< std::pair<unsigned int,unsigned int> > aTriSurPo;
				GetTriAryAroundPoint(ipo, aTriSurPo, vertices, triangles);

				float sumgc[3] = {0,0,0};

				for(unsigned int itsp=0; itsp < aTriSurPo.size();itsp++)
				{
					// Find the other two vertices in the triangle (i.e. one-ring neighbor vertices)
					int v0 = -1;
					int v1 = -1;
					if (triangles[aTriSurPo[itsp].first].v[0] == ipo)
					{
						v0 = 1;
						v1 = 2;
					} else if (triangles[aTriSurPo[itsp].first].v[1] == ipo)
					{
						v0 = 0;
						v1 = 2;
					} else if (triangles[aTriSurPo[itsp].first].v[2] == ipo)
					{
						v0 = 0;
						v1 = 1;
					}

					// Sum all the one-ring neighbors' positions
					sumgc[0] += vertices[triangles[aTriSurPo[itsp].first].v[v0]].p.x;
					sumgc[0] += vertices[triangles[aTriSurPo[itsp].first].v[v1]].p.x;

					sumgc[1] += vertices[triangles[aTriSurPo[itsp].first].v[v0]].p.y;
					sumgc[1] += vertices[triangles[aTriSurPo[itsp].first].v[v1]].p.y;

					sumgc[2] += vertices[triangles[aTriSurPo[itsp].first].v[v0]].p.z;
					sumgc[2] += vertices[triangles[aTriSurPo[itsp].first].v[v1]].p.z;
				}

				// Average the sum with number of vertices: num of triangles * 2 vertices for each triangle
				float GC[3] = {sumgc[0] / (aTriSurPo.size() * 2),
					sumgc[1] / (aTriSurPo.size() * 2), sumgc[2] / (aTriSurPo.size() * 2)};

				// Project the moved point to the original triangle mesh
				bool isProjectedPointFound = false;
				DirectX::Vector3 projectedPoint;
				for(unsigned int itsp=0; itsp< aTriSurPo.size();itsp++)
				{
					DirectX::Vector3 barycentricPoint(GC[0], GC[1], GC[2]);
					projectedPoint =
						ProjectPointOnTriangle(barycentricPoint,
						vertices[triangles[aTriSurPo[itsp].first].v[0]].p,
						vertices[triangles[aTriSurPo[itsp].first].v[1]].p,
						vertices[triangles[aTriSurPo[itsp].first].v[2]].p);

					if (IsPointInsideTriangle(projectedPoint,
						vertices[triangles[aTriSurPo[itsp].first].v[0]].p,
						vertices[triangles[aTriSurPo[itsp].first].v[1]].p,
						vertices[triangles[aTriSurPo[itsp].first].v[2]].p))
					{
						// Found the projected point, break here
						GC[0] = projectedPoint.x;
						GC[1] = projectedPoint.y;
						GC[2] = projectedPoint.z;

						isProjectedPointFound = true;

						break;
					}
				}

				if (isProjectedPointFound)
				{
					//moves to the barycenter of its 1-ring
					vertices[ipo].p.x = GC[0];
					vertices[ipo].p.y = GC[1];
					vertices[ipo].p.z = GC[2];
				}
			}
		}

		void InsertVerticesOnRefreshedMesh(std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			std::vector<STri2D> &triangles)
		{
			//newVertexIndices.clear();

			std::vector< std::pair<int, int> > aFailedEdges;

			// Find the edges longer than a threshold in the deformed vertices
			for(unsigned int itr = 0; itr < updatedVertexIndices.size(); itr++)
			{
				std::vector< std::pair<unsigned int,unsigned int>> neighborTriList;
				GetTriAryAroundPoint(updatedVertexIndices[itr], neighborTriList, vertices, triangles);

				for(unsigned int i = 0; i < neighborTriList.size(); i++)
				{	
					for(unsigned int itre=0; itre<3; itre++)
					{
						int i0 = triangles[neighborTriList[i].first].v[noelTriEdge[itre][0]];  //vertex 0 on edge itre
						int i1 = triangles[neighborTriList[i].first].v[noelTriEdge[itre][1]];  //vertex 1 on edge itre

						DirectX::Vector3 normal0 = vertices[i0].n;
						DirectX::Vector3 normal1 = vertices[i1].n;
						float temp_length = Distance(vertices[i0].p, vertices[i1].p);
						float lengthThreshold = 0.08f;
						float angleThreshold = 0.10f;
						if(Dot(normal0, normal1) < angleThreshold || temp_length > lengthThreshold)
						{
							if (i0 < i1)
							{
								aFailedEdges.push_back(std::make_pair(i0, i1));
							} else {
								aFailedEdges.push_back(std::make_pair(i1, i0));
							}
						}	
					}
				}
			}

			// Remove duplicates in the long edge list
			std::set<std::pair<int, int>> aFailedEdgesSet(aFailedEdges.begin(), aFailedEdges.end());
			aFailedEdges.assign(aFailedEdgesSet.begin(), aFailedEdgesSet.end());

			// Insert vertices in the middle of the edges
			// but based on their undeformed positions
			for(unsigned int itr=0; itr< aFailedEdges.size(); itr++)
			{
				int i0 = aFailedEdges[itr].first;
				int i1 = aFailedEdges[itr].second;

				// Create a vertex at the middle point of the two vertices
				int ipo_ins = vertices.size();
				CVertex3D po3d;
				po3d.p = 
					(vertices[i0].p + vertices[i1].p) * 0.5;
				vertices.push_back( po3d );

				//// Interpolate the joint weights for animation
				//unsigned int nJoints = min(vertices[i0].mJointWeights.size(), vertices[i1].mJointWeights.size());
				//for (unsigned int iJoint = 0; iJoint < nJoints; iJoint++)
				//{
				//	//float weighti0 = mpVertexJointWeights->at(i0 * mNumJoints + iJoint);
				//	//float weighti1 = mpVertexJointWeights->at(i1 * mNumJoints + iJoint);
				//	assert(iJoint >= 0 && iJoint < vertices[i0].mJointWeights.size());
				//	float weighti0 = vertices[i0].mJointWeights[iJoint];
				//	assert(iJoint >= 0 && iJoint < vertices[i1].mJointWeights.size());
				//	float weighti1 = vertices[i1].mJointWeights[iJoint];
				//	float weight = (weighti0 + weighti1) / 2.0;
				//	vertices[vertices.size() - 1].mJointWeights.push_back(weight);
				//}
				// Interpolate the colors
				vertices[vertices.size() - 1].color.x = (vertices[i0].color.x + vertices[i1].color.x) / 2.0;
				vertices[vertices.size() - 1].color.y = (vertices[i0].color.y + vertices[i1].color.y) / 2.0;
				vertices[vertices.size() - 1].color.z = (vertices[i0].color.z + vertices[i1].color.z) / 2.0;
				vertices[vertices.size() - 1].color.w = (vertices[i0].color.w + vertices[i1].color.w) / 2.0;

				// Find the two triangles this long edge is in
				// Add the new mid-point vertex in the mesh, and split the two triangles
				// In boundary edges, there is only one triangle it's in
				int triIndex;
				int edgeIndex;
				for (unsigned int i = 0; i < 2; i++)
				{
					// Find the triangle and edge information the two vertices share
					FindSharedTriEdgeIndex(i0, i1, triIndex, edgeIndex, vertices, triangles);
					if (triIndex < (int)triangles.size() && triIndex >= 0 && edgeIndex < nEdTri && edgeIndex >= 0)
					{
						InsertPoint_ElemEdge(ipo_ins, triIndex, edgeIndex, vertices, triangles);
					}
				}

				// Add this index to the added vertex index array
				updatedVertexIndices.push_back(ipo_ins);
			}
		}

		void Remesh(std::vector<int> &updatedVertexIndices,
			std::vector<CVertex3D> &vertices,
			std::vector<STri2D> &triangles)
		{	
			if (updatedVertexIndices.empty())
			{
				return;
			}

			// Insert appropriate vertices	
			InsertVerticesOnRefreshedMesh(updatedVertexIndices, vertices, triangles);

			Smooth(updatedVertexIndices, vertices, triangles);

			// Update normals of changed vertices
			UpdateNormals(updatedVertexIndices, vertices, triangles);

#ifdef _DEBUG
			CheckTri(triangles);
			CheckTri(vertices, triangles);
#endif // #endif _DEBUG
		}
	}
}

#pragma warning(pop)