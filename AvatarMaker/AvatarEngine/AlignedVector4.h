#include <DirectXMath.h>
#include "AlignedNew.h"

namespace DirectX
{

	struct Vector4A : public XMFLOAT4A , public AlignedNew<Vector4A>
	{
		inline Vector4A(){}
		inline  ~Vector4A(){}

		inline explicit Vector4A(FXMVECTOR vtr){
			XMStoreFloat4A(this,vtr);
		}

		inline Vector4A(float x,float y,float z,float w)
			: XMFLOAT4A(x,y,z,w)
		{}

		inline explicit Vector4A(_In_reads_(4) const float* pArray)
			: XMFLOAT4A(pArray)
		{}

		inline explicit Vector4A(const XMFLOAT3 &vtr3)
			: XMFLOAT4A(vtr3.x,vtr3.y,vtr3.z,1.0f)
		{}

		inline explicit Vector4A(const XMFLOAT3 &vtr3 , float _w)
			: XMFLOAT4A(vtr3.x,vtr3.y,vtr3.z,_w)
		{}

		inline void Set(float _x,float _y,float _z,float _w){
			x =_x; y=_y; z=_z; w=_w;
		}

		inline Vector4A& operator = (FXMVECTOR rhs){
			XMStoreFloat4A(this,rhs);
			return *this;
		}

		inline Vector4A& operator = (const XMFLOAT4 &rhs){
			*this = rhs;
			return *this;
		}

		inline operator XMVECTOR () const{
			return XMLoadFloat4A(this);
		}

		inline operator float* () {
			return (float*)this;
		}

		inline operator const float* () const{
			return (float*)this;
		}
	};

}
