#include "stdafx.h"

#include "StaticModel.h"

//static member for mesh
//ID3D11VertexShader *StaticModelBase::s_pVertexShader;
//ID3D11PixelShader *StaticModelBase::s_pPixelShader;
//ID3D11GeometryShader *StaticModelBase::s_pGeometryShader;
//ID3D11InputLayout *StaticModelBase::s_pInputLayout;
D3D11_PRIMITIVE_TOPOLOGY StaticModelBase::s_PrimitiveType = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
//bool StaticModelBase::s_IsInitialized;
UINT StaticModelBase::s_ConstantSlot = 1;
