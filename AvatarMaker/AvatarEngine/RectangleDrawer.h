#pragma once
#ifndef DX_HELPER_H
#define DX_HELPER_H
#include <d3d11_1.h>

// Helper class to draw a colored rectangle
class RectDrawer {
	public:
		RectDrawer();
		~RectDrawer();
		
		bool Initialize(ID3D11Device *pDevice);
		//Draw the recangle using the pixel as unit value
		void DrawRect_Pixel(ID3D11DeviceContext *pContext, float left, float top, float right, float bottom , DirectX::FXMVECTOR Color);

		void DrawRect_ProjectionCoor(ID3D11DeviceContext *pContext, float left, float top, float right, float bottom , DirectX::FXMVECTOR Color);
		void DrawRect_ProjectionCoor_CenterExtent(ID3D11DeviceContext *pContext ,float Center_X, float Center_Y, float Extent_X, float Extent_Y , DirectX::FXMVECTOR Color);

protected:

		struct ShaderConstants {
			DirectX::XMFLOAT4X4 TransformMatrix;
			DirectX::XMFLOAT4 Color;
		};

		void Draw(ID3D11DeviceContext *pContext , const ShaderConstants& Constants);

		ID3D11PixelShader *m_pPixelShader;
		ID3D11VertexShader *m_pVertexShader;
		ID3D11Buffer *m_pBuffer;
		bool m_inited;
};
#endif