#include "Model.h"
#include "stride_iterator.h"
using namespace DirectX::Scene;
using namespace DirectX;
using namespace std;

void DirectX::Scene::Mesh::Render(ID3D11DeviceContext *pContext)
{
	if (pInputLayout)
		pContext->IASetInputLayout(pInputLayout.Get());

	// Set the input assembler stage
	auto vb = pVertexBuffer.Get();
	UINT vbStride = VertexStride;
	UINT vbOffset = 0;

	pContext->IASetVertexBuffers(0, 1, &vb, &vbStride, &vbOffset);


	if (pEffect)
		pEffect->Apply(pContext);

	pContext->IASetPrimitiveTopology(PrimitiveType);

	if (pIndexBuffer)
	{
		pContext->IASetIndexBuffer(pIndexBuffer.Get(), IndexFormat, 0);
		pContext->DrawIndexed(IndexCount, StartIndex, VertexOffset);
	}
	else
	{
		pContext->Draw(VertexCount, VertexOffset);
	}
	return;
}


template<class _TVertex, class _TIndex>
void DirectX::Scene::GeometryMesh<_TVertex, _TIndex>::UpdateRenderMesh()
{
}
