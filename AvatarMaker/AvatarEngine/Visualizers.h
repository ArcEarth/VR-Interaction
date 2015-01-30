#pragma once
#include "stdafx.h"
#include "ModelInterface.h"
#include "KinematicsSkeleton.h"
#include "Carmera.h"
#include "DynamicMetaBallModel.h"
#include "CompositeFlag.h"

class SkeletonViewer
	: public DirectX::IRenderObject
{

public:
	~SkeletonViewer(void);
	SkeletonViewer(ID3D11DeviceContext* pContext ,const ICamera* pCamera ,const Kinematics::IndexedSkeleton* pSkeleton = nullptr , const DirectX::XMFLOAT4A* _ColorSchedule = nullptr );
	virtual void Render( ID3D11DeviceContext *pContext );
public:
	DirectX::Vector4 Color;
	const DirectX::XMFLOAT4A* ColorSchedule;
	CompositeFlag<Kinematics::State> SelectFlag;

	__declspec(property(get = GetSkeleton , put = SetSkeleton) ) 
		const Kinematics::IndexedSkeleton* Skeleton;

	const Kinematics::IndexedSkeleton* GetSkeleton() const {return m_pSkeleton;}
	void SetSkeleton(const Kinematics::IndexedSkeleton* pSkeleton) {m_pSkeleton = pSkeleton;}

public:
	static std::function<void()> CustomStateFunc;
private:

	const Kinematics::IndexedSkeleton* m_pSkeleton;
	class Viewer;
	std::unique_ptr<Viewer> m_pViewer;
};

class MetaballViewer
	: public DirectX::IRenderObject
{
public:
	~MetaballViewer(void);
	MetaballViewer(ID3D11DeviceContext* pContext ,const ICamera* pCamera);

	virtual void Render( ID3D11DeviceContext *pContext );

	DirectX::Vector4 Color;

	const Geometrics::DynamicMetaBallModel *Target;

	class Viewer;
	std::unique_ptr<Viewer> m_pViewer;
};