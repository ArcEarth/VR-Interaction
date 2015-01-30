#pragma once
#include "editingtoolsinterface.h"
#include "DynamicMetaBallModel.h"
#include "SnapFilter.h"
#include "HumanSkeleton.h"
#include "FloatingText.h"

namespace EditingTools
{

	class ManipulationManager :
		public IDoubleHandTool
	{
	protected:
		enum Modes
		{
			Intial,
			Rotation,
			Scale,
			Translation,
		};
	public:
		ManipulationManager(void);
		ManipulationManager(Geometrics::DynamicMetaBallModel *pTarget ,const Kinematics::HumanSkeleton *pPlayer ,IFloatingText* pTextBlock);

		virtual ~ManipulationManager(void);

		virtual void Start();
		virtual void Finish();
		virtual void Abort();

		virtual void Edit(_In_reads_(8) const DirectX::Vector3* EditingPoints);

		virtual bool IsActive() const {return m_IsWorking;}

		virtual float Radius() const { return 0.0f; }
		virtual bool SetRadius(float Radius) { return false; }
		virtual bool TurnRadius(float deltaRadius) { return false; }

		virtual DirectX::IRenderObject* Visualization() {return nullptr;}

		void setTarget(Geometrics::DynamicMetaBallModel *_pTarget)
		{
			if (m_IsWorking) return;
			pTarget = _pTarget;
		}

		Geometrics::DynamicMetaBallModel * getTarget()
		{
			return pTarget;
		}

		__declspec(property(get = getTarget , put = setTarget)) 
		Geometrics::DynamicMetaBallModel * Target;

		/// <summary>
		/// Rotates the target with a specific quaternion.
		/// </summary>
		/// <param name="qRotation">The rotation quaternion.</param>
		void RotateTarget(DirectX::FXMVECTOR qRotation , DirectX::FXMVECTOR vRotationCenter);

		/// <summary>
		/// Uniform Scales the target.
		/// </summary>
		/// <param name="scale">The scale factor.</param>
		void ScaleTarget(float scale , DirectX::FXMVECTOR vScaleCenter);

		/// <summary>
		/// Translates the target.
		/// </summary>
		/// <param name="deltaY">The delta in Y axis.</param>
		void TranslateTarget(DirectX::FXMVECTOR vDelta);

		/// <summary>
		/// Grounds the target. Force the model's lowest part is EQUAL the given height value.
		/// </summary>
		/// <param name="groundHeight">Height of the ground.</param>
		void GroundTarget(float groundHeight);

		/// <summary>
		/// Up Grounds the target. Force the model's lowest part is LARGER than the given height value.
		/// </summary>
		/// <param name="groundHeight">Height of the ground.</param>
		void UpGroundTarget(float groundHeight);

		void SetGroundHeight(float groundHeight);

	protected:

		void HandleManipulate(DirectX::Vector3& HandleVectorLtoR , DirectX::Vector3& HandleCenter);

	private:
		bool m_IsWorking;
		Modes Mode;

		DirectX::Vector3				OriginHandleVector;
		DirectX::Vector3				OriginHandleCenter;
		DirectX::Vector3				OriginCenter;
		DirectX::Vector3				SnapedRotationAxis;
		DirectX::Vector3				SnapedTranslationDirection;
		Kinematics::KinematicsData		OriginData;
		float							OriginHeight;
		float							PreviousScale;
		float							GroundHeight;
		float							SnapedToPlayerScale;
		DirectX::Vector3				PreviousCenter;
		bool							ForceGroundFlag;

		Filters::SnapFilter<float>		ScaleFilter;
		
		Geometrics::DynamicMetaBallModel	*pTarget;
		const Kinematics::HumanSkeleton		*pPlayer;
		IFloatingText						*pTextIndicator;
	};
}