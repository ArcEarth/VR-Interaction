#pragma once
#include "editingtoolsinterface.h"
#include "HumanSkeleton.h"


namespace EditingTools
{
	class DriveManager :
		public IEditingTool
	{
	public:
		DriveManager(const std::shared_ptr<Geometrics::DynamicMetaBallModel> &pTargetModel , const std::shared_ptr<Kinematics::HS_MB_Adaptor> pHumanAdaptor , unsigned int HandSwitch);
		~DriveManager(void);

		virtual void Start();
		virtual void Finish();
		virtual void Abort();

		bool IsAttached() const;
		bool Attach();
		void Detach();

		virtual void Edit(const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&,const DirectX::Vector3&);

		virtual bool IsActive() const;

		// the Interface for rendering
		virtual DirectX::IRenderObject* Visualization();

	private:
		std::shared_ptr<Geometrics::DynamicMetaBallModel>	m_pTargetModel;
		std::shared_ptr<Kinematics::HS_MB_Adaptor>			m_pAdptor;
		//const Kinematics::HumanSkeleton*			m_pPlayer;
		unsigned int			m_HandSwitch;
	};
} 
