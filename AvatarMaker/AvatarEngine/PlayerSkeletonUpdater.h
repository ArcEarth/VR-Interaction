#pragma once
#include "nuistreamviewer.h"
#include "Player.h"

class PlayerSkeletonUpdater
{
public:
	PlayerSkeletonUpdater(void);
	~PlayerSkeletonUpdater(void);

	std::vector<Player*>& getPlayers() { return m_pPlayers; }
	__declspec(property(get = getPlayers)) 
		std::vector<Player*>& Players;

	bool UpdatePlayerSkeleton(NUI_SKELETON_FRAME* pFrame);

protected:
	void UpdatePlayerSkeleton(Player* player , NUI_SKELETON_DATA& skeleton , long long timeStamp);
	void UpdatePlayerSkeletonTracked(Player* player , NUI_SKELETON_DATA& skeleton);
	void UpdatePlayerSkeletonPositionOnly(Player* player , NUI_SKELETON_DATA& skeleton);

public:
	double				StickyTime;

protected:
	std::vector<Player*> m_pPlayers;
	DirectX::XMFLOAT4X4 m_GroundingMatrix;
	TIMER				m_StickyTimer;
	class Implement;
	std::unique_ptr<Implement> m_pImplement;
};

